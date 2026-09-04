function T = sweep_kth(kvals)
%SWEEP_KTH  Peak commanded wheel speed against the heading-task gain.
%
%   The 120 rpm firmware ceiling (MAX_RPM in MatlabPIDLoop.ino) is not
%   enforced anywhere between the Simulink block and the motors: a command
%   past it extrapolates through Arduino's integer map() and is handed
%   straight to motorOn(). So the gain has to be chosen, not discovered.
%
%   The peak is set by the heading CAPTURE TRANSIENT, not by the lap time,
%   which is why slowing the orbit barely helps and this gain does.
%
%   SWEEP_KTH runs the default set. SWEEP_KTH(KVALS) runs your own.
%   Requires the model to be in its widened state: apply_stage1('widen').
%   Simulation only -- EnableHardware and EnableMotive are forced to 0.

if nargin < 1 || isempty(kvals)
    kvals = [4.0 2.0 1.0 0.4 0.2];
end

here = fileparts(mfilename('fullpath'));
mdl  = 'ROME_Main';
addpath(here, fullfile(here, 'Ground Vehicle'), fullfile(here, 'Optitrack'));

evalin('base', sprintf('run(''%s'');', fullfile(here, 'define_constants.m')));
assignin('base', 'EnableHardware', 0);
assignin('base', 'EnableMotive',   0);

stage1_guard(mdl);   % refuse rather than force-close under an open editor
load_system(fullfile(here, [mdl '.slx']));
stopFcn = get_param(mdl, 'StopFcn');
set_param(mdl, 'StopFcn', '');

ikPort = get_param([mdl '/InverseKinematics'], 'PortHandles').Outport(1);
set_param(mdl, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');
set_param(ikPort, 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', ...
                  'DataLoggingName', 'wheel_rpm_block');

pk   = zeros(numel(kvals), 1);
eth  = zeros(numel(kvals), 1);
rmse = zeros(numel(kvals), 1);

for i = 1:numel(kvals)
    assignin('base', 'k_th', kvals(i));
    out = sim(mdl, 'SimulationMode', 'normal', 'ReturnWorkspaceOutputs', 'on');

    pk(i) = max(abs(out.logsout.get('wheel_rpm_block').Values.Data), [], 'all');

    q  = squeeze(out.q_sim.Data).';
    qd = squeeze(out.q_dot_sim.Data).';
    qr = squeeze(out.q_des.Data).';
    t  = out.q_sim.Time;
    if size(q, 2) ~= 3, q = q.'; qd = qd.'; qr = qr.'; end
    ss = t >= 5;
    rmse(i) = sqrt(mean(vecnorm(q(ss,1:2) - qr(ss,1:2), 2, 2).^2));
    thr = atan2(-q(:,2), -q(:,1));
    e   = atan2(sin(q(:,3)-thr), cos(q(:,3)-thr));
    eth(i) = sqrt(mean(e(ss).^2));
end

set_param(mdl, 'StopFcn', stopFcn);
close_system(mdl, 0);

T = table(kvals(:), pk, eth, rmse, pk < 120, ...
    'VariableNames', {'k_th', 'peak_rpm', 'heading_err_rms_rad', 'path_rmse_m', 'under_120'});

fprintf('\n  heading-task gain sweep, ceiling = 120 rpm\n');
fprintf('  ------------------------------------------------------------\n');
fprintf('   k_th   peak rpm   heading err (rad)   path RMSE (m)   verdict\n');
for i = 1:numel(kvals)
    fprintf('  %5.2f   %8.1f   %17.4f   %13.3e   %s\n', kvals(i), pk(i), ...
        eth(i), rmse(i), tern(pk(i) < 120, 'ok', 'OVER'));
end
fprintf('  ------------------------------------------------------------\n');
fprintf('  Lower k_th costs heading accuracy and buys wheel-speed margin.\n');
fprintf('  Path tracking is unaffected: that is the whole point of putting\n');
fprintf('  the heading task in Q instead of in a third row of A.\n\n');
end


function s = tern(c, a, b)
if c, s = a; else, s = b; end
end
