function out = plot_9dof(out)
%PLOT_9DOF  Run a case study and plot it with the repository's own plotting.
%
%   plotting.m is reused unchanged. It expects the five base-frame signals
%   the 3-DOF model logged,
%
%       q_des  q_sim  q_track  q_dot_des  q_dot_track      each 3 x N
%
%   which do not exist at 9 DOF: there is no commanded base trajectory any
%   more. The reference is the END EFFECTOR, and the base goes wherever the
%   null space puts it.
%
%   What does carry over is the comparison plotting.m draws — desired against
%   achieved, in position, heading, velocity and error — applied to the end
%   effector. So the five signals are rebuilt on the end effector as
%
%       (x, y, theta)  ->  (EE x, EE y, EE yaw)
%
%   and plotting.m is then called as it stands. Its central disc, drawn at
%   the origin, is the chief spacecraft in the RPO scenarios.
%
%   Nothing is duplicated: every figure plotting.m already draws is left to
%   it, and only the quantities that have no 3-DOF equivalent are added here
%   — wheel speeds against the firmware ceiling, the six joint angles, and
%   the two solve health signals.
%
%   PLOT_9DOF() runs the model first. PLOT_9DOF(OUT) reuses a run.

here = fileparts(mfilename('fullpath'));
cd(here); addpath(here, fullfile(here,'Ground Vehicle'), fullfile(here,'Optitrack'));
mdl = 'ROME_9DOF';

if nargin < 1 || isempty(out)
    evalin('base', sprintf('run(''%s'');', fullfile(here,'define_constants.m')));
    % F12: run mode 1 only, asserted (an assignin of EnableHardware would be overwritten by the PostLoadFcn)
    assert_sim_mode('plot_9dof', 'base');
    if bdIsLoaded(mdl), close_system(mdl,0); end
    load_system(fullfile(here,[mdl '.slx']));
    try, assert_sim_mode('plot_9dof', 'base'); catch em_, close_system(mdl,0); rethrow(em_); end   % after the PostLoadFcn
    set_param(mdl,'StopFcn','');            % plotting is called from here instead
    out = sim(mdl,'SimulationMode','normal','ReturnWorkspaceOutputs','on');
    close_system(mdl,0);
end

sc  = evalin('base','scenario');
els = evalin('base','elements');   mu_ = evalin('base','mu');
ts  = evalin('base','time_scale'); ds  = evalin('base','dist_scale');
zw  = evalin('base','z_work');     par = evalin('base','par');
[~,~,~,~,nm] = scenario_defaults(sc);

t  = out.q_cmd.Time;
q  = as_rows(squeeze(out.q_cmd.Data));          % N x 9 commanded configuration
qd = as_rows(squeeze(out.q_dot_cmd.Data));      % N x 9 commanded rates
N  = numel(t);

% ---- rebuild the five signals plotting.m wants, on the end effector ----
ee_des = zeros(3,N);   ee_ach = zeros(3,N);
ve_des = zeros(3,N);   ve_ach = zeros(3,N);
for k = 1:N
    [p, u, V] = EndEffectorTrajectory(t(k), sc, els, mu_, ts, ds, zw, par);
    ee_des(:,k) = [p(1); p(2); 2*atan2(u(4), u(1))];    % x, y, commanded yaw
    ve_des(:,k) = [V(1); V(2); V(6)];
    [pe, Re] = ik9_fk(q(k,:).');
    ee_ach(:,k) = [pe(1); pe(2); atan2(Re(2,1), Re(1,1))];
    Vk = ik9_jac(q(k,:).') * qd(k,:).';                 % achieved EE twist
    ve_ach(:,k) = [Vk(1); Vk(2); Vk(6)];
end

mk = @(D) timeseries(reshape(D, 3, 1, N), t);
out.q_des       = mk(ee_des);
out.q_sim       = mk(ee_ach);
out.q_track     = mk(ee_ach);   % no separate measurement with EnableMotive = 0
out.q_dot_des   = mk(ve_des);
out.q_dot_track = mk(ve_ach);

assignin('base','out', out);
evalin('base','plotting');      % the repository's own figures, unchanged

% ---- only what has no 3-DOF equivalent -------------------------------
w   = as_rows(squeeze(out.wheel_speeds_rpm.Data));
tau = as_rows(squeeze(out.tau_arm.Data));
mn  = squeeze(out.manip.Data);
rs  = squeeze(out.resid.Data);

figure('Name', sprintf('9-DOF actuators and solve health - %s', nm));

subplot(2,2,1)
plot(t, w, 'LineWidth', 1.1); hold on
yline( 120, 'r--', 'MAX\_RPM', 'LineWidth', 1.2);
yline(-120, 'r--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('rpm'); grid on
title('Commanded wheel speeds');
legend('w_1','w_2','w_3','w_4','Location','best');

subplot(2,2,2)
plot(t, rad2deg(q(:,4:9)), 'LineWidth', 1.1);
xlabel('Time (s)'); ylabel('deg'); grid on
title('Commanded joint angles');
legend('q_1','q_2','q_3','q_4','q_5','q_6','Location','best');

subplot(2,2,3)
yyaxis left;  plot(t, mn, 'LineWidth', 1.3); ylabel('manipulability');
hold on; yline(0.10, ':', 'damping threshold');
yyaxis right; semilogy(t, max(rs, eps), 'LineWidth', 1.1); ylabel('constraint residual');
xlabel('Time (s)'); grid on
title('Solve health');

subplot(2,2,4)
plot(t, tau, 'LineWidth', 1.1);
xlabel('Time (s)'); ylabel('N m'); grid on
title('Arm joint torque (margin check only)');
legend('\tau_1','\tau_2','\tau_3','\tau_4','\tau_5','\tau_6','Location','best');

fprintf('\n  %s: %d figures from plotting.m on end-effector quantities,\n', nm, 12);
fprintf('  plus one with the wheel, joint, health and torque traces.\n\n');
end


function A = as_rows(A)
if size(A,1) < size(A,2), A = A.'; end
end
