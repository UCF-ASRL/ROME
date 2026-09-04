function R = check_9dof()
%CHECK_9DOF  Acceptance gate for ROME_9DOF.slx. Simulation only, no hardware.
%
%   Forces EnableHardware = 0 and EnableArm = 0, so nothing reaches a serial
%   port. ERRORS on failure -- a check that prints FAIL and exits 0 gates
%   nothing.
%
%   Thresholds and where each comes from:
%     constraint residual < 1e-10   test_uk_block measured 1.12e-12 at 20 Hz
%     peak wheel  < 120 rpm         MAX_RPM, MatlabPIDLoop.ino. NOTHING
%                                   saturates between the block and the
%                                   motors (defect E4), so the ceiling is hard
%     min manip   > 0.25            test_uk_block measured 0.3177 from the
%                                   arm_home posture
%     EE RMSE     < 1.5 * v_max * dt  transport-delay bound, derived below
%
%   NOT CHECKED, because the numbers do not exist in the repository:
%     - arm joint limits. AR3Serial carries five competing sets and the
%       clamp inside updateStates is commented out (defect E6). They need
%       re-deriving on the hardware before an assertion here would mean
%       anything.
%     - peak arm torque against actuator capability. tau_arm is reported,
%       but no actuator limit is recorded anywhere to compare it against.

here = fileparts(mfilename('fullpath'));
cd(here); addpath(here, fullfile(here,'Ground Vehicle'), fullfile(here,'Optitrack'));
mdl = 'ROME_9DOF';

evalin('base', sprintf('run(''%s'');', fullfile(here,'define_constants.m')));
assignin('base','EnableHardware',0);
assignin('base','EnableMotive',0);
assignin('base','EnableArm',0);

fprintf('\n=======================================================\n');
fprintf('  9-DOF VERIFICATION  -  simulation only, no hardware\n');
fprintf('=======================================================\n');

if bdIsLoaded(mdl), close_system(mdl,0); end
load_system(fullfile(here,[mdl '.slx']));
set_param(mdl,'StopFcn','');
out = sim(mdl,'SimulationMode','normal','ReturnWorkspaceOutputs','on');
close_system(mdl,0);

% squeeze leaves an m x N array for an m x 1 signal logged over N steps.
% Transpose to N x m; reshape would interleave the channels.
g = @(n) as_rows(squeeze(out.(n).Data));
q      = g('q_cmd');
w      = g('wheel_speeds_rpm');
tau    = g('tau_arm');
mani   = g('manip'); mani = mani(:);
res    = g('resid'); res  = res(:);
t      = out.q_cmd.Time;

% Re-derive the commanded end-effector path and compare against the
% reference, using the SAME generator the model runs.
evalin('base','');   % constants already in base
els = evalin('base','elements'); mu_ = evalin('base','mu');
ts  = evalin('base','time_scale'); ds = evalin('base','dist_scale');
zw  = evalin('base','z_work');
sc  = evalin('base','scenario');  pr = evalin('base','par');
N = numel(t); ep = zeros(N,1);
for k = 1:N
    p_ref = EndEffectorTrajectory(t(k), sc, els, mu_, ts, ds, zw, pr);
    pe    = blk_fk9(q(k,:).');
    ep(k) = norm(pe - p_ref);
end
ss = t >= t(end)/4;          % past the capture transient

R.rmse_ee   = sqrt(mean(ep(ss).^2));
R.max_res   = max(res);
R.peak_rpm  = max(abs(w(:)));
R.min_manip = min(mani);
R.peak_tau  = max(abs(tau(:)));
R.base_reach = max(vecnorm(q(:,1:2),2,2));
R.lap        = t(end);

fprintf('\n  ran %.2f s, %d steps at dt = %.3f s (%.0f Hz)\n', ...
        t(end), N, t(2)-t(1), 1/(t(2)-t(1)));

f = 0;
o = R.max_res   < 1e-10; f=f+~o; fprintf('  %s constraint satisfied      max residual %.2e\n', tag(o), R.max_res);
o = R.peak_rpm  < 120;   f=f+~o; fprintf('  %s wheel speed under ceiling peak %.1f rpm (max 120)\n', tag(o), R.peak_rpm);
o = R.min_manip > 0.25;  f=f+~o; fprintf('  %s away from singularity     min manip %.4f\n', tag(o), R.min_manip);
% The command reaches the block one step late by construction: q_cmd returns
% to q_meas through a Unit Delay. At 20 Hz that is 50 ms, so a reference
% moving at v trails by about v*dt however good the solve is. The threshold
% is that bound with 50 percent margin, derived from the reference actually
% being flown rather than carried over from a slower, smaller test path.
vref = zeros(N,1);
for k = 1:N
    [~,~,Vk] = EndEffectorTrajectory(t(k), sc, els, mu_, ts, ds, zw, pr);
    vref(k) = norm(Vk(1:3));
end
R.lag_bound = max(vref) * (t(2)-t(1));
o = R.rmse_ee < 1.5*R.lag_bound;  f = f + ~o;
fprintf('  %s end effector tracks       steady RMSE %.3e m (delay bound %.3e)\n', ...
        tag(o), R.rmse_ee, R.lag_bound);
fprintf('       peak arm torque %.2f N m  (no actuator limit on record)\n', R.peak_tau);
fprintf('       base travels to %.2f m from the origin -- clear that space,\n', R.base_reach);
fprintf('       it is NOT the 4.2 m x 4.2 m of the 3-DOF orbit \n');

fprintf('\n=======================================================\n');
if f == 0
    fprintf('  RESULT: PASS\n');
else
    fprintf('  RESULT: FAIL - %d of 4 checks did not meet threshold\n', f);
end
fprintf('=======================================================\n\n');
if f > 0
    error('check_9dof:failed','%d of 4 checks failed; not fit for a hardware run.', f);
end
end


function pe = blk_fk9(q)
%BLK_FK9  End-effector position for the commanded configuration.
%   Kept separate so the check does not reach into another file's locals.
[pe, ~] = ik9_fk(q);
end


function s = tag(ok)
if ok, s = '[OK]'; else, s = '[--]'; end
end


function A = as_rows(A)
% Orient a logged signal as one row per time step.
if size(A,1) < size(A,2), A = A.'; end
end
