function R = check_9dof()
%CHECK_9DOF  Acceptance gate for ROME_9DOF.slx. Simulation only, no hardware.
%
%   Asserts run_mode = 1 (EnableHardware = 0, fake camera) before and after
%   load_system (assert_sim_mode:notsim) and forces EnableArm = 0, so nothing reaches a serial
%   port. ERRORS on failure -- a check that prints FAIL and exits 0 gates
%   nothing.
%
%   Thresholds and where each comes from:
%     constraint residual < 1e-10   test_uk_block measured 1.12e-12 at 20 Hz
%     peak wheel  < max_rpm         define_constants (80); firmware 120. NOTHING
%                                   saturates between the block and the
%                                   motors (defect E4), so the ceiling is hard
%     min s       > 0.30            s = sigma_min(A M^-1/2) is the smallest
%                                   singular value of the matrix the solve
%                                   pseudo-inverts, so it measures the health
%                                   of the quantity that actually fails. The
%                                   three hardware runs of 16 September that
%                                   diverged started at s = 0.05 to 0.16; the
%                                   five case studies hold s >= 0.67
%                                   throughout. 0.30 sits between the two,
%                                   and no run has been observed in between,
%                                   so the exact value is not yet pinned by
%                                   data -- it is a margin, not a measured
%                                   boundary. Manipulability is printed but
%                                   NOT gated: it read 0.367 at a start whose
%                                   run diverged.
%     EE RMSE     < 1.5 * v_max * dt  transport-delay bound, derived below
%     z_min       >= clearance_margin   lowest arm link point over the WHOLE run (T_stop), 'paper'
%                                   geometry (clearance_9dof on q_cmd). clearance_margin = 0.05 m is
%                                   in define_constants.m, [Claude's call - author to confirm]. z = 0
%                                   is the model's base plate / floor / table plane; values shift
%                                   one-for-one with the unmeasured 0.20 m mount height. xy_min
%                                   (signed distance of the elbow-to-flange points to the base
%                                   square) and the 'ar3' values are printed, NOT gated (the 'ar3'
%                                   gate belongs to Plan 2).
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
assignin('base','EnableArm',0);
% RUN MODE (focus plan F12). This simulates run mode 1 only. The flags come from run_mode in define_constants.m,
% and load_system's PostLoadFcn re-runs define_constants, so an assignin of the flags would be overwritten: the mode is
% ASSERTED instead (assert_sim_mode, shared by every runner), here and again after load_system.
assert_sim_mode('check_9dof', 'base');

fprintf('\n=======================================================\n');
fprintf('  9-DOF VERIFICATION  -  simulation only, no hardware\n');
fprintf('=======================================================\n');

%% STATIC CHECKS. One evaluation each, before any simulation. Both would
% have caught a real bug on 22 September 2026 that a full run only showed
% as "89 rpm and 36 mm during the hold" (the hold was replaced by a smooth
% onset on 24 September 2026; the checks keep their purpose):
%   1. the reference starts from rest (zero twist and twist rate at t = 0)
%      and its velocity and acceleration are continuous through the end of
%      the onset, so the constraint is never handed a step;
%   2. a robot at rest on its exact seed, facing the t = 0 reference, does
%      not move: one solve step must return ~0 rpm and ~0 displacement.
% The second one is the formulation's own prediction of the start, so any
% posture, gravity or convention error shows here without a run.
sc_  = evalin('base','scenario');  pr_ = evalin('base','par');
els_ = evalin('base','elements');  mu0 = evalin('base','mu');
ts_  = evalin('base','time_scale'); ds_ = evalin('base','dist_scale');
zw_  = evalin('base','z_work');    dt_ = evalin('base','dt_9dof');
tt_  = evalin('base','tool_tilt_deg');   % tool tilt (deg): the reference is rebuilt with it below
q0_  = evalin('base','q0_9dof');
r_ = evalin('base','r'); l_ = evalin('base','l'); al_ = evalin('base','alphas');
[p0_,u0_,V0_,A0_] = EndEffectorTrajectory(0, sc_, els_, mu0, ts_, ds_, zw_, pr_, tt_);
T_ON_ = evalin('base','T_ON');   % onset length (s), from define_constants (EndEffectorTrajectory.m keeps its own literal, tests/test_run_length.m)
[~,~,Vm_,Am_] = EndEffectorTrajectory(T_ON_ - 1e-6, sc_, els_, mu0, ts_, ds_, zw_, pr_, tt_);
[~,~,Vp_,Ap_] = EndEffectorTrajectory(T_ON_ + 1e-6, sc_, els_, mu0, ts_, ds_, zw_, pr_, tt_);
onset_ok = norm([V0_;A0_]) < 1e-12 && norm(Vp_ - Vm_) < 1e-6 && norm(Ap_ - Am_) < 1e-6;
clear ukd_current
[q1_,~,w1_] = ukd_current(q0_, zeros(9,1), p0_, u0_, V0_, A0_, dt_, 1, r_, l_, al_);
e1_ = norm(ik9_fk(q1_) - p0_);  w1_ = max(abs(w1_));
still_ok = e1_ < 1e-4 && w1_ < 3;
fprintf('\n  static: %s reference starts from rest and is continuous through the onset\n', tag(onset_ok));
fprintf('  static: %s at rest on the seed the solve stays put  (1 step: %.1e m, %.2f rpm)\n', ...
        tag(still_ok), e1_, w1_);
if ~(onset_ok && still_ok)
    error('check_9dof:static', 'static checks failed; the run is not worth simulating');
end

if bdIsLoaded(mdl), close_system(mdl,0); end
load_system(fullfile(here,[mdl '.slx']));
try, assert_sim_mode('check_9dof', 'base'); catch em_, close_system(mdl,0); rethrow(em_); end   % after the PostLoadFcn
set_param(mdl,'StopFcn','');
% The model's StopTime is 'T_stop' (define_constants): the whole path, onset included.
assert(strcmp(get_param(mdl,'StopTime'), 'T_stop'), 'check_9dof:stoptime', ...
       'model StopTime is ''%s'', expected ''T_stop''', get_param(mdl,'StopTime'));
out = sim(mdl,'SimulationMode','normal','ReturnWorkspaceOutputs','on');
fprintf('  run flags (read back after the sim): EnableMotive %g, motive_source %s, motive_emulate_noise %g, sensor_model %g\n', ...
        evalin('base','EnableMotive'), evalin('base','motive_source'), evalin('base','motive_emulate_noise'), evalin('base','sensor_model'));
close_system(mdl,0);

% squeeze leaves an m x N array for an m x 1 signal logged over N steps.
% Transpose to N x m; reshape would interleave the channels.
g = @(n) as_rows(squeeze(out.(n).Data));
q      = g('q_cmd');
qm     = g('q_meas');        % what the controller read (SensorModel output), N x 9
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
tt  = evalin('base','tool_tilt_deg');   % tool tilt (deg)
sc  = evalin('base','scenario');  pr = evalin('base','par');
N = numel(t); ep = zeros(N,1); epl = zeros(N,1); dt = t(2) - t(1);
for k = 1:N
    p_ref = EndEffectorTrajectory(t(k),      sc, els, mu_, ts, ds, zw, pr, tt);
    p_nxt = EndEffectorTrajectory(t(k) + dt, sc, els, mu_, ts, ds, zw, pr, tt);
    pe    = blk_fk9(q(k,:).');
    ep(k)  = norm(pe - p_ref);   % same sample: dominated by the one-step lead
    epl(k) = norm(pe - p_nxt);   % lead-corrected: command against the reference one sample ahead
end
ss = t >= t(end)/4;          % past the capture transient

% Two steady errors. q_cmd(k) is produced from q_meas = q_cmd(k-1) and the
% reference at t(k), so at steady state FK(q_cmd(k)) sits on p_ref(t(k)+dt):
% the command LEADS the reference by one sample. The same-sample RMSE
% therefore equals |v|*dt by construction and says nothing about the solve
% (independent audit of 23 Sep 2026, finding B2). The lead-corrected RMSE is
% the number that measures the controller; the same-sample one is reported
% as the sampling lead and gated only for boundedness.
R.rmse_ee      = sqrt(mean(ep(ss).^2));    % sampling lead, m
R.rmse_ee_lead = sqrt(mean(epl(ss).^2));   % lead-corrected steady error, m
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
mr = evalin('base','max_rpm');
o = R.peak_rpm  < mr;    f=f+~o; fprintf('  %s wheel speed under ceiling peak %.1f rpm (max_rpm %g, firmware 120)\n', tag(o), R.peak_rpm, mr);
% Conditioning of the matrix the solve inverts, along the whole run:
%
%     s = sigma_min(Jc M^-1/2),    ||qddot - a|| <= ||M^-1/2|| ||b - A a|| / s
%
% computed here from the logged configurations, so it gates the run rather
% than only the starting configuration that scenario_seeds screened. The
% threshold is the screening value of scenario_seeds. Manipulability is
% printed for continuity but is NOT gated: it read 0.367 at a start whose
% run diverged, because it weights base and arm columns equally while the
% solve weights them by M^-1.
s_min_gate = 0.30;
s_run = inf;
for k = 1:N
    [Mk, Jk] = mass_at(q(k,:).');
    s_run = min(s_run, sqrt(max(min(eig(Jk*(Mk\Jk.'))), 0)));
end
R.min_s = s_run;
o = R.min_s > s_min_gate;  f = f + ~o;
fprintf('  %s away from singularity     min s %.4f (screening at %.2f), manip %.4f\n', ...
        tag(o), R.min_s, s_min_gate, R.min_manip);
% The command reaches the block one step late by construction: q_cmd returns
% to q_meas through a Unit Delay. At 20 Hz that is 50 ms, so the command
% leads a reference moving at v by about v*dt however good the solve is.
% The same-sample RMSE is gated against that bound with 50 percent margin
% as a BOUNDEDNESS check only (a diverging run fails it); it is not a
% tracking metric. The lead-corrected RMSE above is the tracking metric.
vref = zeros(N,1);
for k = 1:N
    [~,~,Vk] = EndEffectorTrajectory(t(k), sc, els, mu_, ts, ds, zw, pr, tt);
    vref(k) = norm(Vk(1:3));
end
R.lag_bound = max(vref) * (t(2)-t(1));
% With the measurement model on (define_constants, sensor_model = 1) the
% true error also carries the closed-loop response to the camera noise. The
% position rows see e'' + K_D e' + K_P e = -K_P n, n the noise the block
% reads, white with standard deviation sigma per sample at dt. For a
% second-order system driven by white noise of spectral density sigma^2 dt,
% the steady RMS response is sigma sqrt(dt w_p / (4 zeta)) with w_p^2 = K_P
% and zeta = 1 (derived and checked by Monte Carlo in ROME_Derivations.m,
% Section 6c). The yaw noise reaches the end effector through the arm's
% horizontal reach rho, so sigma^2 = sigma_xy^2 + (rho sigma_yaw)^2. The
% bound is the root-sum-square of the sampling lead and that response, with
% the same 50 percent margin as before; with sensor_model = 0 it reduces to
% the lead alone.
sm_ = evalin('base','sensor_model'); sg_ = evalin('base','mocap_sigma'); wp_ = 4;   % w_p (rad/s), rome_uk_block.m / ukd_current.m
pe0_ = ik9_fk(q(1,:).'); rho_ = norm(pe0_(1:2) - q(1,1:2).');   % horizontal reach of the arm at the start (m)
R.noise_bound = (sm_ ~= 0) * sqrt(sg_(1)^2 + sg_(2)^2 + (rho_*sg_(3))^2) * sqrt((t(2)-t(1)) * wp_ / 4);   % two floor axes plus the heading through the reach (companion 6c, 24 Sep evening)
R.lag_gate = 1.5*sqrt(R.lag_bound^2 + R.noise_bound^2);   % the threshold, bound plus 50 pct margin
o = R.rmse_ee < R.lag_gate;  f = f + ~o;
fprintf('  %s end effector bounded      same-sample RMSE %.3e m (limit %.3e; sampling lead %.3e, noise response %.3e)\n', ...
        tag(o), R.rmse_ee, R.lag_gate, R.lag_bound, R.noise_bound);
fprintf('       lead-corrected steady RMSE %.3e m (command vs reference one sample ahead)\n', ...
        R.rmse_ee_lead);
em_ = zeros(N,1);
for k = 1:N
    [p_ref_,~] = EndEffectorTrajectory(t(k), sc, els, mu_, ts, ds, zw, pr, tt);
    em_(k) = norm(ik9_fk(qm(k,:).') - p_ref_);
end
R.rmse_meas = sqrt(mean(em_(ss).^2));
fprintf('       measured-pose RMSE %.3e m (what the controller read, against the reference)\n', R.rmse_meas);
% Every arm command of the run, mapped to firmware degrees the way
% ROMECommand sends it, must sit inside the firmware ranges with the safety
% margin to spare. If it does not, arm_clamp would cap it on the robot and
% the real arm would no longer be where the model believes it is.
sg = evalin('base','arm_sign');  of = evalin('base','arm_offset_deg');
sw = evalin('base','arm_fw_switch'); ot = evalin('base','arm_fw_other');
lo = min(sw, ot);  hi = max(sw, ot);        % per joint, as arm_clamp sorts them
mg = evalin('base','arm_fw_margin_deg');
fw = sg .* rad2deg(q(:,4:9)) + of;                 % N x 6, firmware degrees
R.fw_min = min(fw, [], 1);  R.fw_max = max(fw, [], 1);
R.fw_room = min([R.fw_min - (lo + mg), (hi - mg) - R.fw_max]);
o = R.fw_room >= 0;  f = f + ~o;
fprintf('  %s arm inside firmware range  closest approach %.1f deg to a limit (margin %g)\n', ...
        tag(o), R.fw_room + mg, mg);
fprintf('       fw min %s\n       fw max %s\n', mat2str(round(R.fw_min)), mat2str(round(R.fw_max)));
% Clearance of the arm's link points over the whole run. 'paper' is gated; 'ar3' and xy_min are logged only.
cm_ = evalin('base','clearance_margin');                    % clearance margin (m), define_constants
copt.d = evalin('base','d');  copt.w = evalin('base','w');  % base size (m)
copt.arm_sign = sg;  copt.arm_offset_deg = of;             % model -> firmware map (-, deg)
zp_ = zeros(N,1);  xp_ = zeros(N,1);  za_ = zeros(N,1);  xa_ = zeros(N,1);   % per-sample z_min, xy_min (m), paper then ar3
for k = 1:N
    [zp_(k), xp_(k)] = clearance_9dof(q(k,:).', 'paper', copt);
    [za_(k), xa_(k)] = clearance_9dof(q(k,:).', 'ar3',   copt);
end
[R.z_min, kz_] = min(zp_);  R.xy_min = min(xp_);   % whole-run minima (m), 'paper'; kz_ = sample index (-) of the lowest z
R.z_min_ar3 = min(za_);  R.xy_min_ar3 = min(xa_);  % logged only (m)
o = R.z_min >= cm_;  f = f + ~o;
fprintf('  %s arm clear of the floor    z_min %.4f m at t = %.2f s (margin %g m, whole run, paper geometry)\n', ...
        tag(o), R.z_min, t(kz_), cm_);
fprintf('       xy_min %.4f m (signed distance to the base square, not gated); ar3 logged, not gated: z_min %.4f m, xy_min %.4f m\n', ...
        R.xy_min, R.z_min_ar3, R.xy_min_ar3);
fprintf('       peak arm torque %.2f N m  (no actuator limit on record)\n', R.peak_tau);
fprintf('       base travels to %.2f m from the origin -- clear that space,\n', R.base_reach);
fprintf('       it is NOT the 4.2 m x 4.2 m of the 3-DOF orbit \n');

fprintf('\n=======================================================\n');
if f == 0
    fprintf('  RESULT: PASS\n');
else
    fprintf('  RESULT: FAIL - %d of 6 checks did not meet threshold\n', f);
end
fprintf('=======================================================\n\n');
if f > 0
    error('check_9dof:failed','%d of 6 checks failed; not fit for a hardware run.', f);
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
