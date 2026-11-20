function makefail_campaign(ids, group, copyRoot, dryRun)
%MAKEFAIL_CAMPAIGN  Plan 1 Task 7b: break each check on purpose, in a throwaway copy, and record whether it FAILS.
%   The author (19 Nov 2026): "in the past our tests were not actually testing the main thing we wanted".
%   Each break below violates the property one check claims to protect; the check must then fail,
%   naming itself. A break whose check still passes is a FINDING (docs/verification/TEST_REGISTER.md).
%
%   HOW TO RUN BY HAND (MATLAB R2026a; ROME_merged itself is never modified):
%       >> cd <ROME_merged>/tests
%       >> makefail_campaign('list')                          % every break: id, group, target, why
%       >> makefail_campaign({'c9_resid', 'cg6_flag'})        % those breaks; log <date>_task7b_manual.log
%       >> makefail_campaign({'c9_resid'}, 'rerun_c9')        % log docs/verification/<date>_task7b_rerun_c9.log
%       >> makefail_campaign('group:check9dof', 'check9dof')  % every break of one group
%   Arguments: ids (cell of break ids, 'group:<name>', 'all' or 'list'); group (log name, default 'manual');
%   copyRoot (folder for the throwaway copies, default tempdir); dryRun (true: prepare each copy and apply
%   its edits, run nothing; default false).
%
%   For each break: ROME_merged is copied to <copyRoot>/mf_<id>_<time>; the edits are applied to the COPY,
%   each text edit asserted to match exactly once (CLAUDE.md section 7); the MATLAB path is reset to the
%   path at entry minus every ROME_merged folder, plus the copy's folders; `which` must resolve the target
%   inside the copy; the target check runs in try/catch; its error identifier, message and the verdict are
%   logged; the copy is deleted.
%   Verdicts:
%     FAILS         the target errored with the expected identifier and its output/message matches the
%                   expected text, which names the target check
%     DID NOT FAIL  the target passed although the property it protects was broken (a FINDING)
%     WRONG REASON  it errored, but not with the expected identifier and text (another check or an error)
%     SETUP ERROR   the copy could not be prepared (an edit matched 0 or 2+ times, ...); nothing was run
%   The expected text is matched against the target's own output (from the '---- running' line on) and its
%   error message only, never against the header lines this driver prints (Task 7c, after the Task 7b
%   review). The self-test row 'selftest_unrelated' checks that: it runs even in a dry run, must not score
%   FAILS, and the driver errors at the end if it does.
%   The whole command window output goes to docs/verification/<date>_task7b_<group>.log (diary), or to
%   tempdir when <folder above ROME_merged>/docs/verification does not exist (the lab repo; the path is printed). A MATLAB
%   crash (exit 139) ends the launch: the breaks after the last "#### START" line in the log did not run.
%   Flat, no local functions (CLAUDE.md section 5, crash 2). Text edits use "<NL>" for a line break, which
%   is written with the file's own line ending.

if nargin < 1 || isempty(ids), ids = 'list'; end
if nargin < 2 || isempty(group), group = 'manual'; end      % log name suffix (char)
if nargin < 3 || isempty(copyRoot), copyRoot = tempdir; end % folder that receives the throwaway copies (char)
if nargin < 4 || isempty(dryRun), dryRun = false; end      % true: apply the edits only, run no check (logical)
here   = fileparts(mfilename('fullpath'));                  % ROME_merged/tests (char)
merged = fileparts(here);                                    % ROME_merged, the source of every copy (char)
logDir = fullfile(fileparts(merged), 'docs', 'verification');   % verification logs (char)
if ~isfolder(logDir)
    % The lab repo has no <repo>/docs/verification (final review I4, 19 Nov 2026); the review repo does.
    logDir = tempdir;                                        % fallback folder for the log (char)
    fprintf('makefail_campaign: %s does not exist; the log goes to %s\n', ...
            fullfile(fileparts(merged), 'docs', 'verification'), logDir);
end
mdl    = 'ROME_9DOF';                                        % model name (char)

% ---- break table, one row per break --------------------------------------------------------------------
% columns: id | group | target (MATLAB code, run in the copy; TMP = copy folder) | expected error identifier
%          ('' = not checked) | expected text (regexp on the break's output and the error message; names the
%          target check) | pre-actions on the copy | why the break violates the property the check protects
% pre-actions: {'edit', file, old, new}   text edit, exactly one match of old (<NL> = line break)
%              {'copy', src, dst}         copy a file inside the copy
%              {'mat', file, code}        load file into struct D, eval code (must assert), save D back
%              {'code', code}             eval code (must assert); TMP is the copy folder
%              {'slx', code, check}       load the copy's model, eval code, save; reload, eval check == true
%              {'chart', name, old, new}  edit the script of MATLAB Function chart <name>, exactly one match
B = cell(0, 7);

% check_9dof (the acceptance gate), scenario 1. The six run checks are all evaluated before the error, so a
% co-firing check is visible in the log ("[--]" lines).
B(end+1,:) = {'c9_onset', 'check9dof', 'check_9dof()', 'check_9dof:static', '\[--\] reference starts from rest', ...
    {{'edit', 'EndEffectorTrajectory.m', "sgd  = (1 - cos(pi*t/T_ON))/2;", "sgd  = (1 - cos(pi*t/T_ON))/2 + 1e-3;"}}, ...
    'onset clock rate sigma'' is 1e-3 at t = 0 (and steps at T_ON): the reference does not start from rest'};
B(end+1,:) = {'c9_still', 'check9dof', 'check_9dof()', 'check_9dof:static', '\[--\] at rest on the seed', ...
    {{'edit', 'ukd_current.m', "posture = q_meas(3:9);", "posture = zeros(7,1);"}}, ...
    'rest posture of the null-space spring is a literal (zeros), not the start (the 22 Sep bug): one solve step at rest on the seed must move'};
B(end+1,:) = {'c9_stoptime', 'check9dof', 'check_9dof()', 'check_9dof:stoptime', 'StopTime is ''T_scaled''', ...
    {{'slx', "set_param(mdl, 'StopTime', 'T_scaled');", "strcmp(get_param(mdl, 'StopTime'), 'T_scaled')"}}, ...
    'model StopTime back to T_scaled (the 18 Nov bug: run stops before the end of the path)'};
B(end+1,:) = {'c9_resid', 'check9dof', 'check_9dof()', 'check_9dof:failed', '\[--\] constraint satisfied', ...
    {{'chart', 'UKDynamics', "qdd = a + Mih*(pinv(B)*(b - Jc*a));", "qdd = a + Mih*(B.'*((B*B.' + 1e-6*eye(6))\(b - Jc*a)));"}}, ...
    'the model''s solve is damped (lambda^2 = 1e-6) so qdd no longer satisfies Jc qdd = b exactly; chart only, the .m files are unchanged'};
B(end+1,:) = {'c9_mins', 'check9dof', 'check_9dof()', 'check_9dof:failed', '\[--\] away from singularity', ...
    {{'edit', 'define_constants.m', "arm_home  = deg2rad([-3.0; 127.0; -10.0; 0.0; -27.0; 0.0]);", "arm_home  = deg2rad([-3.0; 127.0; -10.0; 0.0; -8.0; 0.0]);"}, ...
     {'edit', 'scenario_seeds.m', "s_min = 0.30;", "s_min = 0.0;"}}, ...
    'arm seed near the wrist singularity (J5 -27 -> -8 deg, |sin q5| 0.45 -> 0.14); the seed screening (s >= 0.30) is switched off in the copy so the run reaches the gate'};
% c9_mins (run 19 Nov 13:35) did not reach the singularity: the warm start moved the seed's J5 to -17.4 deg and
% the run's min s was 0.6624. An exploration of the seed (not a test, 13:37) gave s = 0.6771 at J5 -23.8 deg,
% 0.6082 at -3.5 deg, 0.3712 at -1.4 deg and 0.0102 at -0.01 deg (tool down: J2 + J3 + J5 = 90 deg).
% c9_mins_b therefore puts the seed within about 1 deg of the wrist singularity.
B(end+1,:) = {'c9_mins_b', 'check9dof_b', 'check_9dof()', 'check_9dof:failed', '\[--\] away from singularity', ...
    {{'edit', 'define_constants.m', "arm_home  = deg2rad([-3.0; 127.0; -10.0; 0.0; -27.0; 0.0]);", "arm_home  = deg2rad([-3.0; 127.0; -35.5; 0.0; -1.5; 0.0]);"}, ...
     {'edit', 'scenario_seeds.m', "s_min = 0.30;", "s_min = 0.0;"}}, ...
    'arm seed within about 1 deg of the wrist singularity (arm_home J3 -35.5, J5 -1.5 deg, tool down); the seed screening (s >= 0.30) is switched off in the copy so the run reaches the gate'};
B(end+1,:) = {'c9_rmse', 'check9dof', 'check_9dof()', 'check_9dof:failed', '\[--\] end effector bounded', ...
    {{'edit', 'EndEffectorTrajectory.m', "p_des = [pos(1); pos(2); z_work];", "p_des = [pos(1) + 0.03; pos(2); z_work];"}}, ...
    'the gate''s reference (EndEffectorTrajectory.m) is 3 cm from the path the model''s chart flies: the commanded end effector is 3 cm off the reference (stale-chart failure: the chart script must equal its .m file)'};
B(end+1,:) = {'c9_fwroom', 'check9dof', 'check_9dof()', 'check_9dof:failed', '\[--\] arm inside firmware range', ...
    {{'edit', 'lab_settings.m', "arm_offset_deg = [ 0 180  90   0   0   0];", "arm_offset_deg = [ 40 180  90   0   0   0];"}}, ...
    'model -> firmware map of J1 shifted by 40 deg: the ellipse''s J1 command (fw max 122 deg) maps to 162 deg, past the 160 deg end'};

% Self-test of the verdict (Task 7c, after the Task 7b review). The target raises an unrelated error, the
% identifier is not checked, and the expected text 'different motors' appears in this row's own printed header
% lines only. The verdict must be WRONG REASON (or SETUP ERROR), never FAILS. It runs in a dry run too.
B(end+1,:) = {'selftest_unrelated', 'selftest', 'verify_wheelmap()', '', 'different motors', ...
    {{'edit', 'verify_wheelmap.m', "d = fileparts(mfilename('fullpath')); cd(d); addpath(d);", "d = fileparts(mfilename('fullpath')); cd(d); addpath(d);  error('makefail:selftest', 'MAKEFAIL self-test: an unrelated error');"}}, ...
    'self-test: an unrelated error must not score FAILS, although "different motors" is printed in this header'};

% check_guard (CommandGuard.m unit sections, arm_clamp, and the model's CommandGuard chart), gate_all, run_9dof_cases
B(end+1,:) = {'cg0', 'guard', 'check_guard()', '', 'step 1 should be 1/40', ...
    {{'edit', 'CommandGuard.m', "RAMP_STEPS = 40;", "RAMP_STEPS = 20;"}}, 'soft start over 1 s instead of 2 s'};
B(end+1,:) = {'cg1', 'guard', 'check_guard()', '', 'clean command raised flag', ...
    {{'edit', 'CommandGuard.m', "    guard_flag = 0;", "    guard_flag = 1;"}}, 'a clean command is reported as scaled (flag 1)'};
B(end+1,:) = {'cg2', 'guard', 'check_guard()', '', 'direction changed', ...
    {{'edit', 'CommandGuard.m', "    wheel_out  = w*(lim/peak);", "    wheel_out  = min(max(w, -lim), lim);"}}, 'over-limit wheels clipped per wheel instead of scaled as a vector'};
B(end+1,:) = {'cg2c', 'guard', 'check_guard()', '', 'not limited at max_rpm', ...
    {{'edit', 'CommandGuard.m', "lim  = abs(max_rpm);", "lim  = 120;"}}, 'the guard ignores its max_rpm input and limits at the firmware 120 rpm'};
B(end+1,:) = {'cg3', 'guard', 'check_guard()', '', 'wheels not stopped on NaN', ...
    {{'edit', 'CommandGuard.m', "if ~all(isfinite(w)) || ~all(isfinite(j))", "if ~all(isfinite(j))"}}, 'a non-finite WHEEL command is transmitted'};
B(end+1,:) = {'cg4', 'guard', 'check_guard()', '', 'Inf joint not blocked', ...
    {{'edit', 'CommandGuard.m', "if ~all(isfinite(w)) || ~all(isfinite(j))", "if ~all(isfinite(w))"}}, 'a non-finite JOINT command is transmitted'};
B(end+1,:) = {'cg4b', 'guard', 'check_guard()', '', 'first-call hold was', ...
    {{'edit', 'CommandGuard.m', "    joint_hold = arm_home(:);", "    joint_hold = zeros(6,1);"}}, 'hold before the first finite command is 0 rad (the stretched singularity), not arm_home'};
B(end+1,:) = {'cg5_nanjoint', 'guard', 'check_guard()', '', 'NaN joint passed through|NaN joint not blocked', ...
    {{'edit', 'CommandGuard.m', "if ~all(isfinite(w)) || ~all(isfinite(j))", "if ~all(isfinite(w)) || any(isinf(j))"}}, 'a NaN JOINT command is transmitted (Inf is still blocked)'};
B(end+1,:) = {'cg5_overshoot', 'guard', 'check_guard()', '', 'not scaled to the ceiling', ...
    {{'edit', 'CommandGuard.m', "if peak > lim", "if peak > 1.01*lim"}}, 'wheel commands up to 1 % over the ceiling pass unscaled'};
B(end+1,:) = {'cg5b_clamp', 'guard', 'check_guard()', '', 'cap gave', ...
    {{'edit', 'arm_clamp.m', "deg_out = min(max(deg_in, fw_lo + margin), fw_hi - margin);", "deg_out = min(max(deg_in, fw_lo + margin), fw_hi);"}}, 'joint cap without the safety margin at the upper end'};
B(end+1,:) = {'cg5b_lim', 'guard', 'check_guard()', '', 'no longer match limits', ...
    {{'edit', 'lab_settings.m', "arm_fw_switch = [-180  132  141 -165   90  180];", "arm_fw_switch = [-180  130  141 -165   90  180];"}}, 'lab_settings firmware range of J2 no longer the Teensy limits[] (F14a: moved from define_constants)'};
B(end+1,:) = {'cg6_flag', 'guard', 'check_guard()', '', 'guard never reported saturation', ...
    {{'chart', 'CommandGuard', "    guard_flag = 1;", "    guard_flag = 0;   % MAKEFAIL"}}, 'the model''s guard chart scales but never reports saturation'};
B(end+1,:) = {'cg6_nolimit', 'guard', 'check_guard()', '', 'the guard chart sent', ...
    {{'chart', 'CommandGuard', "    wheel_out  = w*(lim/peak);", "    wheel_out  = w;   % MAKEFAIL unscaled"}}, 'the model''s guard chart reports saturation (flag 1) but transmits the over-limit wheel command unscaled'};
B(end+1,:) = {'cg_lock', 'guard', 'check_guard()', 'constants:interrupted', 'previous run was interrupted', ...
    {{'copy', 'define_constants.m', 'define_constants.PRISTINE'}}, 'a PRISTINE lock left by an interrupted run'};
B(end+1,:) = {'ga_lock', 'guard', 'gate_all()', 'constants:interrupted', 'previous run was interrupted', ...
    {{'copy', 'define_constants.m', 'define_constants.PRISTINE'}}, 'a PRISTINE lock left by an interrupted run'};
B(end+1,:) = {'rc_lock', 'guard', 'run_9dof_cases(''sim'')', 'constants:interrupted', 'previous run was interrupted', ...
    {{'copy', 'define_constants.m', 'define_constants.PRISTINE'}}, 'a PRISTINE lock left by an interrupted run'};

% derivation_checks: one break per checked section. A-D and G check algebraic identities between formulas
% typed in the test, on mass_at's M and J, so their break is a mis-written equation; F, H-K and M break the
% code under test.
B(end+1,:) = {'dc_A', 'derivation', 'derivation_checks()', 'derivation_checks:fail', 'A: constraint not met|A: the three forms', ...
    {{'edit', 'derivation_checks.m', "q9 = a + (M\J.')*(Sm\c);", "q9 = a + (J.')*(Sm\c);"}}, 'Eq. (9) without M^-1'};
B(end+1,:) = {'dc_B', 'derivation', 'derivation_checks()', 'derivation_checks:fail', 'B: projector', ...
    {{'edit', 'derivation_checks.m', "P = eye(9) - (M\J.')*(Sm\J);", "P = eye(9) - (J.')*(Sm\J);"}}, 'projector built without the M^-1 metric'};
B(end+1,:) = {'dc_C', 'derivation', 'derivation_checks()', 'derivation_checks:fail', 'C: square A', ...
    {{'edit', 'derivation_checks.m', "qs = as + (Ms\As.')*((As*(Ms\As.'))\(bs - As*as));", "qs = as + (Ms\As.')*((As*(Ms\As.'))\(bs));"}}, 'UK solve without the -A a term'};
B(end+1,:) = {'dc_D', 'derivation', 'derivation_checks()', 'derivation_checks:fail', 'D: gain bound', ...
    {{'edit', 'derivation_checks.m', "bound = norm(inv(Mh))*norm(c)/sv(end);", "bound = norm(inv(Mh))*norm(c)/sv(1);"}}, 'gain bound written with the LARGEST singular value of B'};
B(end+1,:) = {'dc_F', 'derivation', 'derivation_checks()', 'derivation_checks:fail', 'F: wrist determinant', ...
    {{'edit', 'ik9_fk.m', "0,-pi/2,0.222,0; 0,pi/2,0,0; 0,0,0.036,0]", "0,-pi/2,0.222,0; 0,pi/2+0.1,0,0; 0,0,0.036,0]"}}, 'model wrist twist alpha5 off by 0.1 rad: the wrist is no longer spherical'};
% dc_F (run 19 Nov 13:40) failed for the wrong reason: ik9_warm_start.m carries its own copy of the DH table
% (:48), so with ik9_fk.m alone changed no start reached the reference and define_constants stopped at
% scenario_seeds:branch. dc_F_b changes the twist in both copies, so the seed is consistent and F is reached.
B(end+1,:) = {'dc_F_b', 'redesign', 'derivation_checks()', 'derivation_checks:fail', 'F: wrist determinant', ...
    {{'edit', 'ik9_fk.m', "0,-pi/2,0.222,0; 0,pi/2,0,0; 0,0,0.036,0]", "0,-pi/2,0.222,0; 0,pi/2+0.1,0,0; 0,0,0.036,0]"}, ...
     {'edit', 'ik9_warm_start.m', "0,-pi/2,0.222,0; 0,pi/2,0,0; 0,0,0.036,0]", "0,-pi/2,0.222,0; 0,pi/2+0.1,0,0; 0,0,0.036,0]"}}, ...
    'model wrist twist alpha5 off by 0.1 rad in both kinematics copies (ik9_fk, ik9_warm_start): the wrist is no longer spherical'};
B(end+1,:) = {'dc_G', 'derivation', 'derivation_checks()', 'derivation_checks:fail', 'G: DLS forms', ...
    {{'edit', 'derivation_checks.m', "r2 = B.' * ((Sm + lam^2*eye(6)) \ c);", "r2 = B.' * ((Sm + lam*eye(6)) \ c);"}}, 'second DLS form damped with lambda instead of lambda^2'};
B(end+1,:) = {'dc_H', 'derivation', 'derivation_checks()', 'derivation_checks:fail', 'H: consistency vector', ...
    {{'edit', 'define_constants.m', "alphas = [deg2rad(315), deg2rad(225), deg2rad(135), deg2rad(45)];", "alphas = [deg2rad(315), deg2rad(225), deg2rad(135), deg2rad(40)];"}}, 'wheel 4 at 40 deg: the wheel map no longer has the null vector [-1 1 -1 1]'};
B(end+1,:) = {'dc_I', 'derivation', 'derivation_checks()', 'derivation_checks:fail', 'I: CW control', ...
    {{'edit', 'EndEffectorTrajectory.m', "acc = [0;        0    ];", "acc = [1e-6;        0    ];"}}, 'V-bar reference carries a radial acceleration: the control it implies is not A19''s'};
B(end+1,:) = {'dc_J', 'derivation', 'derivation_checks()', 'derivation_checks:fail', 'J: yaw acceleration', ...
    {{'edit', 'EndEffectorTrajectory.m', "psi_dot = (p_des(1)*vel(2) - p_des(2)*vel(1)) / den;", "psi_dot = 1.01 * (p_des(1)*vel(2) - p_des(2)*vel(1)) / den;"}}, 'yaw rate of the pointing law 1 % off'};
B(end+1,:) = {'dc_J_ades6', 'derivation', 'derivation_checks()', 'derivation_checks:fail', 'J: yaw acceleration', ...
    {{'edit', 'EndEffectorTrajectory.m', "A_des = [acc(1); acc(2); 0; 0; 0; psi_ddot];", "A_des = [acc(1); acc(2); 0; 0; 0; 1.01*psi_ddot];"}}, 'the yaw acceleration the model feeds forward (A_des(6), Eq. 32) 1 % off'};
B(end+1,:) = {'dc_K', 'derivation', 'derivation_checks()', 'derivation_checks:fail', 'K: scaling', ...
    {{'edit', 'EndEffectorTrajectory.m', "mu_s = mu * dist_scale^3;", "mu_s = mu;"}}, 'mu not scaled by D^3 (the 16 Sep bug): the D-scaled orbit is no longer similar to the D = 1 orbit'};
B(end+1,:) = {'dc_M', 'derivation', 'derivation_checks()', 'derivation_checks:fail', 'M: a corrected case', ...
    {{'edit', 'ukd_current.m', "KD = [2*wp;2*wp;2*wp; 2*wr;2*wr;2*wr];", "KD = -[2*wp;2*wp;2*wp; 2*wr;2*wr;2*wr];"}}, 'the solver''s derivative gain has the wrong sign: the corrected cases cannot complete'};

% verify_scenarios, verify_eetraj, verify_wheelmap
B(end+1,:) = {'vs4_ratio', 'references', 'verify_scenarios()', 'verify_scenarios:failed', '\[--\] NMC 2:1 ellipse', ...
    {{'edit', 'EndEffectorTrajectory.m', "pos = [ rho*c;        -2*rho*s    ];", "pos = [ rho*c;        -2.02*rho*s    ];"}, ...
     {'edit', 'EndEffectorTrajectory.m', "vel = [-rho*n*s;      -2*rho*n*c  ];", "vel = [-rho*n*s;      -2.02*rho*n*c  ];"}, ...
     {'edit', 'EndEffectorTrajectory.m', "acc = [-rho*n^2*c;     2*rho*n^2*s];", "acc = [-rho*n^2*c;     2.02*rho*n^2*s];"}}, ...
    'NMC in-track semi-axis 2.02 rho (pos, vel, acc consistent): not the 2:1 ellipse (and therefore not unforced CW either)'};
B(end+1,:) = {'vs4_centre', 'references', 'verify_scenarios()', 'verify_scenarios:failed', '\[--\] NMC 2:1 ellipse', ...
    {{'edit', 'EndEffectorTrajectory.m', "pos = [ rho*c;        -2*rho*s    ];", "pos = [ rho*c;        -2*rho*s + 0.1*rho];"}}, ...
    'NMC ellipse offset 0.1 rho along track: not Eq. (25) about the chief; still an unforced CW solution and still consistent in V and A'};
B(end+1,:) = {'vs6_circle_acc', 'references', 'verify_scenarios()', 'verify_scenarios:failed', '\[--\] A_des is d/dt of V_des', ...
    {{'edit', 'EndEffectorTrajectory.m', "acc = [-R*w^2*c;  -R*w^2*s_];", "acc = 1.01*[-R*w^2*c;  -R*w^2*s_];"}}, 'circle acceleration 1 % off its velocity''s derivative'};
B(end+1,:) = {'vs7_quatnorm', 'references', 'verify_scenarios()', 'verify_scenarios:failed', '\[--\] u_des is a unit quaternion', ...
    {{'edit', 'EndEffectorTrajectory.m', "u_dn = [0; cos(psi/2); sin(psi/2); 0];", "u_dn = 1.001*[0; cos(psi/2); sin(psi/2); 0];"}}, 'attitude quaternion of norm 1.001'};
B(end+1,:) = {'vs7_rate', 'references', 'verify_scenarios()', 'verify_scenarios:failed', '\[--\] omega_z matches the yaw', ...
    {{'edit', 'EndEffectorTrajectory.m', "psi_dot = (p_des(1)*vel(2) - p_des(2)*vel(1)) / den;", "psi_dot = (p_des(1)*vel(2) - p_des(2)*vel(1)) / den + 0.01;"}}, 'commanded yaw rate 0.01 rad/s while the attitude does not turn'};
B(end+1,:) = {'vs7_quat_yaw', 'references', 'verify_scenarios()', 'verify_scenarios:failed', '\[--\] omega_z matches the yaw', ...
    {{'edit', 'EndEffectorTrajectory.m', "u_dn = [0; cos(psi/2); sin(psi/2); 0];", "u_dn = [0; cos((psi + 0.01*t)/2); sin((psi + 0.01*t)/2); 0];"}}, 'attitude quaternion turns at 0.01 rad/s while the commanded yaw rate says it does not'};
B(end+1,:) = {'vs_vbar_radial', 'references', 'verify_scenarios()', 'verify_scenarios:failed', '\[--\] V-bar radial identically zero', ...
    {{'edit', 'EndEffectorTrajectory.m', "pos = [0;  y0 - v0*ts];", "pos = [1e-6;  y0 - v0*ts];"}}, 'V-bar radial coordinate 1 um instead of 0'};
B(end+1,:) = {'vs_rbar_intrack', 'references', 'verify_scenarios()', 'verify_scenarios:failed', '\[--\] R-bar in-track identically zero', ...
    {{'edit', 'EndEffectorTrajectory.m', "pos = [x0 - v0*ts;  0];", "pos = [x0 - v0*ts;  1e-6];"}}, 'R-bar in-track coordinate 1 um instead of 0'};
B(end+1,:) = {'ve_orbit', 'references', 'verify_eetraj()', 'verify:orbit', 'does not reproduce OrbitTrajectory', ...
    {{'edit', 'EndEffectorTrajectory.m', "a = a_raw * dist_scale;", "a = a_raw * dist_scale * 1.001;"}}, 'orbit semi-major axis 0.1 % off the 3-DOF model''s'};
B(end+1,:) = {'ve_quat', 'references', 'verify_eetraj()', 'verify:quat', 'not a unit quaternion', ...
    {{'edit', 'EndEffectorTrajectory.m', "u_dn = [0; cos(psi/2); sin(psi/2); 0];", "u_dn = 1.001*[0; cos(psi/2); sin(psi/2); 0];"}}, 'attitude quaternion of norm 1.001'};
B(end+1,:) = {'ve_ades', 'references', 'verify_eetraj()', 'verify:Ades', 'A_des is not the derivative', ...
    {{'edit', 'EndEffectorTrajectory.m', "A_des = [acc(1); acc(2); 0; 0; 0; psi_ddot];", "A_des = 1.01*[acc(1); acc(2); 0; 0; 0; psi_ddot];"}}, 'A_des scaled x1.01'};
B(end+1,:) = {'wm_alphas_reversed', 'references', 'verify_wheelmap()', 'verify_wheelmap:mismatch', 'workspace alphas \(wheels[\s\S]*differs from thesis Table 1', ...
    {{'edit', 'define_constants.m', "alphas = [deg2rad(315), deg2rad(225), deg2rad(135), deg2rad(45)];", "alphas = [deg2rad(45), deg2rad(135), deg2rad(225), deg2rad(315)];"}}, ...
    'wheel angles in reverse order in define_constants: every speed goes to the diagonally opposite motor'};
B(end+1,:) = {'wm_block_order', 'references', 'verify_wheelmap()', 'verify_wheelmap:mismatch', 'ukd_current.m \(wheels', ...
    {{'edit', 'ukd_current.m', "w(i) = (sin(alpha(i))*v_b(1) + cos(alpha(i))*v_b(2) + l*v_b(3))/r;", "w(i) = (sin(alpha(5-i))*v_b(1) + cos(alpha(5-i))*v_b(2) + l*v_b(3))/r;"}}, ...
    'the block''s own wheel map (ukd_current.m rome_wheels) reads the alphas in reverse order'};
B(end+1,:) = {'wm_chart_order', 'references', 'verify_wheelmap()', 'verify_wheelmap:mismatch', 'UKDynamics chart \(wheels', ...
    {{'chart', 'UKDynamics', "w(i) = (sin(alpha(i))*v_b(1) + cos(alpha(i))*v_b(2) + l*v_b(3))/r;", "w(i) = (sin(alpha(5-i))*v_b(1) + cos(alpha(5-i))*v_b(2) + l*v_b(3))/r;"}}, ...
    'the same reversal in the UKDynamics CHART the model runs (ukd_current.m unchanged)'};

% run_9dof_cases post mode, test_v3_table3 columns (case 1 of the copy's runs/), test_run_length
B(end+1,:) = {'rc_post', 'runs', 'run_9dof_cases(''post'')', 'run_9dof_cases:post', 'saved run length does not match', ...
    {{'edit', 'define_constants.m', "speed_factor = 1.0;", "speed_factor = 0.8;"}}, 'the saved runs/ are stale: define_constants now asks for a different run length'};
B(end+1,:) = {'t3_T', 'runs', 'test_v3_table3(fullfile(TMP, ''runs''), TMP)', 'test_v3_table3:mismatch', 'mismatch \(1 cells\):\s+case 1 T \[s\]', ...
    {{'mat', 'runs/case_1.mat', "mf_n = numel(D.t); D.t = D.t(1:end-20); assert(numel(D.t) == mf_n - 20);"}}, 'case 1 run 1 s short (31 -> 30 s)'};
B(end+1,:) = {'t3_posal', 'runs', 'test_v3_table3(fullfile(TMP, ''runs''), TMP)', 'test_v3_table3:mismatch', 'mismatch \(1 cells\):\s+case 1 pos aligned', ...
    {{'mat', 'runs/case_1.mat', "mf_v = D.S.rmse_ee_lead; D.S.rmse_ee_lead = 1.01*mf_v; assert(D.S.rmse_ee_lead ~= mf_v);"}}, 'case 1 lead-corrected position RMSE 1 % higher (604 -> 610 um)'};
B(end+1,:) = {'t3_poslat', 'runs', 'test_v3_table3(fullfile(TMP, ''runs''), TMP)', 'test_v3_table3:mismatch', 'mismatch \(1 cells\):\s+case 1 pos latency', ...
    {{'mat', 'runs/case_1.mat', "mf_v = D.S.rmse_ee; D.S.rmse_ee = 1.03*mf_v; assert(D.S.rmse_ee ~= mf_v);"}}, 'case 1 same-sample position RMSE 3 % higher (8.0 -> 8.3 mm)'};
B(end+1,:) = {'t3_attal', 'runs', 'test_v3_table3(fullfile(TMP, ''runs''), TMP)', 'test_v3_table3:mismatch', 'mismatch \(1 cells\):\s+case 1 att aligned', ...
    {{'mat', 'runs/case_1.mat', "mf_v = D.S.rmse_att_lead; D.S.rmse_att_lead = mf_v + 0.003; assert(D.S.rmse_att_lead ~= mf_v);"}}, 'case 1 lead-corrected attitude RMSE +0.003 deg'};
B(end+1,:) = {'t3_attlat', 'runs', 'test_v3_table3(fullfile(TMP, ''runs''), TMP)', 'test_v3_table3:mismatch', 'mismatch \(1 cells\):\s+case 1 att latency', ...
    {{'mat', 'runs/case_1.mat', "mf_v = D.S.rmse_att; D.S.rmse_att = mf_v + 0.03; assert(D.S.rmse_att ~= mf_v);"}}, 'case 1 same-sample attitude RMSE +0.03 deg'};
B(end+1,:) = {'t3_mins', 'runs', 'test_v3_table3(fullfile(TMP, ''runs''), TMP)', 'test_v3_table3:mismatch', 'mismatch \(1 cells\):\s+case 1 min s', ...
    {{'mat', 'runs/case_1.mat', "mf_v = D.S.min_s; D.S.min_s = mf_v - 0.003; assert(D.S.min_s ~= mf_v);"}}, 'case 1 min s lower by 0.003 (0.677 -> 0.674)'};
B(end+1,:) = {'t3_margin', 'runs', 'test_v3_table3(fullfile(TMP, ''runs''), TMP)', 'test_v3_table3:mismatch', 'mismatch \(1 cells\):\s+case 1 margin', ...
    {{'mat', 'runs/case_1.mat', "mf_v = D.q(:,4); D.q(:,4) = mf_v + deg2rad(3); assert(max(abs(D.q(:,4) - mf_v)) > 0);"}}, 'case 1 J1 command 3 deg closer to its firmware end'};
B(end+1,:) = {'trl_95_stoptime', 'runs', 'test_run_length()', 'test_run_length:fail', 'C1: sim ended at', ...
    {{'slx', "set_param(mdl, 'StopTime', 'T_stop - 1');", "strcmp(get_param(mdl, 'StopTime'), 'T_stop - 1')"}}, ...
    'model StopTime = T_stop - 1 s; C''s StopTime string check (:82) fires with it, necessarily'};
B(end+1,:) = {'trl_95_stopblock', 'runs', 'test_run_length()', 'test_run_length:fail', 'C1: sim ended at', ...
    {{'slx', "add_block('simulink/Sources/Clock', [mdl '/mf_clock']); add_block('simulink/Logic and Bit Operations/Compare To Constant', [mdl '/mf_cmp'], 'relop', '>=', 'const', 'T_stop - 1'); add_block('simulink/Sinks/Stop Simulation', [mdl '/mf_stop']); add_line(mdl, 'mf_clock/1', 'mf_cmp/1'); add_line(mdl, 'mf_cmp/1', 'mf_stop/1');", ...
      "~isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'mf_stop')) && strcmp(get_param(mdl, 'StopTime'), 'T_stop')"}}, ...
    'a Stop Simulation block ends every run 1 s early while StopTime stays ''T_stop'' (isolates the run-length guard :95)'};
B(end+1,:) = {'trl_Tstop_short', 'runs', 'test_run_length()', 'test_run_length:fail', 'C1: run ended at[\s\S]*C5: run ended at', ...
    {{'edit', 'define_constants.m', "T_stop = T_scaled + T_ON/2;", "T_stop = T_scaled;"}}, ...
    'T_stop back to T_scaled: every run stops T_ON/2 = 5 s before the end of the path (the 18 Nov bug, now through the variable instead of the StopTime string)'};

% trl_Tstop_short (13:42) passed test_run_length. Does anything else in the suite catch T_stop = T_scaled?
% run_9dof_cases passes StopTime = T_stop (run_9dof_cases.m:68-69) and test_v3_table3 compares round(t(end))
% with the paper's Run [s] column (:51, :63): steps 7 + 8 of the suite, run here on the same break.
B(end+1,:) = {'trl_Tstop_t3', 'verify_tstop', 'run_9dof_cases(''sim''); test_v3_table3(fullfile(TMP, ''runs''), TMP)', 'test_v3_table3:mismatch', 'case 1 T \[s\]: rounded 26', ...
    {{'edit', 'define_constants.m', "T_stop = T_scaled + T_ON/2;", "T_stop = T_scaled;"}}, ...
    'T_stop back to T_scaled (as trl_Tstop_short), seen through suite steps 7 and 8 (run_9dof_cases, then test_v3_table3 on the runs it writes)'};

% test_ar3_fk (analysis/ar3_fk.m, analysis/romefw_to_arcs_deg.m)
B(end+1,:) = {'tar_ii', 'tests', 'test_ar3_fk()', 'test_ar3_fk:fail', 'FAIL  ii: base rotation', ...
    {{'edit', 'analysis/ar3_fk.m', "q_deg = q_arcs_deg(:);", "q_deg = q_arcs_deg(:) .* [1.01; 1; 1; 1; 1; 1];"}}, 'J1 turns 1.01 x its command: adding D to q1 no longer rotates the arm by D'};
B(end+1,:) = {'tar_iii', 'tests', 'test_ar3_fk()', 'test_ar3_fk:fail', 'FAIL  iii: R not orthonormal', ...
    {{'edit', 'analysis/ar3_fk.m', "    T06 = T06 * A;", "    T06 = T06 * A;  if i == 6, T06(1:3, 1:3) = 1.001 * T06(1:3, 1:3); end"}}, 'the flange rotation scaled by 1.001 (positions untouched)'};
B(end+1,:) = {'tar_far', 'tests', 'test_ar3_fk()', 'test_ar3_fk:fail', 'FAIL  iv rome_dirpin: far end on the wrong side', ...
    {{'edit', 'analysis/romefw_to_arcs_deg.m', "s = [ 1    1    1    1    1    -1  ];", "s = [ -1    1    1    1    1    -1  ];"}, ...
     {'edit', 'analysis/romefw_to_arcs_deg.m', "c = [10 -132    2.7  0.5  14.15  31.9];", "c = [-350 -132    2.7  0.5  14.15  31.9];"}, ...
     {'edit', 'analysis/romefw_to_arcs_deg.m', "s = [ 1   -1    1    1   -1     1  ];", "s = [ -1   -1    1    1   -1     1  ];"}, ...
     {'edit', 'analysis/romefw_to_arcs_deg.m', "c = [10    2.4  2.7  0.5 -14.15 -31.9];", "c = [-350    2.4  2.7  0.5 -14.15 -31.9];"}}, ...
    'J1 direction reversed in both maps with the offset kept on the anchor (-180 -> -170): the far end lands on the wrong side'};
B(end+1,:) = {'tar_default', 'tests', 'test_ar3_fk()', 'test_ar3_fk:fail', 'FAIL  iv: the default map is not rome_dirpin', ...
    {{'edit', 'analysis/romefw_to_arcs_deg.m', "    map = 'rome_dirpin';", "    map = 'arcs_caldir';"}}, 'default map changed to arcs_caldir'};
B(end+1,:) = {'tar_differ', 'tests', 'test_ar3_fk()', 'test_ar3_fk:fail', 'FAIL  iv: the maps should differ', ...
    {{'edit', 'analysis/romefw_to_arcs_deg.m', "s = [ 1   -1    1    1   -1     1  ];", "s = [ 1   -1    1    1    1     1  ];"}, ...
     {'edit', 'analysis/romefw_to_arcs_deg.m', "c = [10    2.4  2.7  0.5 -14.15 -31.9];", "c = [10    2.4  2.7  0.5 -194.15 -31.9];"}}, ...
    'arcs_caldir J5 given rome_dirpin''s sign (offset kept on the anchor): the maps agree on J5'};
B(end+1,:) = {'tar_unknown', 'tests', 'test_ar3_fk()', 'test_ar3_fk:fail', 'FAIL  iv: an unknown map name did not raise', ...
    {{'edit', 'analysis/romefw_to_arcs_deg.m', "    otherwise<NL>        error('romefw_to_arcs_deg:map',", "    otherwise<NL>        s = [1 1 1 1 1 -1];  c = [10 -132 2.7 0.5 14.15 31.9];  sprintf("}}, 'an unknown map name silently falls back to rome_dirpin'};

% test_commanding_check (analysis/commanding_check.m and its run file)
B(end+1,:) = {'tcc_sha', 'tests', 'test_commanding_check()', 'test_commanding_check:sha', 'run file SHA-256', ...
    {{'code', "mf_f = fullfile(TMP, 'data', '9-24-26-15-33_Case_1_data.mat'); mf_d = dir(mf_f); mf_n = mf_d.bytes; mf_fid = fopen(mf_f, 'a'); fwrite(mf_fid, uint8(0)); fclose(mf_fid); mf_d = dir(mf_f); assert(mf_d.bytes == mf_n + 1);"}}, ...
    'the 24 Sep run file changed by one appended byte'};
B(end+1,:) = {'tcc_c', 'tests', 'test_commanding_check()', 'test_commanding_check:fail', 'FAIL  c rome_dirpin: upper arm', ...
    {{'edit', 'analysis/romefw_to_arcs_deg.m', "c = [10 -132    2.7  0.5  14.15  31.9];", "c = [10 -122    2.7  0.5  14.15  31.9];"}}, 'rome_dirpin J2 offset 10 deg off: the physical path no longer puts the upper arm where the Task 5 photo check does'};
B(end+1,:) = {'tcc_d', 'tests', 'test_commanding_check()', 'test_commanding_check:fail', 'FAIL  d: derived map', ...
    {{'edit', 'analysis/commanding_check.m', "Cc = [0 90 90  0  0 180];", "Cc = [0 80 90  0  0 180];"}}, 'convention-consistent map J2 offset 10 deg off'};
B(end+1,:) = {'tcc_e', 'tests', 'test_commanding_check()', 'test_commanding_check:fail', 'FAIL  e: held joint', ...
    {{'edit', 'analysis/commanding_check.m', "fw_phys(:, j) = opt.hold(j);", "fw_phys(:, j) = opt.hold(j) + 1;"}}, 'a held joint is held 1 deg off the requested angle'};

% test_clearance (clearance_9dof.m, link_clearance.m)
B(end+1,:) = {'tc_flange', 'tests', 'test_clearance()', 'test_clearance:fail', '(b|c): z_min', ...
    {{'edit', 'link_clearance.m', "for i = 2:6", "for i = 2:5"}}, 'z_min and xy_min ignore the last link (wrist to flange)'};
B(end+1,:) = {'tc_xy', 'tests', 'test_clearance()', 'test_clearance:fail', 'd: xy_min', ...
    {{'edit', 'link_clearance.m', "sd = sqrt(max(dx, 0).^2 + max(dy, 0).^2) + min(max(dx, dy), 0);", "sd = sqrt(max(dx, 0).^2 + max(dy, 0).^2) - min(max(dx, dy), 0);"}}, 'signed distance positive inside the base square'};
B(end+1,:) = {'tc_ar3mount', 'tests', 'test_clearance()', 'test_clearance:fail', 'f: ar3 z_min', ...
    {{'edit', 'clearance_9dof.m', "Q = Pa + Qm(:, 1);", "Q = Pa;"}}, '''ar3'' geometry without the 0.20 m mount'};
B(end+1,:) = {'tc_yaw', 'tests', 'test_clearance()', 'test_clearance:fail', 'e: z_min or xy_min changes', ...
    {{'edit', 'clearance_9dof.m', "[~, ~, Q] = ik9_fk(q9);", "[~, ~, Q] = ik9_fk(q9);  Q(1:2, :) = Q(1:2, :) + q9(1:2);"}}, 'link points measured with the base position added (world, not body frame)'};

% test_pacing (apply_pacing.m, the stubs, the model callbacks)
B(end+1,:) = {'tp_d1', 'pacing', 'test_pacing()', 'test_pacing:fail', 'd1: no error', ...
    {{'edit', 'apply_pacing.m', "catch err<NL>    if live", "catch err<NL>    if false"}}, 'pacing that cannot be set no longer refuses a hardware run'};
B(end+1,:) = {'tp_noflag', 'pacing', 'test_pacing()', 'test_pacing:fail', 'd2: no error', ...
    {{'edit', 'apply_pacing.m', "    error('apply_pacing:noflag', 'EnableHardware is not defined in the base workspace; run define_constants first. Run refused.');", "    hw = 0;"}}, 'an undefined EnableHardware is treated as simulation instead of refused (d2 and d5)'};
B(end+1,:) = {'tp_d3', 'pacing', 'test_pacing()', 'test_pacing:fail', 'd3: ', ...
    {{'edit', 'apply_pacing.m', "if ~(isnumeric(hw) && isscalar(hw) && isfinite(hw))", "if ~isnumeric(hw)"}}, 'a non-scalar EnableHardware is not refused as badflag'};
B(end+1,:) = {'tp_d6', 'pacing', 'test_pacing()', 'test_pacing:fail', 'd6: ', ...
    {{'edit', 'apply_pacing.m', "    return                                        % simulation only, define_constants run standalone", "    error('apply_pacing:makefail', 'MAKEFAIL d6');"}}, 'define_constants run standalone (no model, EnableHardware = 0) now errors'};
B(end+1,:) = {'tp_net', 'pacing', 'test_pacing()', 'test_pacing:network', 'contains a network call', ...
    {{'edit', 'tests/stubs/ROMECommand.m', "            armReady       = false;", "            armReady       = false;  if false, dev = tcpclient('127.0.0.1', 1); end %#ok<NASGU>"}}, 'the ROMECommand stub contains a TCP client call'};
B(end+1,:) = {'tp_which', 'pacing', 'test_pacing()', 'test_pacing:stub', 'MotiveData resolves to', ...
    {{'copy', 'Optitrack/MotiveData.m', 'MotiveData.m'}}, 'a stray copy of the REAL MotiveData.m at the folder root shadows the network-free stub'};
B(end+1,:) = {'tp_callback', 'pacing', 'test_pacing()', 'test_pacing:callback', 'InitFcn/StartFcn', ...
    {{'slx', "set_param(mdl, 'StartFcn', '');", "isempty(get_param(mdl, 'StartFcn')) && contains(get_param(mdl, 'InitFcn'), 'apply_pacing')"}}, 'the model''s StartFcn no longer calls apply_pacing'};
B(end+1,:) = {'tp_fr_branch', 'pacing', 'test_pacing()', 'test_pacing:fail', 'f: Fast Restart run 2', ...
    {{'edit', 'apply_pacing.m', "if live && bdIsLoaded(mdl) && strcmp(get_param(mdl, 'FastRestart'), 'on')", "if false && live && bdIsLoaded(mdl) && strcmp(get_param(mdl, 'FastRestart'), 'on')"}}, 'apply_pacing no longer refuses a hardware run under Fast Restart'};
B(end+1,:) = {'tp_fr_text', 'pacing', 'test_pacing()', 'test_pacing:fail', 'f: Fast Restart run 2', ...
    {{'edit', 'apply_pacing.m', "'EnableHardware = 1 but Fast Restart is on for %s: pacing is not re-applied between fast restarts. Turn Fast Restart off. Run refused.', mdl);", "'EnableHardware = 1 but a quick-restart mode is on for %s. Run refused.', mdl);"}}, 'the Fast Restart refusal no longer says why (the reason text check of f)'};
B(end+1,:) = {'tp_readback', 'pacing', 'test_pacing()', 'test_pacing:fail', 'test_pacing', ...
    {{'edit', 'apply_pacing.m', "if live && ~(strcmp(got, 'on') && isequal(double(rate), 1))", "if false && live && ~(strcmp(got, 'on') && isequal(double(rate), 1))"}}, 'the read-back refusal ("cannot confirm" on a loaded model, apply_pacing.m:60-62) removed'};
% Final review (19 Nov 2026): the SensorModel-on-hardware refusal (I5(4)) and test_run_length D without file writes (I2)
B(end+1,:) = {'tp_sensormodel', 'pacing', 'test_pacing()', 'test_pacing:fail', 'd7: ', ...
    {{'edit', 'apply_pacing.m', "    if ~smok", "    if false"}}, 'apply_pacing no longer refuses a hardware run with sensor_model ~= 0 (SensorModel noise on the robot)'};
B(end+1,:) = {'tp_dc_force', 'pacing', 'test_pacing()', 'test_pacing:fail', 'd8: ', ...
    {{'edit', 'define_constants.m', "    sensor_model = 0;  % forced", "    sensor_model = 1;  % forced"}}, 'define_constants no longer forces sensor_model = 0 when EnableHardware ~= 0 (its own apply_pacing call then refuses)'};
B(end+1,:) = {'trl_D_off', 'runs', 'test_run_length()', 'test_run_length:fail', 'D: EnableHardware = 0 gave EnablePacing ''on''', ...
    {{'edit', 'apply_pacing.m', "state = {'off', 'on'};", "state = {'on', 'on'};"}}, ...
    'apply_pacing sets pacing on also with EnableHardware = 0; C co-fires (pacing on after load) and its five runs go in real time, about 4.5 min'};

% test_suite (14 steps since Plan 2 Task 1, counted from the steps cell of tests/test_suite.m on 19 Nov; test_clearance
% is step 13): lock at the start, hash and lock at the end, a failing step 13
B(end+1,:) = {'ts_lock_start', 'suite', 'test_suite()', 'test_suite:lock', 'PRISTINE exists before the suite', ...
    {{'copy', 'define_constants.m', 'define_constants.PRISTINE'}}, 'a PRISTINE lock present when the suite starts'};
B(end+1,:) = {'ts_hash', 'suite', 'test_suite()', 'test_suite:fail', '0 of 14 steps failed or skipped \(\); hash unchanged 0; lock absent 1', ...
    {{'edit', 'tests/test_clearance.m', "fprintf('test_clearance PASS\n');", "mf_fid = fopen(fullfile(root, 'define_constants.m'), 'a'); fwrite(mf_fid, [newline '% MAKEFAIL mid-suite edit' newline]); fclose(mf_fid); fprintf('test_clearance PASS\n');"}}, ...
    'define_constants.m edited during the suite (by step 13, which still passes)'};
B(end+1,:) = {'ts_lock_end', 'suite', 'test_suite()', 'test_suite:fail', '0 of 14 steps failed or skipped \(\); hash unchanged 1; lock absent 0', ...
    {{'edit', 'tests/test_clearance.m', "fprintf('test_clearance PASS\n');", "copyfile(fullfile(root, 'define_constants.m'), fullfile(root, 'define_constants.PRISTINE')); fprintf('test_clearance PASS\n');"}}, ...
    'a PRISTINE lock left behind during the suite (by step 13, which still passes)'};
B(end+1,:) = {'ts_step13', 'suite', 'test_suite()', 'test_suite:fail', '1 of 14 steps failed or skipped \(test_clearance\); hash unchanged 1; lock absent 1', ...
    {{'edit', 'tests/test_clearance.m', "fprintf('test_clearance PASS\n');", "error('makefail:step13', 'MAKEFAIL step 13'); fprintf('test_clearance PASS\n');"}}, ...
    'step 13 (test_clearance) errors: the 14-step suite must fail and name it'};

% ---- selection ---------------------------------------------------------------------------------------------
if ischar(ids) || isstring(ids)
    ids = char(ids);
    if strcmp(ids, 'list')
        for r = 1:size(B, 1)
            fprintf('%-20s %-11s %-46s %s\n', B{r,1}, B{r,2}, B{r,3}, B{r,7});
        end
        return
    elseif strcmp(ids, 'all')
        ids = B(:,1).';
    elseif startsWith(ids, 'group:')
        ids = B(strcmp(B(:,2), ids(7:end)), 1).';
    else
        ids = {ids};
    end
end
assert(~isempty(ids), 'makefail_campaign:none', 'no break selected');

% ---- run ---------------------------------------------------------------------------------------------------
startDir = pwd;                                              % caller's folder, restored after each break (char)
p0       = strsplit(path, pathsep);                          % path entries at entry (cell of char)
basePath = strjoin(p0(~startsWith(lower(p0), lower(merged))), pathsep);   % the path without any ROME_merged folder
logFile  = fullfile(logDir, sprintf('%s_task7b_%s.log', datestr(now, 'yyyy-mm-dd'), group)); %#ok<TNOW1,DATST>
diary(logFile);  diary on
fprintf('makefail_campaign  %s  MATLAB %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), version); %#ok<TNOW1,DATST>
fprintf('source (never modified): %s\ncopies under: %s\ndry run: %d\nbreaks (%d): %s\n', merged, copyRoot, dryRun, numel(ids), strjoin(ids, ', '));
res = cell(numel(ids), 4);                                   % per break: id, verdict, identifier, message (char)
nSelfBad = 0;                                                % self-tests of the verdict that scored FAILS (count)
for b = 1:numel(ids)
    r = find(strcmp(B(:,1), ids{b}));
    if numel(r) ~= 1
        res(b,:) = {ids{b}, 'UNKNOWN ID', '', ''};
        fprintf('\n#### UNKNOWN break id %s\n', ids{b});
        continue
    end
    code = B{r,3};  eid = B{r,4};  epat = B{r,5};  pre = B{r,6};   % target, expected id, expected text, edits
    mark = sprintf('#### START %s  %s', ids{b}, datestr(now, 'yyyy-mm-dd HH:MM:SS')); %#ok<TNOW1,DATST>
    fprintf('\n%s\n', mark);
    fprintf('target: %s\nexpected: FAIL, identifier ''%s'', text /%s/\nbreak: %s\n', code, eid, epat, B{r,7});
    TMP = fullfile(copyRoot, sprintf('mf_%s_%s', ids{b}, datestr(now, 'HHMMSSFFF'))); %#ok<TNOW1,DATST>  % the throwaway copy (char)
    verdict = '';  err = [];  emsg = '';  eidGot = '';
    try
        bdclose('all');  cd(startDir);  path(basePath);
        evalin('base', 'clear all');
        copyfile(merged, TMP);
        cd(TMP);
        addpath(TMP, fullfile(TMP, 'tests'), fullfile(TMP, 'analysis'), fullfile(TMP, 'Ground Vehicle'), fullfile(TMP, 'Optitrack'));
        rehash path
        fprintf('copy: %s\n', TMP);
        for a = 1:numel(pre)
            pa = pre{a};
            switch pa{1}
                case 'edit'
                    f = fullfile(TMP, pa{2});
                    fid = fopen(f, 'r');  by = fread(fid, Inf, '*uint8').';  fclose(fid);
                    txt = native2unicode(by, 'UTF-8');               % file text (char)
                    if contains(txt, char([13 10])), eol = char([13 10]); else, eol = newline; end
                    old = strrep(char(pa{3}), '<NL>', eol);  new = strrep(char(pa{4}), '<NL>', eol);
                    k = strfind(txt, old);
                    if numel(k) ~= 1
                        error('makefail_campaign:edit', '%s: expected exactly 1 match, found %d, of: %s', pa{2}, numel(k), old);
                    end
                    txt = [txt(1:k-1) new txt(k+numel(old):end)];
                    fid = fopen(f, 'w');  fwrite(fid, unicode2native(txt, 'UTF-8'));  fclose(fid);
                    fid = fopen(f, 'r');  by = fread(fid, Inf, '*uint8').';  fclose(fid);
                    assert(isequal(native2unicode(by, 'UTF-8'), txt), 'makefail_campaign:edit', 'read-back of %s differs', pa{2});
                    fprintf('edit %s line %d (1 match):\n  - %s\n  + %s\n', pa{2}, 1 + sum(txt(1:k-1) == newline), char(pa{3}), char(pa{4}));
                case 'copy'
                    copyfile(fullfile(TMP, pa{2}), fullfile(TMP, pa{3}));
                    assert(isfile(fullfile(TMP, pa{3})), 'makefail_campaign:copy', 'copy to %s failed', pa{3});
                    fprintf('copy %s -> %s\n', pa{2}, pa{3});
                case 'mat'
                    f = fullfile(TMP, pa{2});
                    D = load(f);
                    eval(char(pa{3}));
                    save(f, '-struct', 'D');
                    fprintf('mat %s: %s\n', pa{2}, char(pa{3}));
                case 'code'
                    eval(char(pa{2}));
                    fprintf('code: %s\n', char(pa{2}));
                case 'slx'
                    bdclose('all');  load_system(fullfile(TMP, [mdl '.slx']));
                    eval(char(pa{2}));
                    save_system(mdl);  close_system(mdl, 0);
                    load_system(fullfile(TMP, [mdl '.slx']));
                    ok = eval(char(pa{3}));  close_system(mdl, 0);
                    assert(ok, 'makefail_campaign:slx', 'model check failed after: %s', char(pa{2}));
                    fprintf('model %s: %s\n  read back: %s -> true\n', mdl, char(pa{2}), char(pa{3}));
                case 'chart'
                    bdclose('all');  load_system(fullfile(TMP, [mdl '.slx']));
                    rt = sfroot;  ch = rt.find('-isa', 'Stateflow.EMChart', 'Name', pa{2});
                    keep = false(size(ch));  for c = 1:numel(ch), keep(c) = startsWith(ch(c).Path, [mdl '/']); end
                    ch = ch(keep);
                    assert(numel(ch) == 1, 'makefail_campaign:chart', 'found %d charts named %s in %s', numel(ch), pa{2}, mdl);
                    src = ch.Script;  old = char(pa{3});  new = char(pa{4});
                    k = strfind(src, old);
                    if numel(k) ~= 1
                        error('makefail_campaign:chart', 'chart %s: expected exactly 1 match, found %d, of: %s', pa{2}, numel(k), old);
                    end
                    ch.Script = [src(1:k-1) new src(k+numel(old):end)];
                    save_system(mdl);  close_system(mdl, 0);
                    load_system(fullfile(TMP, [mdl '.slx']));
                    rt = sfroot;  ch = rt.find('-isa', 'Stateflow.EMChart', 'Name', pa{2});
                    keep = false(size(ch));  for c = 1:numel(ch), keep(c) = startsWith(ch(c).Path, [mdl '/']); end
                    ch = ch(keep);
                    nNew = numel(strfind(ch.Script, new));  nOld = numel(strfind(ch.Script, old));
                    close_system(mdl, 0);
                    assert(nNew == 1 && (nOld == 0 || contains(new, old)), 'makefail_campaign:chart', 'read-back of chart %s: %d new, %d old', pa{2}, nNew, nOld);
                    fprintf('chart %s/%s (1 match, read back after save):\n  - %s\n  + %s\n', mdl, pa{2}, old, new);
                otherwise
                    error('makefail_campaign:pre', 'unknown pre-action %s', pa{1});
            end
        end
        bdclose('all');  cd(TMP);  evalin('base', 'clear all');
        fn = regexp(code, '^\w+', 'match', 'once');           % target function name (char)
        wfn = which(fn);                                       % where it resolves (char)
        fprintf('which %s -> %s\n', fn, wfn);
        assert(startsWith(lower(wfn), lower(TMP)), 'makefail_campaign:which', '%s resolves outside the copy: %s', fn, wfn);
    catch se
        verdict = 'SETUP ERROR';  eidGot = se.identifier;  emsg = se.message;
        fprintf('SETUP ERROR: %s | %s\n', se.identifier, se.message);
    end
    if isempty(verdict) && dryRun && ~strcmp(B{r,2}, 'selftest')   % self-tests of the verdict run in a dry run too
        verdict = 'DRY RUN';
    elseif isempty(verdict)
        runMark = sprintf('---- running %s  [%s]', code, ids{b});   % start of the target's own output (char)
        fprintf('%s\n', runMark);
        try
            eval(code);
        catch err
        end
        cd(startDir);
        if ~isempty(err)
            eidGot = err.identifier;  emsg = err.message;
            fprintf('\n---- %s errored\n  identifier: %s\n  message: %s\n', code, eidGot, emsg);
        else
            fprintf('\n---- %s returned without error\n', code);
        end
        diary off
        L = fileread(logFile);  kk = strfind(L, runMark);  seg = L(kk(end)+numel(runMark):end);
        % seg = the target's output only, from after the '---- running' line: the header lines printed
        % above it (target, expected text, break) must not be able to satisfy the expected text (Task 7b
        % review; before Task 7c seg began at '#### START', which made a plain-text pattern match its own
        % 'expected:' line, so any error could score FAILS).
        diary on
        if isempty(err)
            verdict = 'DID NOT FAIL';
        elseif (isempty(eid) || strcmp(eidGot, eid)) && ~isempty(regexp([seg newline emsg], epat, 'once'))
            verdict = 'FAILS';
        else
            verdict = 'WRONG REASON';
        end
        cof = regexp(seg, '[^\n]*(\[--\]|FAIL  |STEP FAILED)[^\n]*', 'match');   % failing lines printed by the target
        fprintf('---- failing lines printed by the target (%d):\n', numel(cof));
        for c = 1:min(numel(cof), 25), fprintf('  | %s\n', strtrim(cof{c})); end
    end
    if strcmp(B{r,2}, 'selftest')
        okSelf = ~strcmp(verdict, 'FAILS');        % an unrelated error must never score FAILS (logical)
        nSelfBad = nSelfBad + ~okSelf;
        sfx = 'PASS';  if ~okSelf, sfx = 'FAILED'; end   % self-test outcome (char)
        fprintf('#### SELFTEST %s : verdict %s -> %s (must be WRONG REASON or SETUP ERROR, never FAILS)\n', ids{b}, verdict, sfx);
    end
    res(b,:) = {ids{b}, verdict, eidGot, emsg};
    fprintf('#### VERDICT %s : %s | identifier %s | %s\n', ids{b}, verdict, eidGot, strtok(emsg, newline));
    try, bdclose('all'); catch, end
    cd(startDir);  path(basePath);  evalin('base', 'clear all');
    try
        rmdir(TMP, 's');  fprintf('copy deleted\n');
    catch de
        fprintf('copy NOT deleted (%s): %s\n', de.message, TMP);
    end
end
fprintf('\n#### SUMMARY %s  (%d breaks)\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), numel(ids)); %#ok<TNOW1,DATST>
for b = 1:numel(ids)
    fprintf('%-20s %-13s %-32s %s\n', res{b,1}, res{b,2}, res{b,3}, strtok(res{b,4}, newline));
end
diary off
if nSelfBad > 0
    error('makefail_campaign:selftest', '%d self-test(s) of the verdict scored FAILS on an unrelated error; the verdicts are not trustworthy', nSelfBad);
end
end
