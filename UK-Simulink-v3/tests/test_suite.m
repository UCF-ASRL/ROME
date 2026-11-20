function test_suite(only, extra)
%TEST_SUITE  Full check of ROME_merged. Errors if any step fails.
%   Runs, in this order, each in its own try/catch so every result is printed:
%     1 gate_all               acceptance gate, check_9dof on scenarios 1-5, in the model
%     2 check_guard            CommandGuard and arm_clamp
%     3 derivation_checks      numerical checks of the derivation (data/*.mat)
%     4 verify_scenarios       references against the manuscript equations (12 checks)
%     5 verify_wheelmap        wheel map (workspace alphas, ukd_current.m, UKDynamics chart) against thesis Table 1 / Fig. 6
%     6 verify_eetraj          EndEffectorTrajectory against OrbitTrajectory_ref
%     7 run_9dof_cases('sim')  the five case studies in the model -> runs/case_<n>.mat
%     8 test_v3_table3         runs/ of step 7 against Table 3 of the final paper (V3),
%                              margin from this folder's define_constants.m; SKIPPED (and the
%                              suite fails) when step 7 failed, so old runs/ are never checked
%     9 test_run_length        every run goes to T_stop, T_ON literals agree, pacing follows EnableHardware
%    10 test_ar3_fk            vendor-DH AR3 kinematics (analysis/ar3_fk) and the ROME->ARCS joint map
%    11 test_pacing            pacing set at every run start (InitFcn apply_pacing), temp copy with network stubs
%    12 test_commanding_check  analysis/commanding_check on the 24 Sep run: plumbing (model DH in the physical
%                              path = ik9_fk, 1e-9 m) with built-in make-it-fail, t = 0 anchors, derived map
%    13 test_clearance         clearance_9dof: arm_home z_min > 0.05, tool 0.02 m below the floor, forearm over
%                              the deck, base-pose invariance, 'ar3' oracle, tie to commanding_check
%    14 test_motive_reader     NatNet frame parser (msgId, rigid-body ID, finite, quaternion norm) and
%                              MotiveData hold/valid/nDropped through a no-socket test seam (Plan 2 Task 1)
%    15 test_hw_nocam         hardware mode without cameras end to end against a recording fake robot
%                              (temp copy, no network): sent lines = simulation commands step by step, pacing,
%                              limits, no sensor noise, STOP_ALL last at normal end and error stop (Plan 2 Task 1c),
%                              arm_freeze = 1 holds the sent joints at arm_freeze_deg (vi, Plan 2 Task 5 / F1)
%    16 test_arm_freeze        upstream arm freeze offline: switch output = q0_9dof(4:9) = arm_freeze_deg also on
%                              a second Run after ROMECommand's arm_freeze = 0 override, = CommandGuard output with
%                              arm_freeze = 0, banner, ROMECommand.m = lab 63a10aa blob (Plan 2 Task 5 (iv) / F1)
%    17 test_motive_model      absolute floor-frame camera path with the real-camera source ('optitrack'), injected
%                              NatNet pose, reader gate, motive_cal refusal (focus plan F5b; added by F7, ruling I3)
%    18 test_motive_source     motive_source values, the 'optitrack'-only refusal at both call sites, define_constants
%                              in a function workspace with a fresh / stale base (focus plan F7, F5b review I1)
%    19 test_motive_emulator   camera emulator: exact plumbing, paper noise (Table 3), robot mode 2 against the fake
%                              robot with fake arm telemetry, unwrap reset between runs and in Fast Restart (F7, I2)
%    20 test_run_mode          one-number mode selector (F12): rome_mode through the model's four Mode buttons gives the
%                              author's four flag sets and arm source; bad run_mode refused at load; only the digit
%                              changes; stale-SHA refusal; the gate and case runner refuse any mode but 1
%    21 test_run_mode_core     run_mode_table and set_run_mode on synthetic text (F12 core): flags, refusals, CRLF/LF
%    22 test_lab_settings      F14a: 4-line student block, lab_settings.m the one source of the lab constants (sibling,
%                              not the path), unit comments, stale check and park-record SHA over both files, snapshot
%    23 test_run_verdict_core  F14a: the PASS/FAIL rule of a robot run on synthetic facts, one fault per item
%    24 test_run_verdict       F14a: the verdict line and save_run() on the Run-button path against the fake robot,
%                              modes 2 and 3, one passing run and one injected fault per item
%    25 test_heading_plot      F14a (F14d fix): the plotted heading error of the saved case-1 run <= 2.6 deg
%    26 test_prerun_wiring     F2/F11 wiring: InitFcn prerun_check refuses a robot run with every reason; the park record
%                              calibrateROMEArm writes is accepted in modes 2 and 3; a Mode button after it is refused
%    27 test_run_record        F3 wiring: mode-1 plot_9dof + save_run() use the StopFcn stop record (no late snapshot);
%                              the controller's input StateSource out 1 is logged as q_ctrl_in
%    28 test_camera_guard_wiring  F6 wiring, fake robot mode 3: 5-sample camera dropout held, 6 samples stop the run
%                              (wheels 0, STOP_ALL last), no arm telemetry -> arm wait refuses with wheels 0
%    29 test_camera_guard_core camera_guard_core / arm_ready_core, pure (F6 core), checks (a)-(q)
%    30 test_prerun_core       prerun_check_core and lab/arm_park_record, pure (F2/F11 core), checks (i)-(ix)
%    31 test_hw_tools_core     save_run and compare_hw_sim_core, pure (F3 core), checks (i)-(v)
%    32 test_romecommand_sendlog  the send log in ROMECommand.m changes nothing the fake robot receives (P1-P5)
%    33 test_jog_send          jogROMEArm's arm commands: capped by arm_clamp, none before ARM telemetry (fixes5)
%   Every listed step must have its own case below; a listed name without one FAILS (test_suite:nostep), so a step
%   can never be counted as passed without running (wire3 review C1, 20 Nov 2026). test_suite(only) runs only the
%   named steps (cell of step names; the others are SKIPPED, so the suite then FAILS by design); test_suite(only,
%   extra) also appends the names in extra to the list (make-it-fail: an extra name without a case must FAIL).
%   and asserts that define_constants.m and lab_settings.m have the same SHA-256 before and after
%   (gate_all, check_guard and run_9dof_cases rewrite it under the PRISTINE lock,
%   CLAUDE.md section 20.4) and that no define_constants.PRISTINE is left behind.
%   A function, not a script: run_9dof_cases clears the base workspace.
%   Written flat, without local functions (CLAUDE.md section 5, crash 2).
here = fileparts(mfilename('fullpath'));    % ROME_merged/tests
root = fileparts(here);                     % ROME_merged
addpath(here, root);
dc   = fullfile(root, 'define_constants.m');            % the file the checks take custody of
lock = fullfile(root, 'define_constants.PRISTINE');      % custody lock written by gate_all / check_guard / run_9dof_cases
assert(~isfile(lock), 'test_suite:lock', 'define_constants.PRISTINE exists before the suite: an interrupted check left it; restore define_constants.m from it first (student procedure, appendix A3)');
fid = fopen(dc, 'r');  b0 = fread(fid, Inf, '*uint8');  fclose(fid);     % bytes before
md = java.security.MessageDigest.getInstance('SHA-256');
h0 = lower(reshape(dec2hex(typecast(md.digest(b0), 'uint8'), 2).', 1, []));   % SHA-256 before, hex
fprintf('test_suite: %s\n  define_constants.m SHA-256 before: %s\n', root, h0);
labf = fullfile(root, 'lab_settings.m');                % the TA file (F14a): no step may change it either
fid = fopen(labf, 'r');  bL0 = fread(fid, Inf, '*uint8');  fclose(fid);   % bytes before
md = java.security.MessageDigest.getInstance('SHA-256');
hL0 = lower(reshape(dec2hex(typecast(md.digest(bL0), 'uint8'), 2).', 1, []));   % SHA-256 before, hex
fprintf('  lab_settings.m SHA-256 before:     %s\n', hL0);
t0 = tic;

steps = {'gate_all', 'check_guard', 'derivation_checks', 'verify_scenarios', ...
         'verify_wheelmap', 'verify_eetraj', 'run_9dof_cases(''sim'')', 'test_v3_table3', 'test_run_length', 'test_ar3_fk', 'test_pacing', 'test_commanding_check', 'test_clearance', 'test_motive_reader', 'test_hw_nocam', 'test_arm_freeze', 'test_motive_model', 'test_motive_source', 'test_motive_emulator', 'test_run_mode', 'test_run_mode_core', ...
         'test_lab_settings', 'test_run_verdict_core', 'test_run_verdict', 'test_heading_plot', 'test_prerun_wiring', 'test_run_record', 'test_camera_guard_wiring', ...
         'test_camera_guard_core', 'test_prerun_core', 'test_hw_tools_core', 'test_romecommand_sendlog', 'test_jog_send'};   % step names, in run order
if nargin >= 2 && ~isempty(extra), steps = [steps, cellstr(extra)]; end   % make-it-fail only: names without a case
if nargin < 1 || isempty(only), only = steps; end
only = cellstr(only);             % steps to run (cell of char); the others are SKIPPED (not a pass)
ok   = false(1, numel(steps));    % pass flag per step (a skipped step is not a pass)
skip = false(1, numel(steps));    % true for a step not run because a step it depends on failed
msg  = repmat({''}, 1, numel(steps));   % error or skip message per step (char, '' when none)
dur  = zeros(1, numel(steps));    % wall time per step (s)
for k = 1:numel(steps)
    fprintf('\n================ %d/%d %s ================\n', k, numel(steps), steps{k});
    cd(root);
    ts = tic;                     % timer of this step
    if ~any(strcmp(steps{k}, only))
        skip(k) = true;  msg{k} = 'SKIPPED: not selected (test_suite(only))';  fprintf('  %s\n', msg{k});
        continue
    end
    if k == 8 && ~ok(7)
        % test_v3_table3 checks the runs written by step 7; without them it would check old files.
        skip(k) = true;
        msg{k} = 'SKIPPED: run_9dof_cases failed, runs/ would be stale';
        fprintf('  %s\n', msg{k});
        dur(k) = toc(ts);
        continue
    end
    try
        switch steps{k}            % by NAME: a listed name without a case reaches otherwise and FAILS (review C1)
            case 'gate_all', gate_all();
            case 'check_guard', check_guard();
            case 'derivation_checks', derivation_checks();
            case 'verify_scenarios', verify_scenarios();
            case 'verify_wheelmap', verify_wheelmap();
            case 'verify_eetraj', verify_eetraj();
            case 'run_9dof_cases(''sim'')', run_9dof_cases('sim');
            case 'test_v3_table3', test_v3_table3(fullfile(root, 'runs'), root);
            case 'test_run_length', test_run_length();
            case 'test_ar3_fk', test_ar3_fk();
            case 'test_pacing', test_pacing();
            case 'test_commanding_check', test_commanding_check();
            case 'test_clearance', test_clearance();
            case 'test_motive_reader', test_motive_reader();
            case 'test_hw_nocam', test_hw_nocam();
            case 'test_arm_freeze', test_arm_freeze();
            case 'test_motive_model', test_motive_model();
            case 'test_motive_source', test_motive_source();
            case 'test_motive_emulator', test_motive_emulator();
            case 'test_run_mode', test_run_mode();
            case 'test_run_mode_core', test_run_mode_core();
            case 'test_lab_settings', test_lab_settings();
            case 'test_run_verdict_core', test_run_verdict_core();
            case 'test_run_verdict', test_run_verdict();
            case 'test_heading_plot', test_heading_plot();
            case 'test_prerun_wiring', test_prerun_wiring();
            case 'test_run_record', test_run_record();
            case 'test_camera_guard_wiring', test_camera_guard_wiring();
            case 'test_camera_guard_core', test_camera_guard_core();
            case 'test_prerun_core', test_prerun_core();
            case 'test_hw_tools_core', test_hw_tools_core();
            case 'test_romecommand_sendlog', test_romecommand_sendlog();
            case 'test_jog_send', test_jog_send();
            otherwise, error('test_suite:nostep', 'step %d ''%s'' is listed but has no case: not run, so not passed', k, steps{k});
        end
        ok(k) = true;
    catch e
        msg{k} = sprintf('%s: %s', e.identifier, e.message);
        fprintf('\n  STEP FAILED: %s\n', msg{k});
    end
    dur(k) = toc(ts);
end
cd(root);

fid = fopen(dc, 'r');  b1 = fread(fid, Inf, '*uint8');  fclose(fid);     % bytes of define_constants.m after the steps
md = java.security.MessageDigest.getInstance('SHA-256');
h1 = lower(reshape(dec2hex(typecast(md.digest(b1), 'uint8'), 2).', 1, []));   % SHA-256 after, hex
hash_ok = strcmp(h0, h1);         % true if define_constants.m is byte-identical to before the suite
lock_ok = ~isfile(lock);          % true if no define_constants.PRISTINE custody lock was left behind
fid = fopen(labf, 'r');  bL1 = fread(fid, Inf, '*uint8');  fclose(fid);   % bytes of lab_settings.m after the steps
md = java.security.MessageDigest.getInstance('SHA-256');
hL1 = lower(reshape(dec2hex(typecast(md.digest(bL1), 'uint8'), 2).', 1, []));   % SHA-256 after, hex
lab_ok = strcmp(hL0, hL1);        % true if lab_settings.m is byte-identical to before the suite (F14a)

fprintf('\n================ test_suite summary (%.0f s) ================\n', toc(t0));
lab = {'FAIL', 'PASS'};           % verdict labels, indexed by 1 + logical
for k = 1:numel(steps)
    s = lab{1 + ok(k)};
    if skip(k), s = 'SKIP'; end
    fprintf('  %s  %-24s %6.0f s  %s\n', s, steps{k}, dur(k), msg{k});
end
fprintf('  %s  define_constants.m SHA-256 after:  %s\n', lab{1 + hash_ok}, h1);
fprintf('  %s  lab_settings.m SHA-256 after:      %s\n', lab{1 + lab_ok}, hL1);
fprintf('  %s  no define_constants.PRISTINE left\n', lab{1 + lock_ok});
if ~(all(ok) && hash_ok && lock_ok && lab_ok)
    bad = strjoin(steps(~ok), ', ');   % names of the failed or skipped steps
    error('test_suite:fail', 'test_suite FAILED: %d of %d steps failed or skipped (%s); hash unchanged %d; lock absent %d; lab_settings unchanged %d', ...
          sum(~ok), numel(steps), bad, hash_ok, lock_ok, lab_ok);
end
fprintf('\ntest_suite PASS: %d of %d steps, define_constants.m unchanged\n', numel(steps), numel(steps));   % lab_settings.m unchanged too (its own line above); this line's text is quoted by the student procedure
end
