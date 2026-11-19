function test_suite()
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
%   and asserts that define_constants.m has the same SHA-256 before and after
%   (gate_all, check_guard and run_9dof_cases rewrite it under the PRISTINE lock,
%   CLAUDE.md section 20.4) and that no define_constants.PRISTINE is left behind.
%   A function, not a script: run_9dof_cases clears the base workspace.
%   Written flat, without local functions (CLAUDE.md section 5, crash 2).
here = fileparts(mfilename('fullpath'));    % ROME_merged/tests
root = fileparts(here);                     % ROME_merged
addpath(here, root);
dc   = fullfile(root, 'define_constants.m');            % the file the checks take custody of
lock = fullfile(root, 'define_constants.PRISTINE');      % custody lock written by gate_all / check_guard / run_9dof_cases
assert(~isfile(lock), 'test_suite:lock', 'define_constants.PRISTINE exists before the suite: restore it first (CLAUDE.md 20.4)');
fid = fopen(dc, 'r');  b0 = fread(fid, Inf, '*uint8');  fclose(fid);     % bytes before
md = java.security.MessageDigest.getInstance('SHA-256');
h0 = lower(reshape(dec2hex(typecast(md.digest(b0), 'uint8'), 2).', 1, []));   % SHA-256 before, hex
fprintf('test_suite: %s\n  define_constants.m SHA-256 before: %s\n', root, h0);
t0 = tic;

steps = {'gate_all', 'check_guard', 'derivation_checks', 'verify_scenarios', ...
         'verify_wheelmap', 'verify_eetraj', 'run_9dof_cases(''sim'')', 'test_v3_table3', 'test_run_length', 'test_ar3_fk', 'test_pacing', 'test_commanding_check', 'test_clearance', 'test_motive_reader'};   % step names, in run order
ok   = false(1, numel(steps));    % pass flag per step (a skipped step is not a pass)
skip = false(1, numel(steps));    % true for a step not run because a step it depends on failed
msg  = repmat({''}, 1, numel(steps));   % error or skip message per step (char, '' when none)
dur  = zeros(1, numel(steps));    % wall time per step (s)
for k = 1:numel(steps)
    fprintf('\n================ %d/%d %s ================\n', k, numel(steps), steps{k});
    cd(root);
    ts = tic;                     % timer of this step
    if k == 8 && ~ok(7)
        % test_v3_table3 checks the runs written by step 7; without them it would check old files.
        skip(k) = true;
        msg{k} = 'SKIPPED: run_9dof_cases failed, runs/ would be stale';
        fprintf('  %s\n', msg{k});
        dur(k) = toc(ts);
        continue
    end
    try
        switch k
            case 1, gate_all();
            case 2, check_guard();
            case 3, derivation_checks();
            case 4, verify_scenarios();
            case 5, verify_wheelmap();
            case 6, verify_eetraj();
            case 7, run_9dof_cases('sim');
            case 8, test_v3_table3(fullfile(root, 'runs'), root);
            case 9, test_run_length();
            case 10, test_ar3_fk();
            case 11, test_pacing();
            case 12, test_commanding_check();
            case 13, test_clearance();
            case 14, test_motive_reader();
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

fprintf('\n================ test_suite summary (%.0f s) ================\n', toc(t0));
lab = {'FAIL', 'PASS'};           % verdict labels, indexed by 1 + logical
for k = 1:numel(steps)
    s = lab{1 + ok(k)};
    if skip(k), s = 'SKIP'; end
    fprintf('  %s  %-24s %6.0f s  %s\n', s, steps{k}, dur(k), msg{k});
end
fprintf('  %s  define_constants.m SHA-256 after:  %s\n', lab{1 + hash_ok}, h1);
fprintf('  %s  no define_constants.PRISTINE left\n', lab{1 + lock_ok});
if ~(all(ok) && hash_ok && lock_ok)
    bad = strjoin(steps(~ok), ', ');   % names of the failed or skipped steps
    error('test_suite:fail', 'test_suite FAILED: %d of %d steps failed or skipped (%s); hash unchanged %d; lock absent %d', ...
          sum(~ok), numel(steps), bad, hash_ok, lock_ok);
end
fprintf('\ntest_suite PASS: %d of %d steps, define_constants.m unchanged\n', numel(steps), numel(steps));
end
