function manual_makefail_camera_guard_core()
%MANUAL_MAKEFAIL_CAMERA_GUARD_CORE  Source-level make-it-fail of test_camera_guard_core (focus plan F6, pure core).
%   Breaks the CODE UNDER TEST, never the test. Same harness as manual_makefail_run_mode_core. For each break k:
%     1. build a temp mirror <tmp>/ROME_merged/tests holding identical copies of the real
%        tests/test_camera_guard_core.m, camera_guard_core.m and arm_ready_core.m; ROME_merged itself is never written;
%     2. apply the break's count-asserted text substitutions (each must match exactly once) to one copied file;
%     3. cd into the temp tests folder, restore the starting path, clear functions, run the unchanged copy of
%        test_camera_guard_core (it adds its own .., i.e. the temp mirror, first on the path and asserts with `which`
%        that both functions resolved there), check with `which` afterwards that they resolve inside the mirror, and
%        EXPECT the error test_camera_guard_core:fail whose message starts with 'FAIL: <the named check>'.
%        Any other outcome is scored NOT CAUGHT or WRONG REASON.
%   Break 0 is the positive control: no substitution, the copy must PASS.
%   Breaks (what a regression would look like -> the check that must catch it):
%     1 a good frame trusts the valid flag alone (no finite-pose test)           -> (b)
%     2 the dropout limit compares with >= (stops one sample early)               -> (f) boundary
%     3 stop is not latched (a later good frame clears it)                        -> (g)
%     4 a dropout passes the incoming pose instead of the last good one           -> (d)
%     5 the dropout counter is not restarted by a good frame                      -> (h)
%     6 arm: armReady alone starts the run (a bare ARM_READY on the seed angles)  -> (m)
%     7 arm: go is not latched (drops when armReady drops)                        -> (p)
%   Errors at the end (manual_makefail_camera_guard_core:result) unless every break failed at its named check and the
%   control passed. Pure: no model, no define_constants, no network.
root = fileparts(fileparts(mfilename('fullpath')));       % ROME_merged
CG = 'camera_guard_core.m';  AR = 'arm_ready_core.m';
B = {  % k | file | {old (once), new; ...} | description | expected start of the failure message
  0, '', {}, 'control: unmodified copies', '';
  1, CG, {'double(valid) ~= 0 && all(isfinite(pose(:)));', 'double(valid) ~= 0;'}, ...
     'good frame = valid flag only, no finite-pose test', 'FAIL: (b)';
  2, CG, {'if S.n_drop > n_drop_ok', 'if S.n_drop >= n_drop_ok'}, ...
     'dropout limit compared with >=', 'FAIL: (f)';
  3, CG, {'if S.stopped', 'if false'}, ...
     'stop not latched', 'FAIL: (g)';
  4, CG, {'pose_out = S.last_pose;', 'pose_out = pose(:);'}, ...
     'dropout passes the incoming pose', 'FAIL: (d)';
  5, CG, {'S.n_drop = 0;', ''}, ...
     'dropout counter not restarted by a good frame', 'FAIL: (h)';
  6, AR, {'if rdy && fresh', 'if rdy'}, ...
     'arm: armReady alone starts the run', 'FAIL: (m)';
  7, AR, {'if S.go', 'if false'}, ...
     'arm: go not latched', 'FAIL: (p)'};
files = {CG, AR};
bad = [];  here0 = pwd;  path0 = path;
for r = 1:size(B, 1)
    k = B{r,1};
    tmp = tempname;  mk = fullfile(tmp, 'ROME_merged');
    mkdir(fullfile(mk, 'tests'));
    copyfile(fullfile(root, 'tests', 'test_camera_guard_core.m'), fullfile(mk, 'tests'));
    for f = files, copyfile(fullfile(root, f{1}), fullfile(mk, f{1})); end
    if k > 0
        fp = fullfile(mk, B{r,2});
        s = fileread(fp);
        S = B{r,3};
        for j = 1:size(S, 1)
            nHit = numel(strfind(s, S{j,1}));
            if nHit ~= 1, error('manual_makefail_camera_guard_core:anchor', 'break %d, substitution %d: pattern matched %d times in %s', k, j, nHit, B{r,2}); end
            s = strrep(s, S{j,1}, S{j,2});
        end
        fid = fopen(fp, 'w');  fwrite(fid, s);  fclose(fid);
    end
    cd(fullfile(mk, 'tests'));  path(path0);  clear functions;   %#ok<CLFUNC>
    passed = true;  msg = '';
    try
        test_camera_guard_core();
    catch err
        passed = false;  msg = err.message;
        if ~strcmp(err.identifier, 'test_camera_guard_core:fail'), msg = ['WRONG ERROR ' err.identifier ': ' msg]; end
    end
    % which() after the run: the test restored the path on exit, so re-add the mirror the way the test does
    addpath(mk);  wCG = which('camera_guard_core');  wAR = which('arm_ready_core');
    cd(here0);  path(path0);  clear functions;  %#ok<CLFUNC>
    if ~(startsWith(wCG, tmp) && startsWith(wAR, tmp))
        error('manual_makefail_camera_guard_core:path', 'code under test did not resolve inside the temp mirror: %s | %s', wCG, wAR);
    end
    if k == 0
        if passed, fprintf('BREAK 0 (%s): PASSED as required (control)\n', B{r,4});
        else, fprintf('BREAK 0: control FAILED: %s\n', msg);  bad(end+1) = 0; end %#ok<AGROW>
    elseif passed
        fprintf('BREAK %d (%s): NOT CAUGHT, test_camera_guard_core passed\n', k, B{r,4});  bad(end+1) = k; %#ok<AGROW>
    elseif startsWith(msg, B{r,5})
        fprintf('BREAK %d (%s): FAILED AS REQUIRED at the named check: %s\n', k, B{r,4}, msg);
    else
        fprintf('BREAK %d (%s): WRONG REASON (want "%s..."): %s\n', k, B{r,4}, B{r,5}, msg);  bad(end+1) = k; %#ok<AGROW>
    end
    try, rmdir(tmp, 's'); catch, end
end
if ~isempty(bad)
    error('manual_makefail_camera_guard_core:result', 'breaks not caught as required: %s', mat2str(bad));
end
fprintf('manual_makefail_camera_guard_core: control passed, %d of %d breaks failed at their named check\n', size(B, 1) - 1, size(B, 1) - 1);
end
