function manual_makefail_prerun_core()
%MANUAL_MAKEFAIL_PRERUN_CORE  Source-level make-it-fail of test_prerun_core (focus plan F2 + F11).
%   Breaks the CODE UNDER TEST, never the test. For each break k:
%     1. build a temp mirror <tmp>/ROME_merged/{tests,lab} holding identical copies of the real
%        tests/test_prerun_core.m, prerun_check_core.m, allowed_ranges.m and lab/arm_park_record.m;
%        ROME_merged itself is never written;
%     2. apply ONE count-asserted text substitution (must match exactly once) to one copied file;
%     3. cd into the temp tests folder, restore the starting path, clear functions, run the unchanged copy of
%        test_prerun_core (it adds its own ../ and ../lab, i.e. the temp mirror, first on the path), check with
%        `which` that prerun_check_core and arm_park_record resolved INSIDE the temp mirror, and EXPECT the error
%        test_prerun_core:fail whose message starts with 'FAIL: <the named check>'. Any other outcome is scored
%        NOT CAUGHT or WRONG REASON.
%   Break 0 is the positive control: no substitution, the copy must PASS.
%   Errors at the end (manual_makefail_prerun_core:result) unless every break failed at its named check and the
%   control passed. Pure: no model, no define_constants, no network.
root = fileparts(fileparts(mfilename('fullpath')));       % ROME_merged
PC = 'prerun_check_core.m';  AR = 'allowed_ranges.m';  PR = fullfile('lab', 'arm_park_record.m');
B = {  % k, file, old text (once), new text, description, expected start of the failure message
  0, '', '', '', 'control: unmodified copies', '';
  1, PC, 'badJ = find(~(d <= C.park_tol_deg));', 'badJ = [];', ...
     'drop the frozen-vs-parked angle comparison', 'FAIL: (v)';
  2, AR, '''interval'', 0, 120, false, true', '''interval'', 0, Inf, false, true', ...
     'drop the max_rpm upper bound (120)', 'FAIL: (iii)';
  3, PC, 'if ~srcOk || strcmp(src, ''optitrack'')', 'if true', ...
     'treat the emulator like optitrack', 'FAIL: (iv)';
  4, PC, 'ok = isempty(reasons);', sprintf('if numel(reasons) > 1, reasons = reasons(1); end\nok = isempty(reasons);'), ...
     'return the first reason only', 'FAIL: (vi)';
  5, PC, 'strcmp(rec.dc_sha, shaNow)', 'true', ...
     'drop the parked record''s define_constants SHA comparison', 'FAIL: (ii) park record other SHA';
  6, PR, 'park_history = [hist, e]', 'park_history = e', ...
     'overwrite the park history instead of appending', 'FAIL: (viii)'};   % break 7 (frozen_feedback) removed with that item, author 19 Nov
files = {PC, AR, PR};
bad = [];  here0 = pwd;  path0 = path;
for r = 1:size(B, 1)
    k = B{r,1};
    tmp = tempname;  mk = fullfile(tmp, 'ROME_merged');
    mkdir(fullfile(mk, 'tests'));  mkdir(fullfile(mk, 'lab'));
    copyfile(fullfile(root, 'tests', 'test_prerun_core.m'), fullfile(mk, 'tests'));
    for f = files, copyfile(fullfile(root, f{1}), fullfile(mk, f{1})); end
    if k > 0
        fp = fullfile(mk, B{r,2});
        s = fileread(fp);
        nHit = numel(strfind(s, B{r,3}));
        if nHit ~= 1, error('manual_makefail_prerun_core:anchor', 'break %d: pattern matched %d times in %s', k, nHit, B{r,2}); end
        s = strrep(s, B{r,3}, B{r,4});
        fid = fopen(fp, 'w');  fwrite(fid, s);  fclose(fid);
    end
    cd(fullfile(mk, 'tests'));  path(path0);  clear functions;   %#ok<CLFUNC>
    passed = true;  msg = '';
    try
        test_prerun_core();
    catch err
        passed = false;  msg = err.message;
        if ~strcmp(err.identifier, 'test_prerun_core:fail'), msg = ['WRONG ERROR ' err.identifier ': ' msg]; end
    end
    % which() after the run: the test restored the path on exit, so re-add the mirror the way the test does
    addpath(mk, fullfile(mk, 'lab'));  wPC = which('prerun_check_core');  wPR = which('arm_park_record');
    cd(here0);  path(path0);  clear functions;  %#ok<CLFUNC>
    if ~(startsWith(wPC, tmp) && startsWith(wPR, tmp))
        error('manual_makefail_prerun_core:path', 'code under test did not resolve inside the temp mirror: %s | %s', wPC, wPR);
    end
    if k == 0
        if passed, fprintf('BREAK 0 (%s): PASSED as required (control)\n', B{r,5});
        else, fprintf('BREAK 0: control FAILED: %s\n', msg);  bad(end+1) = 0; end %#ok<AGROW>
    elseif passed
        fprintf('BREAK %d (%s): NOT CAUGHT, test_prerun_core passed\n', k, B{r,5});  bad(end+1) = k; %#ok<AGROW>
    elseif startsWith(msg, B{r,6})
        fprintf('BREAK %d (%s): FAILED AS REQUIRED at the named check: %s\n', k, B{r,5}, msg);
    else
        fprintf('BREAK %d (%s): WRONG REASON (want "%s..."): %s\n', k, B{r,5}, B{r,6}, msg);  bad(end+1) = k; %#ok<AGROW>
    end
    try, rmdir(tmp, 's'); catch, end
end
if ~isempty(bad)
    error('manual_makefail_prerun_core:result', 'breaks not caught as required: %s', mat2str(bad));
end
fprintf('manual_makefail_prerun_core: control passed, %d of %d breaks failed at their named check\n', size(B, 1) - 1, size(B, 1) - 1);
end
