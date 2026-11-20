function manual_makefail_run_mode_core()
%MANUAL_MAKEFAIL_RUN_MODE_CORE  Source-level make-it-fail of test_run_mode_core (focus plan F12, pure core).
%   Breaks the CODE UNDER TEST, never the test. For each break k:
%     1. build a temp mirror <tmp>/ROME_merged/tests holding identical copies of the real
%        tests/test_run_mode_core.m, run_mode_table.m and set_run_mode.m; ROME_merged itself is never written
%        (the mirror has no define_constants.m, so check (v) of the copy compares nothing real);
%     2. apply the break's count-asserted text substitutions (each must match exactly once) to one copied file;
%     3. cd into the temp tests folder, restore the starting path, clear functions, run the unchanged copy of
%        test_run_mode_core (it adds its own .., i.e. the temp mirror, first on the path, and asserts with
%        `which` that both functions resolved there), check with `which` afterwards that they resolve inside the
%        mirror, and EXPECT the error test_run_mode_core:fail whose message starts with 'FAIL: <the named check>'.
%        Any other outcome is scored NOT CAUGHT or WRONG REASON.
%   Break 0 is the positive control: no substitution, the copy must PASS.
%   Break 3 note: set_run_mode keeps line endings by construction (it replaces only the digit span of raw bytes),
%   so there is no preservation line to delete. The honest break is to INSERT the normalisation a text-mode read
%   would do (fopen 'rt' on Windows drops the CR of every CRLF): one line removing all CR bytes after the read.
%   Errors at the end (manual_makefail_run_mode_core:result) unless every break failed at its named check and the
%   control passed. Pure: no model, no define_constants, no network.
root = fileparts(fileparts(mfilename('fullpath')));       % ROME_merged
RT = 'run_mode_table.m';  SR = 'set_run_mode.m';
B = {  % k | file | {old (once), new; ...} | description | expected start of the failure message
  0, '', {}, 'control: unmodified copies', '';
  1, RT, {'    2, 1, 1, ''emulator'',  1, ', '    2, 1, 1, ''emulator'',  0, ';
          '    3, 1, 1, ''emulator'',  0, ', '    3, 1, 1, ''emulator'',  1, '}, ...
     'swap the arm_freeze values of modes 2 and 3', 'FAIL: (i)';
  2, SR, {'if nStrict ~= 1 || nLoose ~= 1', 'if false'}, ...
     'remove the count assertion', 'FAIL: (iv)';
  3, SR, {'txt = char(b);', 'b(b == 13) = [];  txt = char(b);'}, ...
     'drop CRLF preservation (normalise to LF on read)', 'FAIL: (iii) line endings';
  4, SR, {'if isfile(lockf)', 'if false'}, ...
     'drop the custody-lock refusal (fix round 1)', 'FAIL: (vii)'};
files = {RT, SR};
bad = [];  here0 = pwd;  path0 = path;
for r = 1:size(B, 1)
    k = B{r,1};
    tmp = tempname;  mk = fullfile(tmp, 'ROME_merged');
    mkdir(fullfile(mk, 'tests'));
    copyfile(fullfile(root, 'tests', 'test_run_mode_core.m'), fullfile(mk, 'tests'));
    for f = files, copyfile(fullfile(root, f{1}), fullfile(mk, f{1})); end
    if k > 0
        fp = fullfile(mk, B{r,2});
        s = fileread(fp);
        S = B{r,3};
        for j = 1:size(S, 1)
            nHit = numel(strfind(s, S{j,1}));
            if nHit ~= 1, error('manual_makefail_run_mode_core:anchor', 'break %d, substitution %d: pattern matched %d times in %s', k, j, nHit, B{r,2}); end
            s = strrep(s, S{j,1}, S{j,2});
        end
        fid = fopen(fp, 'w');  fwrite(fid, s);  fclose(fid);
    end
    cd(fullfile(mk, 'tests'));  path(path0);  clear functions;   %#ok<CLFUNC>
    passed = true;  msg = '';
    try
        test_run_mode_core();
    catch err
        passed = false;  msg = err.message;
        if ~strcmp(err.identifier, 'test_run_mode_core:fail'), msg = ['WRONG ERROR ' err.identifier ': ' msg]; end
    end
    % which() after the run: the test restored the path on exit, so re-add the mirror the way the test does
    addpath(mk);  wRT = which('run_mode_table');  wSR = which('set_run_mode');
    cd(here0);  path(path0);  clear functions;  %#ok<CLFUNC>
    if ~(startsWith(wRT, tmp) && startsWith(wSR, tmp))
        error('manual_makefail_run_mode_core:path', 'code under test did not resolve inside the temp mirror: %s | %s', wRT, wSR);
    end
    if k == 0
        if passed, fprintf('BREAK 0 (%s): PASSED as required (control)\n', B{r,4});
        else, fprintf('BREAK 0: control FAILED: %s\n', msg);  bad(end+1) = 0; end %#ok<AGROW>
    elseif passed
        fprintf('BREAK %d (%s): NOT CAUGHT, test_run_mode_core passed\n', k, B{r,4});  bad(end+1) = k; %#ok<AGROW>
    elseif startsWith(msg, B{r,5})
        fprintf('BREAK %d (%s): FAILED AS REQUIRED at the named check: %s\n', k, B{r,4}, msg);
    else
        fprintf('BREAK %d (%s): WRONG REASON (want "%s..."): %s\n', k, B{r,4}, B{r,5}, msg);  bad(end+1) = k; %#ok<AGROW>
    end
    try, rmdir(tmp, 's'); catch, end
end
if ~isempty(bad)
    error('manual_makefail_run_mode_core:result', 'breaks not caught as required: %s', mat2str(bad));
end
fprintf('manual_makefail_run_mode_core: control passed, %d of %d breaks failed at their named check\n', size(B, 1) - 1, size(B, 1) - 1);
end
