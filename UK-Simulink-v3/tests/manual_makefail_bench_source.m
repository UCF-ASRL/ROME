function manual_makefail_bench_source()
%MANUAL_MAKEFAIL_BENCH_SOURCE  Source-level make-it-fail of test_bench (Plan 3 Task 2, fix round 2).
%   Breaks the EVALUATOR SOURCE, not the test. For each break k:
%     1. build a temp mirror  <tmp>/ROME_merged/{tests,lab,analysis}  holding an identical copy of the real
%        tests/test_bench.m, lab/bench_*.m, analysis/romefw_to_arcs_deg.m, analysis/ar3_fk.m and define_constants.m
%        (the last is only read as text by the test); ROME_merged itself is never written;
%     2. apply ONE count-asserted text substitution (must match exactly once) to one copied evaluator file;
%     3. cd into the temp tests folder, check with `which` that the evaluator resolves INSIDE the temp mirror,
%        run the unchanged copy of test_bench and EXPECT it to error with test_bench:fail; the failing check name
%        is printed from the error message.
%   Break 0 is a positive control: no substitution, the copy of test_bench must PASS (proves the harness runs).
%   Errors at the end if any break did not fail as required, or the control did not pass.
%   Pure functions, synthetic data, no model, no define_constants execution, no network.
root = fileparts(fileparts(mfilename('fullpath')));       % ROME_merged
JS = 'bench_joint_signs.m';  PT = 'bench_parse_telemetry.m';  WO = 'bench_wheel_order.m';
B = {  % file, old text (must occur once), new text, description
  0, '', '', '', 'control: unmodified copy';
  1, JS, 'implied_sign(i) ~= define_sign(i)', 'implied_sign(i) == define_sign(i)', 'sign comparison inverted';
  2, JS, 'if followed(i) && any(j == [2 5 6])', 'if any(j == [2 5 6])', 'followed(i) filter dropped from map observations';
  3, JS, 'maps = {''rome_dirpin'', ''arcs_caldir''};', 'maps = {''arcs_caldir'', ''rome_dirpin''};', 'map order swapped';
  4, PT, 'need = 9;  nv = 4;', 'need = 8;  nv = 4;', 'GV field-count rule 9 -> 8';
  5, JS, 'lp(m) = a0(2) + 90;', 'lp(m) = a0(2) + 0;', 'lean offset +90 -> +0';
  6, WO, 'index_ok(i) = isequal(act, e.k_cmd);', 'index_ok(i) = isequal(act, mod(e.k_cmd, 4) + 1);', 'wheel channel index off by one'};
bad = [];  here0 = pwd;  path0 = path;
for r = 1:size(B, 1)
    k = B{r,1};
    tmp = tempname;  mk = fullfile(tmp, 'ROME_merged');
    mkdir(fullfile(mk, 'tests'));  mkdir(fullfile(mk, 'lab'));  mkdir(fullfile(mk, 'analysis'));
    copyfile(fullfile(root, 'tests', 'test_bench.m'), fullfile(mk, 'tests'));
    copyfile(fullfile(root, 'define_constants.m'), mk);
    copyfile(fullfile(root, 'analysis', 'romefw_to_arcs_deg.m'), fullfile(mk, 'analysis'));
    copyfile(fullfile(root, 'analysis', 'ar3_fk.m'), fullfile(mk, 'analysis'));
    for f = {JS, PT, WO}, copyfile(fullfile(root, 'lab', f{1}), fullfile(mk, 'lab')); end
    if k > 0
        fp = fullfile(mk, 'lab', B{r,2});
        s = fileread(fp);
        nHit = numel(strfind(s, B{r,3}));
        if nHit ~= 1, error('manual_makefail_bench_source:anchor', 'break %d: pattern matched %d times in %s', k, nHit, B{r,2}); end
        s = strrep(s, B{r,3}, B{r,4});
        fid = fopen(fp, 'w');  fwrite(fid, s);  fclose(fid);
    end
    cd(fullfile(mk, 'tests'));  path(path0);  clear functions;   %#ok<CLFUNC>
    w = which('bench_joint_signs');  % resolves via the test's own addpath of ../lab
    ok = true;  msg = '';
    try
        test_bench();
    catch err
        ok = false;  msg = err.message;
        if ~strcmp(err.identifier, 'test_bench:fail'), msg = ['WRONG ERROR ' err.identifier ': ' msg]; end
    end
    inTmp = startsWith(which('bench_joint_signs'), tmp);
    cd(here0);  path(path0);  clear functions;  %#ok<CLFUNC>
    if ~inTmp, error('manual_makefail_bench_source:path', 'evaluator did not resolve inside the temp mirror: %s', w); end
    if k == 0
        if ok, fprintf('BREAK 0 (%s): PASSED as required (control)\n', B{r,5});
        else, fprintf('BREAK 0: control FAILED: %s\n', msg);  bad(end+1) = 0; end %#ok<AGROW>
    else
        if ~ok && startsWith(msg, 'FAIL:')
            fprintf('BREAK %d (%s): FAILED AS REQUIRED at check: %s\n', k, B{r,5}, msg);
        else
            fprintf('BREAK %d (%s): NOT FAILING AS REQUIRED (ok=%d) %s\n', k, B{r,5}, ok, msg);  bad(end+1) = k; %#ok<AGROW>
        end
    end
    rmdir(tmp, 's');
end
if ~isempty(bad), error('manual_makefail_bench_source:notfailed', 'problems with breaks: %s', mat2str(bad)); end
fprintf('manual_makefail_bench_source: MAKE-IT-FAIL OK (control passed, %d source breaks all failed test_bench)\n', size(B,1)-1);
end
