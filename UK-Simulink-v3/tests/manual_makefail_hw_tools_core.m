function manual_makefail_hw_tools_core()
%MANUAL_MAKEFAIL_HW_TOOLS_CORE  Source-level make-it-fail of test_hw_tools_core (focus plan F3 core, 19 Nov 2026).
%   Breaks the SOURCE of save_run.m / compare_hw_sim_core.m, never the test. For each break k:
%     1. build a temp mirror <tmp>/ROME_merged/{tests,tools,runs} with identical copies of save_run.m,
%        compare_hw_sim_core.m, ik9_fk.m, mass_at.m, tools/p2t1c_pacing_deficit.m, runs/case_1.mat and
%        tests/test_hw_tools_core.m; ROME_merged itself is never written;
%     2. apply ONE count-asserted text substitution (must match exactly once) to the copied source file;
%     3. cd into the temp tests folder (the test then puts the mirror first on the path), check with `which`
%        that both functions resolve INSIDE the mirror, run the unchanged copy of test_hw_tools_core and EXPECT
%        it to error with test_hw_tools_core:fail; the failed checks are printed from the message.
%   Break 0 is a positive control: no substitution, the copy must PASS.
%   Errors at the end if any break did not fail as required, or the control did not pass.
root = fileparts(fileparts(mfilename('fullpath')));       % ROME_merged (char)
SR = 'save_run.m';  CC = 'compare_hw_sim_core.m';
B = {  % k, file, old text (must occur once), new text, description
  0, '', '', '', 'control: unmodified copy';
  1, SR, 'while ~claim_file(f)', 'while false', 'no-overwrite guard removed';
  2, CC, 'yaw_e = atan2(sin(dyaw), cos(dyaw));', 'yaw_e = 0 * dyaw;', 'heading dropped from the base error';
  3, CC, '[dro, droSrc] = detect_dropouts(H, P);', 'dro = zeros(0, 2);  droSrc = ''ignored'';', 'dropout ignored';
  4, CC, 'S.l, S.alphas(:));', 'S.l, -S.alphas(:));', 'wheel-map sign wrong (sin term of every wheel flipped)'};
files = {'save_run.m', 'compare_hw_sim_core.m', 'ik9_fk.m', 'mass_at.m'};
bad = [];  here0 = pwd;  path0 = path;
for r = 1:size(B, 1)
    k = B{r, 1};
    tmp = tempname;  mk = fullfile(tmp, 'ROME_merged');
    mkdir(fullfile(mk, 'tests'));  mkdir(fullfile(mk, 'tools'));  mkdir(fullfile(mk, 'runs'));
    for f = files, copyfile(fullfile(root, f{1}), mk); end
    copyfile(fullfile(root, 'tools', 'p2t1c_pacing_deficit.m'), fullfile(mk, 'tools'));
    copyfile(fullfile(root, 'runs', 'case_1.mat'), fullfile(mk, 'runs'));
    copyfile(fullfile(root, 'tests', 'test_hw_tools_core.m'), fullfile(mk, 'tests'));
    if k > 0
        fp = fullfile(mk, B{r, 2});
        s = fileread(fp);
        nHit = numel(strfind(s, B{r, 3}));
        if nHit ~= 1, error('manual_makefail_hw_tools_core:anchor', 'break %d: pattern matched %d times in %s', k, nHit, B{r, 2}); end
        s = strrep(s, B{r, 3}, B{r, 4});
        fid = fopen(fp, 'w');  fwrite(fid, s);  fclose(fid);
    end
    cd(fullfile(mk, 'tests'));  path(path0);  clear functions;   %#ok<CLFUNC>
    ok = true;  msg = '';
    try
        test_hw_tools_core();
    catch err
        ok = false;  msg = err.message;
        if ~strcmp(err.identifier, 'test_hw_tools_core:fail'), msg = ['WRONG ERROR ' err.identifier ': ' msg]; end
    end
    w1 = which('save_run');  w2 = which('compare_hw_sim_core');
    inTmp = startsWith(w1, tmp) && startsWith(w2, tmp);
    cd(here0);  path(path0);  clear functions;   %#ok<CLFUNC>
    rmdir(tmp, 's');
    if ~inTmp, error('manual_makefail_hw_tools_core:path', 'sources did not resolve inside the mirror: %s | %s', w1, w2); end
    if k == 0
        if ok, fprintf('BREAK 0 (%s): PASSED as required (control)\n', B{r, 5});
        else, fprintf('BREAK 0: control FAILED: %s\n', msg);  bad(end+1) = 0; end %#ok<AGROW>
    elseif ~ok && startsWith(msg, 'FAIL:')
        fprintf('BREAK %d (%s): FAILED AS REQUIRED: %s\n', k, B{r, 5}, msg);
    else
        fprintf('BREAK %d (%s): NOT FAILING AS REQUIRED (ok=%d) %s\n', k, B{r, 5}, ok, msg);  bad(end+1) = k; %#ok<AGROW>
    end
end
if ~isempty(bad), error('manual_makefail_hw_tools_core:notfailed', 'problems with breaks: %s', mat2str(bad)); end
fprintf('manual_makefail_hw_tools_core: MAKE-IT-FAIL OK (control passed, %d source breaks all failed test_hw_tools_core)\n', size(B, 1) - 1);
end
