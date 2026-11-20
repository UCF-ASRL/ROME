function manual_makefail_base_plant()
%MANUAL_MAKEFAIL_BASE_PLANT  Source-level make-it-fail of test_base_plant (Plan 2 Task 4a).
%   Breaks base_plant_step.m, not the test. For each break k:
%     1. build a temp mirror <tmp>/ROME_merged/{., tests} holding identical copies of base_plant_step.m and
%        tests/test_base_plant.m; ROME_merged itself is never written;
%     2. apply ONE count-asserted text substitution (must match exactly once) to the copied base_plant_step.m;
%     3. cd into the temp tests folder, run the unchanged copy of test_base_plant (it addpaths its own parent,
%        the temp ROME_merged), check with `which` afterwards that base_plant_step resolved INSIDE the temp
%        mirror, and EXPECT error test_base_plant:fail whose message names the expected check.
%   Break 0 is a positive control: no substitution, the copy must PASS.
%   Errors at the end if any break did not fail as required, or the control did not pass.
%   Pure functions, no model, no define_constants, no network. Flat (CLAUDE.md section 5, crash 2).
root = fileparts(fileparts(mfilename('fullpath')));   % ROME_merged (char)
B = {  % k, old text (must occur once), new text, check that must appear in the failure, description
  0, '', '', '', 'control: unmodified copy';
  1, 'delta = P.fw_Kp * e + P.fw_Ki * e_int;', 'delta = P.fw_Kp * e + 2 * P.fw_Ki * e_int;', '(i step1 correct)', 'Ki 0.125 -> 0.25 (2*P.fw_Ki)';
  2, 'e = w_cmd - w_meas;', 'e = w_meas - w_cmd;', '(ii settles at cap)', 'PI error sign swapped';
  3, 'pwm = min(max(pwm, 0), 255);', 'pwm = pwm;', '(i step1 anchor pwm)', 'PWM clamp L250 removed';
  4, 'a = exp(-dt / tau);', 'a = 1 - dt / tau;', '(iii closed form 1e-12)', 'lag discretised by forward Euler'};
bad = [];  here0 = pwd;  path0 = path;   % failed breaks (vector), start folder, start path
for r = 1:size(B, 1)
    k = B{r,1};                          % break number
    tmp = tempname;  mk = fullfile(tmp, 'ROME_merged');  % temp mirror root (char)
    mkdir(fullfile(mk, 'tests'));
    copyfile(fullfile(root, 'base_plant_step.m'), mk);
    copyfile(fullfile(root, 'tests', 'test_base_plant.m'), fullfile(mk, 'tests'));
    fp = fullfile(mk, 'base_plant_step.m');  % the copy to break (char)
    if k > 0
        s = fileread(fp);
        nHit = numel(strfind(s, B{r,2}));
        if nHit ~= 1, error('manual_makefail_base_plant:anchor', 'break %d: pattern matched %d times', k, nHit); end
        s = strrep(s, B{r,2}, B{r,3});
        fid = fopen(fp, 'w');  fwrite(fid, s);  fclose(fid);
        if numel(strfind(fileread(fp), B{r,3})) < 1, error('manual_makefail_base_plant:write', 'break %d not written', k); end
    end
    cd(fullfile(mk, 'tests'));  path(path0);  clear functions;  %#ok<CLFUNC>
    ok = true;  msg = '';                % test passed, failure message
    try
        test_base_plant();
    catch err
        ok = false;  msg = err.message;
        if ~strcmp(err.identifier, 'test_base_plant:fail'), msg = ['WRONG ERROR ' err.identifier ': ' msg]; end
    end
    w = which('base_plant_step');        % where the function resolved (char)
    inTmp = startsWith(w, tmp);
    cd(here0);  path(path0);  clear functions;  %#ok<CLFUNC>
    if ~inTmp, error('manual_makefail_base_plant:path', 'base_plant_step did not resolve inside the temp mirror: %s', w); end
    if k == 0
        if ok, fprintf('BREAK 0 (%s): PASSED as required (control)\n', B{r,5});
        else, fprintf('BREAK 0: control FAILED: %s\n', msg);  bad(end+1) = 0; end %#ok<AGROW>
    else
        if ~ok && startsWith(msg, 'FAIL:') && contains(msg, B{r,4})
            fprintf('BREAK %d (%s): FAILED AS REQUIRED (expected %s present). Message: %s\n', k, B{r,5}, B{r,4}, msg);
        else
            fprintf('BREAK %d (%s): NOT FAILING AS REQUIRED (ok=%d, expected %s) %s\n', k, B{r,5}, ok, B{r,4}, msg);
            bad(end+1) = k; %#ok<AGROW>
        end
    end
    rmdir(tmp, 's');
end
if ~isempty(bad), error('manual_makefail_base_plant:notfailed', 'problems with breaks: %s', mat2str(bad)); end
fprintf('manual_makefail_base_plant: MAKE-IT-FAIL OK (control passed, %d source breaks all failed test_base_plant)\n', size(B,1)-1);
end
