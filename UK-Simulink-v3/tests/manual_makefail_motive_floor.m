function manual_makefail_motive_floor()
%MANUAL_MAKEFAIL_MOTIVE_FLOOR  Source-level make-it-fail of test_motive_floor (focus plan F5a).
%   Breaks the functions under test, not the test. For each break k:
%     1. copy the ONE file to break (Optitrack/motive_to_floor.m, floor_to_motive.m or yaw_unwrap.m) into a fresh temp
%        folder; ROME_merged itself is never written;
%     2. apply ONE count-asserted text substitution (must match exactly once) to the copy;
%     3. put the temp folder FIRST on the path (test_motive_floor adds its own folders with '-end', so the
%        copy wins), run the unchanged test_motive_floor, check with `which` that the broken function resolved
%        INSIDE the temp folder, and EXPECT error test_motive_floor:fail whose message holds one of the
%        expected check tags.
%   Break 0 is a positive control: no substitution, the unmodified copy must PASS.
%   Errors at the end if any break did not fail as required, or the control did not pass.
%   Pure functions, no model, no define_constants, no network. Flat (CLAUDE.md section 5, crash 2).
root = fileparts(fileparts(mfilename('fullpath')));   % ROME_merged (char)
B = {  % k, file, old text (must occur once), new text, expected tags (any one), description
  0, 'motive_to_floor', '', '', {}, 'control: unmodified copy of motive_to_floor';
  1, 'motive_to_floor', 'psi = atan2(v(2, :), v(1, :));', 'psi = atan2(-v(2, :), v(1, :));', {'(iii-a'}, 'wrong yaw sign';
  2, 'motive_to_floor', 'pos = cal.scale * (cal.R * p) + cal.t;', 'pos = (cal.R * p) + cal.t;', {'(i round trip', '(ii '}, 'scale dropped';
  3, 'yaw_unwrap', 'psi = psi_prev + d;', 'psi = psi_raw;', {'(iii-b offset lap'}, 'unwrap removed (raw heading passed through)';
  4, 'floor_to_motive', 'ay = sqrt(1 - A(1,1) + A(2,2) - A(3,3)) / 2;', 'ay = sqrt(1 + A(1,1) + A(2,2) - A(3,3)) / 2;', {'(i-b'}, 'Shepperd A22 branch: sign of A11 in ay flipped (only cal 7, Ry(180), takes this branch)'};
bad = [];  path0 = path;                    % failed breaks (vector), start path (char)
for r = 1:size(B, 1)
    k = B{r,1};                             % break number
    fn = B{r,2};                            % function broken (char)
    tmp = tempname;  mkdir(tmp);            % temp folder holding the one copy (char)
    copyfile(fullfile(root, 'Optitrack', [fn '.m']), tmp);
    fp = fullfile(tmp, [fn '.m']);          % the copy to break (char)
    if k > 0
        s = fileread(fp);
        nHit = numel(strfind(s, B{r,3}));
        if nHit ~= 1, error('manual_makefail_motive_floor:anchor', 'break %d: pattern matched %d times', k, nHit); end
        s = strrep(s, B{r,3}, B{r,4});
        fid = fopen(fp, 'w');  fwrite(fid, s);  fclose(fid);
        if numel(strfind(fileread(fp), B{r,4})) ~= 1, error('manual_makefail_motive_floor:write', 'break %d not written', k); end
    end
    path(path0);  addpath(tmp);  clear functions;  %#ok<CLFUNC>
    ok = true;  msg = '';                   % test passed, failure message
    try
        test_motive_floor();
    catch err
        ok = false;  msg = err.message;
        if ~strcmp(err.identifier, 'test_motive_floor:fail'), msg = ['WRONG ERROR ' err.identifier ': ' msg]; end
    end
    w = which(fn);                          % where the broken function resolved (char)
    inTmp = startsWith(w, tmp);
    path(path0);  clear functions;  %#ok<CLFUNC>
    rmdir(tmp, 's');
    if ~inTmp, error('manual_makefail_motive_floor:path', '%s did not resolve inside the temp folder: %s', fn, w); end
    if k == 0
        if ok, fprintf('BREAK 0 (%s): PASSED as required (control; resolved %s)\n', B{r,6}, w);
        else, fprintf('BREAK 0: control FAILED: %s\n', msg);  bad(end+1) = 0; end %#ok<AGROW>
    else
        hit = ~ok && startsWith(msg, 'FAIL:') && any(cellfun(@(tg) contains(msg, tg), B{r,5}));
        if hit
            fprintf('BREAK %d (%s, %s): FAILED AS REQUIRED (expected one of %s). Message: %s\n', k, fn, B{r,6}, strjoin(B{r,5}, ' | '), msg);
        else
            fprintf('BREAK %d (%s, %s): NOT FAILING AS REQUIRED (ok=%d, expected one of %s) %s\n', k, fn, B{r,6}, ok, strjoin(B{r,5}, ' | '), msg);
            bad(end+1) = k; %#ok<AGROW>
        end
    end
end
if ~isempty(bad), error('manual_makefail_motive_floor:notfailed', 'problems with breaks: %s', mat2str(bad)); end
fprintf('manual_makefail_motive_floor: MAKE-IT-FAIL OK (control passed, %d source breaks all failed test_motive_floor)\n', size(B,1)-1);
end
