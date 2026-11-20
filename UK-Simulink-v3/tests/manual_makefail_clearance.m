function manual_makefail_clearance()
% MANUAL_MAKEFAIL_CLEARANCE  Shows that check_9dof's clearance check can FAIL.
%   Run by hand from MATLAB R2026a (author's manual re-run of the Task 7 make-it-fail,
%   which crashed with exit 139 on 19 Nov 2026 when run automatically):
%       >> cd <ROME_merged>/tests; manual_makefail_clearance
%   What it does:
%     1. copies ROME_merged to a temporary folder (ROME_merged itself is never touched),
%     2. in the COPY sets clearance_margin = 0.40 m, above the run's measured z_min of 0.32 m
%        (the edit is asserted to match exactly once, CLAUDE.md section 7),
%     3. runs check_9dof on the copy (scenario 1, the file's default),
%     4. EXPECTED: check_9dof errors, its output shows "[--]" on the clearance line and
%        "1 of 6 checks failed"; the script prints MAKE-IT-FAIL OK. Any other outcome errors.
%   WARNING: check_9dof can hit the intermittent MATLAB crash (exit 139, TRACKER section 0); a crash
%   is not a pass. The script prints MAKE-IT-FAIL OK only if all three hold: the error identifier is
%   check_9dof:failed, its message contains "1 of 6", and the clearance line in the captured output
%   reads "[--]". Otherwise it errors.
%   The whole command window output is saved to
%   docs/verification/<date>_task7_makefail_gate_margin_MANUAL.log, or to tempdir when that folder
%   does not exist (the lab repo; the path is printed).

here    = fileparts(fileparts(mfilename('fullpath')));        % ROME_merged folder
logDir  = fullfile(fileparts(here), 'docs', 'verification');  % verification logs folder
if ~isfolder(logDir)
    % The lab repo has no <repo>/docs/verification (final review I4, 19 Nov 2026); the review repo does.
    fprintf('manual_makefail_clearance: %s does not exist; the log goes to %s\n', logDir, tempdir);
    logDir = tempdir;                                         % fallback folder for the log (char)
end
logFile = fullfile(logDir, [datestr(now,'yyyy-mm-dd') '_task7_makefail_gate_margin_MANUAL.log']); %#ok<TNOW1,DATST>
tmp     = [tempname '_rome_makefail'];                        % throwaway copy of ROME_merged
oldDir  = pwd;  oldPath = path;                               % caller state, restored at the end
cleanup = onCleanup(@() restore_state(oldDir, oldPath, tmp)); %#ok<NASGU>

if isfile(logFile), delete(logFile); end                       % diary appends; the verdict reads this run only (the earlier log stays in git)
diary(logFile); diary on
fprintf('manual_makefail_clearance  %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf('MATLAB %s\n', version);
copyfile(here, tmp);
fprintf('copied %s\n    -> %s\n', here, tmp);

dcFile = fullfile(tmp, 'lab_settings.m');                     % the COPY's lab settings (clearance_margin moved there, F14a)
src    = fileread(dcFile);                                    % its text
pat    = 'clearance_margin = 0\.05;';                         % the line set by Task 7
nHit   = numel(regexp(src, pat));                             % matches of that line
assert(nHit == 1, 'expected exactly 1 match of "%s" in the copy, found %d', pat, nHit);
fid = fopen(dcFile, 'w');  fwrite(fid, regexprep(src, pat, 'clearance_margin = 0.40;'));  fclose(fid);
fprintf('copy edited: clearance_margin = 0.40 m (run z_min is 0.32 m, so the check must fail)\n');

cd(tmp);  addpath(tmp, '-begin');
err = [];                                                     % MException from check_9dof, [] if it passed
try
    check_9dof();
catch err
    fprintf('\ncheck_9dof errored:\n  id %s\n  %s\n', err.identifier, err.message);
end
diary off
logTxt = fileread(logFile);                                   % everything printed so far (the diary), char
ln     = regexp(logTxt, '[^\n]*arm clear of the floor[^\n]*', 'match');   % clearance line(s), cell of char
okId   = ~isempty(err) && strcmp(err.identifier, 'check_9dof:failed');       % the gate's own failure id (logical)
okMsg  = ~isempty(err) && contains(err.message, '1 of 6');                   % exactly one of six checks failed (logical)
okLine = numel(ln) == 1 && startsWith(strtrim(ln{1}), '[--]');               % clearance line reads [--] (logical)
diary on                                                      % append the verdict to the log
fprintf('verdict: error id check_9dof:failed %d, message has "1 of 6" %d, clearance line [--] %d\n', okId, okMsg, okLine);
if ~(okId && okMsg && okLine)
    error('manual_makefail_clearance:notdemonstrated', ...
          'MAKE-IT-FAIL NOT DEMONSTRATED (id %d, message %d, line %d); a crash or setup error is not a pass. See %s', okId, okMsg, okLine, logFile);
end
fprintf('MAKE-IT-FAIL OK (log: %s)\n', logFile);
diary off
end

function restore_state(oldDir, oldPath, tmp)
% Return to the caller's folder and path, and delete the throwaway copy.
diary off
cd(oldDir);  path(oldPath);
if exist(tmp, 'dir'), rmdir(tmp, 's'); end
end
