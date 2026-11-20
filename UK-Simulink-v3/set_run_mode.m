function [M, oldMode] = set_run_mode(n, dcFile)
%SET_RUN_MODE  Select run mode n (1-4) by rewriting ONLY the run_mode line of define_constants.m (focus plan F12).
%   set_run_mode(n)          edits define_constants.m in this function's own folder.
%   set_run_mode(n, dcFile)  edits dcFile instead (char path; the tests use a temp copy).
%   [M, oldMode] = set_run_mode(...) also returns run_mode_table(n) (struct) and the run_mode the file held (double).
%
%   What it does, in this order:
%     1. validates n through run_mode_table (error run_mode_table:bad, listing the four modes). This happens
%        before the file is opened, so a bad n never touches it;
%     1b. refuses while <name>.PRISTINE exists next to the file (set_run_mode:locked): a harness holding the file
%        would write its own copy back and silently undo the mode (fix round 1);
%     2. reads the file as raw bytes (fread uint8, no text decoding, no line-ending conversion);
%     3. count-asserts the run_mode line: exactly ONE line must match ^run_mode[ \t]*=[ \t]*\d+[ \t]*; and exactly
%        ONE line may assign run_mode at all (^[ \t]*run_mode[ \t]*=, not ==), else error set_run_mode:count and
%        the file is untouched. [ \t] is used where the brief wrote \s because MATLAB's \s also matches \r and \n,
%        so "run_mode =<newline>3;" would have been accepted as one line;
%     4. replaces ONLY the digits of that match. Every other byte (the spacing around '=', a trailing comment, the
%        line ending, CRLF or LF, non-ASCII bytes) is copied unchanged, so line endings are preserved by
%        construction, not by a conversion step;
%     5. writes the new bytes to a temp file in the same folder (name not ending in .m, so never on the path),
%        moves it over the original, and re-reads the original to check the bytes landed (set_run_mode:write /
%        set_run_mode:verify; the temp file is deleted on any failure). When the file already holds run_mode = n
%        nothing is written (idempotent);
%     6. prints the mode banner line (run_mode_table(n).banner) and one line saying what changed.
%   It does NOT run define_constants: the wiring step (after F7) decides who re-runs it. Until it is re-run, the
%   workspace still holds the previous mode, and arm_freeze_banner refuses a Run because the file's SHA-256 no
%   longer equals dc_sha_at_run (arm_freeze_banner:stale).
%   The mapping n -> flags lives in run_mode_table.m only; this file holds no flag value.
M = run_mode_table(n);                                    % validates n; errors before any file I/O (struct)
if nargin < 2 || isempty(dcFile)
    dcFile = fullfile(fileparts(mfilename('fullpath')), 'define_constants.m');   % default: next to this file (char)
end
dcFile = char(dcFile);
[lockDir, lockBase] = fileparts(dcFile);                  % folder and name of the target (char)
lockf = fullfile(lockDir, [lockBase '.PRISTINE']);        % custody lock of gate_all / check_guard / run_9dof_cases / test_run_length (char)
if isfile(lockf)
    error('set_run_mode:locked', ['%s exists: a check (test_suite, gate_all, check_guard, run_9dof_cases or test_run_length) holds %s and writes its own copy back when it ' ...
          'ends, so a mode set now would be lost. Wait for it to finish (if it was interrupted, restore the file from the ' ...
          'lock first). File left unchanged.'], lockf, dcFile);
end

b = read_bytes(dcFile);                                   % file bytes as read (uint8 row)
txt = char(b);                                            % one char per byte, for the regexp only (char row)
[~, tok] = regexp(txt, '^run_mode[ \t]*=[ \t]*(\d+)[ \t]*;', 'match', 'tokenExtents', 'lineanchors');   % strict matches
nStrict = numel(tok);                                     % lines of the form run_mode = <digits>; (count)
nLoose  = numel(regexp(txt, '^[ \t]*run_mode[ \t]*=(?!=)', 'start', 'lineanchors'));   % lines assigning run_mode at all (count)
if nStrict ~= 1 || nLoose ~= 1
    error('set_run_mode:count', ['%s: expected exactly one line "run_mode = <1-4>;" (found %d) and exactly one ' ...
        'line assigning run_mode (found %d). File left unchanged.'], dcFile, nStrict, nLoose);
end
d = tok{1}(1, :);                                         % [first last] byte index of the digits (1x2)
oldMode = str2double(txt(d(1):d(2)));                     % run_mode the file held (double)
nb = [b(1:d(1)-1), uint8(sprintf('%d', M.n)), b(d(2)+1:end)];   % new bytes: only the digit span replaced (uint8 row)

if isequal(nb, b)
    what = 'unchanged (already this mode), nothing written';   % what happened, for the printed line (char)
else
    [folder, base, ext] = fileparts(dcFile);              % folder, name and extension of the target (char)
    if isempty(folder), folder = pwd; end
    [~, u] = fileparts(tempname);                         % unique suffix (char)
    tmpf = fullfile(folder, ['.' base ext '.set_run_mode_' u '.tmp']);   % temp file, same folder, not *.m (char)
    try
        fid = fopen(tmpf, 'w');                           % temp file id (double, -1 on failure)
        if fid < 0, error('set_run_mode:write', 'cannot create the temp file %s', tmpf); end
        cnt = fwrite(fid, nb, 'uint8');  fclose(fid);     % bytes written (count)
        if cnt ~= numel(nb), error('set_run_mode:write', 'wrote %d of %d bytes to %s', cnt, numel(nb), tmpf); end
        [okMove, msg] = movefile(tmpf, dcFile, 'f');      % move succeeded (logical) and the reason if not (char)
        if ~okMove, error('set_run_mode:write', 'could not move %s over %s: %s', tmpf, dcFile, msg); end
    catch err
        if isfile(tmpf), delete(tmpf); end
        rethrow(err);
    end
    if ~isequal(read_bytes(dcFile), nb)
        error('set_run_mode:verify', '%s does not hold the bytes just written (re-read differs)', dcFile);
    end
    what = sprintf('run_mode %d -> %d', oldMode, M.n);   % what happened (char)
end
fprintf('%s\n', M.banner);
fprintf('set_run_mode: %s: %s. define_constants has NOT been re-run.\n', dcFile, what);
end

function b = read_bytes(f)
%READ_BYTES  All bytes of file f as a uint8 row; error set_run_mode:read if it cannot be opened.
fid = fopen(f, 'r');
if fid < 0, error('set_run_mode:read', 'cannot open %s', f); end
b = fread(fid, Inf, '*uint8').';
fclose(fid);
end
