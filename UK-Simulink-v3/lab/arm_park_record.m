function rec = arm_park_record(mode, varargin)
%ARM_PARK_RECORD  Record of where calibrateROMEArm parked the arm (focus plan F11).
%   rec = arm_park_record('write', fw_deg, file, dc_sha)   append one entry, return it
%   rec = arm_park_record('read', file)                     newest entry, or [] when the file does not exist
%     fw_deg  1x6 (or 6x1) parked joint target, firmware degrees (calibrateROMEArm's homePosDeg: the HOME / park
%             command it sent, calibrateROMEArm.m:60-61)
%     file    path of the record, a .mat file (char)
%     dc_sha  SHA-256 of define_constants.m as calibrateROMEArm ran it (dc_sha_at_run, hex char, 64 characters)
%     rec     struct  fw_deg (1x6, deg), dc_sha (char), time (char, local time 'yyyy-mm-ddTHH:MM:SS'), n (entry number),
%             history_count (entries in the file)
%   HISTORY, NEVER A SILENT OVERWRITE: the file holds the variable park_history, a struct array; 'write' loads it,
%   appends the new entry and saves it back, so every park ever recorded stays in the file and the newest is the one
%   'read' returns. A file that exists but does not hold a valid park_history is never replaced: 'write' and 'read'
%   error (arm_park_record:foreign). Delete or move such a file by hand.
%   WRITE SAFETY: the new history is saved to a temp file in the same folder and then moved over the record, so an
%   interrupted save leaves the old history intact. SINGLE WRITER assumed: two writers at the same moment can each
%   load the same history and the later move drops the other's entry. Only calibrateROMEArm writes, one session at
%   a time; there is no file lock.
%   Used by prerun_check_core.m (S.park_record): a frozen hardware run is refused when the newest record is missing,
%   was made with another define_constants SHA, or differs from the frozen command by more than park_tol_deg
%   (allowed_ranges.m). The record is not proof of the physical posture: at the end of fullCalibrate the firmware
%   reports homePosDeg whatever the encoders say (calibrateROMEArm.m:126-131), so the posture is confirmed by eye.
%   Reads and writes only the file given. Errors on bad input (arm_park_record:input).
switch mode
    case 'write'
        if numel(varargin) ~= 3
            error('arm_park_record:input', 'usage: arm_park_record(''write'', fw_deg, file, dc_sha)');
        end
        [fw, file, sha] = varargin{:};
        if ~(isnumeric(fw) && isreal(fw) && numel(fw) == 6 && all(isfinite(fw(:))))
            error('arm_park_record:input', 'fw_deg must be 6 finite firmware degrees, got %s', mat2str(fw));
        end
        if isstring(sha) && isscalar(sha), sha = char(sha); end
        if ~(ischar(sha) && ~isempty(regexp(sha, '^[0-9a-f]{64}$', 'once')))
            error('arm_park_record:input', 'dc_sha must be a 64-character lower-case hex SHA-256 (dc_file_sha), got %s', mat2str(sha));
        end
        file = check_file(file);
        hist = load_history(file);                  % struct array of earlier entries (may be empty)
        e = struct('fw_deg', double(fw(:).'), 'dc_sha', sha, ...
                   'time', char(datetime('now', 'Format', 'yyyy-MM-dd''T''HH:mm:ss')), 'n', numel(hist) + 1);
        if isempty(hist), park_history = e; else, park_history = [hist, e]; end
        d = fileparts(file);  if isempty(d), d = pwd; end   % folder of the record (char)
        tmpf = [tempname(d) '.mat'];                 % temp file in the same folder (char)
        save(tmpf, 'park_history');
        movefile(tmpf, file, 'f');                   % replace in one step: an interrupted save never truncates the history
        rec = e;  rec.history_count = e.n;
    case 'read'
        if numel(varargin) ~= 1
            error('arm_park_record:input', 'usage: rec = arm_park_record(''read'', file)');
        end
        file = check_file(varargin{1});
        if ~isfile(file), rec = []; return; end
        hist = load_history(file);
        if isempty(hist), rec = []; return; end
        rec = hist(end);  rec.history_count = numel(hist);
    otherwise
        error('arm_park_record:input', 'mode must be ''write'' or ''read'', got %s', mat2str(mode));
end
end

function file = check_file(file)
%CHECK_FILE  file must be a non-empty char / string path ending in .mat.
if isstring(file) && isscalar(file), file = char(file); end
if ~(ischar(file) && ~isempty(file) && endsWith(lower(file), '.mat'))
    error('arm_park_record:input', 'file must be a .mat path, got %s', mat2str(file));
end
end

function hist = load_history(file)
%LOAD_HISTORY  park_history of an existing file ([] when the file does not exist); errors on a foreign file.
hist = [];
if ~isfile(file), return; end
try
    L = load(file);
catch err
    error('arm_park_record:foreign', '%s exists but cannot be read as a .mat (%s); not overwritten', file, err.message);
end
need = {'fw_deg', 'dc_sha', 'time', 'n'};
if ~(isfield(L, 'park_history') && isstruct(L.park_history) && all(isfield(L.park_history, need)))
    error('arm_park_record:foreign', '%s exists but holds no valid park_history (fields %s); not overwritten', file, strjoin(need, ', '));
end
hist = L.park_history(:).';
end
