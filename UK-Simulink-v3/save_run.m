function f = save_run(out, snap, tag, dirOut)
%SAVE_RUN  Save one run (logged signals + run snapshot) to a new .mat file; never overwrites.
%   f = save_run(out, snap, tag, dirOut)    focus plan F3 / Plan 3 Task 3 (pure part), 19 Nov 2026.
%   out     the run's logged signals: a Simulink.SimulationOutput or any struct (all of it is saved as given,
%           also a partial one, e.g. a run that ended in a dropout stop). Raw telemetry and per-line send
%           times, when a recorder provides them, travel inside it as out.telemetry / out.send_log (fields in
%           compare_hw_sim_core.m).
%   snap    run snapshot struct (collect_run_snapshot.m builds it from the base workspace). Expected fields:
%             scenario, user (user-block variable values), dc_sha_at_run, repo_commit, matlab_release,
%             EnableHardware, EnableMotive, motive_source, arm_freeze_user;
%           optional: run_stamp ('yyyy-mm-dd_HHMMSS' char or datetime; default: now), q0_9dof, dt_9dof, r, l,
%           alphas (needed by compare_hw_sim_core for the per-line deficit). A missing expected field does NOT
%           stop the save (a run must never be lost): it is set to 'MISSING', listed in snap.missing, and a
%           warning save_run:snapshot is issued.
%   tag     short label, letters, digits, '_' or '-', 1 to 40 characters (char or string).
%   dirOut  output folder (char); default <this folder>/runs_lab. Created if absent.
%   f       full path of the file written (char):  <dirOut>/<yyyy-mm-dd_HHMMSS>_s<scenario>_<tag>.mat, or, when
%           that name exists, the same name with _n2, _n3, ... appended.
%   NO OVERWRITE. A name is claimed by creating the file atomically (java.io.File.createNewFile, which fails
%   when the file exists), so two calls in the same second, also from two MATLAB sessions writing to the same
%   folder, get two names. The claimed empty file is then written by save(); on a save error it is deleted.
%   LIMIT: the claim is atomic only on ONE file system. It does NOT protect against cross-machine sync: two PCs
%   writing the same name into a OneDrive (or other synced) folder each see a free name locally, and the sync
%   client later keeps one, or renames one into a conflict copy, without telling MATLAB. Hence a warning
%   save_run:onedrive whenever the output folder path contains "OneDrive" (the default folder does, on this PC):
%   save lab runs to a local, unsynced folder, or keep one writer per synced folder.
%   Pure apart from the file it writes: reads no workspace, runs no model.
if nargin < 4 || isempty(dirOut), dirOut = fullfile(fileparts(mfilename('fullpath')), 'runs_lab'); end
tag = char(string(tag));                       % run label (char)
assert(~isempty(regexp(tag, '^[A-Za-z0-9_-]{1,40}$', 'once')), 'save_run:tag', ...
       'tag must be 1-40 characters of letters, digits, _ or - (got "%s")', tag);
if ~isstruct(snap), snap = struct('snapshot_given', {snap}); end
need = {'scenario', 'user', 'dc_sha_at_run', 'repo_commit', 'matlab_release', 'EnableHardware', ...
        'EnableMotive', 'motive_source', 'arm_freeze_user'};   % expected snapshot fields (cellstr)
missing = need(~isfield(snap, need));          % absent ones (cellstr)
for k = 1:numel(missing), snap.(missing{k}) = 'MISSING'; end
snap.missing = missing;
if ~isempty(missing)
    warning('save_run:snapshot', 'run snapshot lacks %s; saved as ''MISSING''', strjoin(missing, ', '));
end
if isfield(snap, 'run_stamp') && ~isempty(snap.run_stamp)
    if isdatetime(snap.run_stamp)
        d = snap.run_stamp;  d.Format = 'yyyy-MM-dd_HHmmss';  stamp = char(d);
    else
        stamp = char(string(snap.run_stamp));
    end
else
    d = datetime('now');  d.Format = 'yyyy-MM-dd_HHmmss';  stamp = char(d);
    snap.run_stamp = stamp;
end
assert(~isempty(regexp(stamp, '^\d{4}-\d{2}-\d{2}_\d{6}$', 'once')), 'save_run:stamp', ...
       'run_stamp must read yyyy-mm-dd_HHMMSS (got "%s")', stamp);
if isnumeric(snap.scenario) && isscalar(snap.scenario) && isfinite(snap.scenario)
    sc = sprintf('%d', round(snap.scenario));  % scenario number (char)
else
    sc = 'X';                                   % unknown scenario
end
if contains(dirOut, 'OneDrive', 'IgnoreCase', true)
    warning('save_run:onedrive', ['output folder %s is under OneDrive: no-overwrite holds on this PC only, ' ...
            'not across machines syncing the folder'], dirOut);
end
if ~isfolder(dirOut), mkdir(dirOut); end
base = sprintf('%s_s%s_%s', stamp, sc, tag);   % file name without suffix (char)
f = fullfile(dirOut, [base '.mat']);
n = 1;                                          % candidate number (count)
while ~claim_file(f)
    n = n + 1;
    if n > 999, error('save_run:names', 'more than 999 runs named %s in %s', base, dirOut); end
    f = fullfile(dirOut, sprintf('%s_n%d.mat', base, n));
end
d = datetime('now');  d.Format = 'yyyy-MM-dd HH:mm:ss.SSS';
saved_at = char(d);                             % wall-clock time of the save (char)
saved_by = 'save_run';                          % writer (char)
try
    save(f, 'out', 'snap', 'saved_at', 'saved_by', '-v7.3');
catch err
    if isfile(f), delete(f); end
    rethrow(err);
end
fprintf('save_run: %s\n', f);
end

function ok = claim_file(f)
% True when f did not exist and is now created (empty) by this call; false when it already existed.
ok = logical(java.io.File(f).createNewFile());
end
