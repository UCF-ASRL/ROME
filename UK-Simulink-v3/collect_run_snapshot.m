function snap = collect_run_snapshot(dcFile)
%COLLECT_RUN_SNAPSHOT  Run snapshot for save_run, read from the BASE workspace (focus plan F3, 19 Nov 2026).
%   snap = collect_run_snapshot()          uses <this folder>/define_constants.m for the user-block names
%   snap = collect_run_snapshot(dcFile)
%   NOT TESTED against the live model (F3 core is pure; the StopFcn wiring that calls this comes later in the
%   MATLAB lane). Read-only: changes no variable, runs no model.
%   Fields:
%     user            struct of every user-block variable present in the base workspace. The user block is the
%                     text of define_constants.m above the line "Below here is the formulation"; its names are
%                     the identifiers assigned at the start of a line there (regexp on the file text).
%     user_names      those names (cellstr), present or not
%     run_mode, scenario, EnableHardware, EnableMotive, arm_freeze_user, arm_freeze  base values (NaN when absent)
%     motive_source   base value, or 'absent' (the variable does not exist before focus plan F7)
%     dc_sha_at_run   base value set by define_constants (SHA-256 hex of the file as run), or 'absent'
%     dc_sha_file_now dc_file_sha of dcFile now (a mismatch means the file changed after it was run)
%     repo_commit     `git rev-parse HEAD` of the repository holding this folder, or 'unknown'
%     repo_dirty      true when `git status --porcelain` lists changes, NaN when git failed
%     matlab_release  version() (char)
%     q0_9dof, dt_9dof, r, l, alphas, arm_sign, arm_offset_deg   base values used by compare_hw_sim_core (or NaN)
%     run_stamp       now, 'yyyy-mm-dd_HHMMSS'
here = fileparts(mfilename('fullpath'));       % ROME_merged (char)
if nargin < 1, dcFile = fullfile(here, 'define_constants.m'); end
txt = fileread(dcFile);                         % define_constants text (char)
iEnd = strfind(txt, 'Below here is the formulation');
names = {};                                     % user-block variable names (cellstr)
if ~isempty(iEnd)
    tok = regexp(txt(1:iEnd(1)), '^\s*([A-Za-z]\w*)\s*=[^=]', 'tokens', 'lineanchors');
    names = unique(cellfun(@(c) c{1}, tok, 'UniformOutput', false), 'stable');
end
user = struct();
for k = 1:numel(names)
    if evalin('base', sprintf('exist(''%s'', ''var'')', names{k})) == 1
        user.(names{k}) = evalin('base', names{k});
    end
end
snap = struct();
snap.user = user;
snap.user_names = names;
for n = {'run_mode', 'scenario', 'EnableHardware', 'EnableMotive', 'arm_freeze_user', 'arm_freeze', 'q0_9dof', 'dt_9dof', ...
         'r', 'l', 'alphas', 'arm_sign', 'arm_offset_deg'}
    snap.(n{1}) = base_or(n{1}, NaN);
end
snap.motive_source = base_or('motive_source', 'absent');
snap.dc_sha_at_run = base_or('dc_sha_at_run', 'absent');
snap.dc_sha_file_now = dc_file_sha(dcFile);
[st, o] = system(sprintf('git -C "%s" rev-parse HEAD', here));
if st == 0, snap.repo_commit = strtrim(o); else, snap.repo_commit = 'unknown'; end
[st, o] = system(sprintf('git -C "%s" status --porcelain', here));
if st == 0, snap.repo_dirty = ~isempty(strtrim(o)); else, snap.repo_dirty = NaN; end
snap.matlab_release = version;
d = datetime('now');  d.Format = 'yyyy-MM-dd_HHmmss';
snap.run_stamp = char(d);
end

function v = base_or(n, d)
% Base-workspace variable n, or default d when it does not exist.
if evalin('base', sprintf('exist(''%s'', ''var'')', n)) == 1, v = evalin('base', n); else, v = d; end
end
