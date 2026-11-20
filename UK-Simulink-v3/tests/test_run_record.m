function varargout = test_run_record(brk)
%TEST_RUN_RECORD  F3 wiring (20 Nov 2026): the mode-1 student path records its stop, and the controller's input is logged.
%   Runs on a TEMPORARY COPY of ROME_merged (tests/stubs ROMECommand and MotiveData, no network); ROME_merged is never
%   written. Checks (all evaluated, then one error listing the failed ones):
%     (a) the real ROME_9DOF.slx has 'To Workspace q_ctrl_in' (VariableName q_ctrl_in) fed by StateSource output 1
%     (b) copy, mode 1: plot_9dof() (which runs the model, as a student does), then save_run() with no arguments: no
%         warning save_run:latesnap, the base holds rome_last_run (the StopFcn record, by_code true, robot false), the
%         saved file's snapshot is rome_last_run.snap (same run_stamp: the snapshot of the stop) and its tag is mode1
%     (c) the same run's out.q_ctrl_in is N x 9 on q_meas's time base, and its joints (4:9) equal q_meas's joints bit
%         for bit (mode 1: JointSource takes SimJoints = SensorModel 4:9), every element finite
%   MAKE-IT-FAIL: test_run_record('<id>') applies one break to the COPY; the named check must FAIL:
%     mf_stopfcn   plot_9dof clears the StopFcn again (the line F3 removed)  -> (b)
%   test_run_record('list') returns the table. Flat function without local functions (CLAUDE.md 5). Clears the base.
B = {'mf_stopfcn', 'b', {'plot_9dof.m', '    % StopFcn kept (F3 wiring): rome_run_end records the stop (rome_last_run), so save_run() saves this run''s snapshot', ...
                         '    set_param(mdl,''StopFcn'','''');'}, 'plot_9dof clears the StopFcn'};
if nargin >= 1 && strcmp(brk, 'list'), varargout{1} = B; return; end
ib = [];
if nargin >= 1 && ~isempty(brk)
    ib = find(strcmp(B(:,1), brk));
    assert(numel(ib) == 1, 'test_run_record:brk', 'unknown break %s', brk);
    fprintf('MAKE-IT-FAIL break %s (check (%s) must FAIL): %s\n', brk, B{ib,2}, B{ib,4});
end
here = fileparts(mfilename('fullpath'));  root = fileparts(here);  mdl = 'ROME_9DOF';
fails = {};
% ---- (a) the real model -------------------------------------------------------------------------------------------
if bdIsLoaded(mdl), close_system(mdl, 0); end
evalc('load_system(fullfile(root, [mdl ''.slx'']))');
BP = [mdl '/To Workspace q_ctrl_in'];  src = '<none>';  vn = '';
if getSimulinkBlockHandle(BP) ~= -1
    pc = get_param(BP, 'PortConnectivity');
    if pc(1).SrcBlock ~= -1, src = sprintf('%s/%d', get_param(pc(1).SrcBlock, 'Name'), pc(1).SrcPort + 1); end
    vn = get_param(BP, 'VariableName');
end
close_system(mdl, 0);
okA = strcmp(src, 'StateSource/1') && strcmp(vn, 'q_ctrl_in');
fprintf('(a) To Workspace q_ctrl_in fed by %s, variable ''%s'' -> %s\n', src, vn, mat2str(okA));
if ~okA, fails{end+1} = '(a) q_ctrl_in block'; end
% ---- copy ------------------------------------------------------------------------------------------------------------
tmp = tempname;  startdir = pwd;  path0 = path;
mkdir(tmp);  copyfile(root, tmp);
copyfile(fullfile(here, 'stubs', 'ROMECommand.m'), fullfile(tmp, 'ROMECommand.m'), 'f');
copyfile(fullfile(here, 'stubs', 'MotiveData.m'),  fullfile(tmp, 'Optitrack', 'MotiveData.m'), 'f');
if isfolder(fullfile(tmp, 'runs_lab')), rmdir(fullfile(tmp, 'runs_lab'), 's'); end
mdlcell = {mdl};
cleanup = onCleanup(@() cellfun(@(c) c(), {@() cd(startdir), @() path(path0), @() close('all'), ...
          @() cellfun(@(m) close_system(m, 0), mdlcell(bdIsLoaded(mdl))), @() rmdir(tmp, 's')})); %#ok<NASGU>
if ~isempty(ib)
    ed = B{ib,3};  f = fullfile(tmp, ed{1});
    fid = fopen(f, 'r');  s = char(fread(fid, Inf, '*uint8').');  fclose(fid);
    n = numel(strfind(s, ed{2}));
    assert(n == 1, 'test_run_record:brk', 'break edit matches %d times (expected 1)', n);
    fid = fopen(f, 'w');  fwrite(fid, uint8(strrep(s, ed{2}, ed{3})));  fclose(fid);
end
addpath(tmp, fullfile(tmp, 'Ground Vehicle'), fullfile(tmp, 'Optitrack'));
cd(tmp);  rehash;
wh = cellfun(@which, {'plot_9dof', 'save_run', 'rome_run_end', 'define_constants'}, 'UniformOutput', false);
assert(all(startsWith(wh, tmp)), 'test_run_record:path', 'not resolved inside the copy: %s', strjoin(wh(~startsWith(wh, tmp)), ' | '));
evalin('base', 'clear all');
% ---- (b) the mode-1 student path -----------------------------------------------------------------------------------
set(0, 'DefaultFigureVisible', 'off');
evalc('plot_9dof');
close('all');
lastwarn('');
txt = evalc('f = save_run();');
[wmsg, wid] = lastwarn;
R = [];  if evalin('base', 'exist(''rome_last_run'', ''var'')') == 1, R = evalin('base', 'rome_last_run'); end
okR = isstruct(R) && isfield(R, 'by_code') && R.by_code && ~R.robot;
okS = false;  tagOk = false;
if isfile(f)
    Sv = load(f, 'snap');
    okS = isstruct(R) && isfield(Sv.snap, 'run_stamp') && strcmp(Sv.snap.run_stamp, R.snap.run_stamp);   % the stop's stamp (save_run adds snap.missing, so the structs differ by that field)
    [~, nmF] = fileparts(f);  tagOk = endsWith(nmF, '_mode1');
end
okB = ~strcmp(wid, 'save_run:latesnap') && okR && okS && tagOk;
fprintf('(b) save_run() after plot_9dof: %s; warning ''%s'' %s; stop record %d; saved snap = stop snap %d; tag mode1 %d -> %s\n', ...
        strtrim(txt), wid, wmsg(1:min(end, 120)), okR, okS, tagOk, mat2str(okB));
if ~okB, fails{end+1} = '(b) mode-1 save_run uses the stop record'; end
% ---- (c) the controller's input is logged ---------------------------------------------------------------------------
o = evalin('base', 'out');
okC = false;  dtxt = 'q_ctrl_in not in out';
try
    qc = o.q_ctrl_in;  qm = o.q_meas;
    C = squeeze(qc.Data);  M = squeeze(qm.Data);
    if size(C, 1) == 9, C = C.'; end
    if size(M, 1) == 9, M = M.'; end
    okC = isequal(qc.Time, qm.Time) && size(C, 2) == 9 && isequal(C(:, 4:9), M(:, 4:9)) && all(isfinite(C(:)));
    dtxt = sprintf('%d x %d, same time base %d, joints equal q_meas %d (max diff %g), finite %d', size(C, 1), size(C, 2), ...
                   isequal(qc.Time, qm.Time), isequal(C(:, 4:9), M(:, 4:9)), max(abs(C(:, 4:9) - M(:, 4:9)), [], 'all'), all(isfinite(C(:))));
catch e
    dtxt = e.message;
end
fprintf('(c) out.q_ctrl_in: %s -> %s\n', dtxt, mat2str(okC));
if ~okC, fails{end+1} = '(c) q_ctrl_in logged'; end
evalin('base', 'clear all');
if ~isempty(fails)
    error('test_run_record:fail', 'test_run_record FAILED: %s', strjoin(fails, '; '));
end
fprintf('test_run_record PASS\n');
end
