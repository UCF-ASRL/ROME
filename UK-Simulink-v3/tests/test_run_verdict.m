function varargout = test_run_verdict(brk)
%TEST_RUN_VERDICT  The PASS/FAIL line at the end of a robot run and save_run() with no arguments, end to end on the
%   Run-button path against the recording fake robot (focus plan F14a, 19 Nov 2026).
%   NO NETWORK. A TEMPORARY COPY of ROME_merged in which ROMECommand.m is GENERATED from Keanu's file with ONLY
%   tcpclient(...) replaced by RobotLinkEcho(...) (one match asserted; tests/stubs_hw: records every line with its wall
%   stamp and answers each ROME line with ARM telemetry echoing its joints), and the MotiveData block is pointed at
%   MotiveDataCounting (no socket) in memory. The base variable rome_send_recorder = @() read_link_record(<record file>)
%   is the send recorder the verdict asks for STOP_ALL and the watchdog gap.
%   Every run is started the way the Run button starts it: set_param(mdl, 'SimulationCommand', 'start') (asynchronous;
%   Simulink then assigns the base variable out after the StopFcn, which a sim() call does not do), then the test waits
%   for SimulationStatus 'stopped' and for the verdict (base rome_last_verdict, written by rome_run_finish.m or
%   run_refused.m). The run length is shortened in the base after load: T_stop = 4 s (the model's StopTime is the
%   variable T_stop, and the verdict reads T_stop at the stop). A previous run's out is put back in the base before each
%   run, so the verdict must not take the old output for the new one.
%   Runs (scenario 1):
%     m1          mode 1 (simulation): no verdict, no timer; then save_run() (see (s)).
%     P2, P3      modes 2 and 3, no fault.
%     A2, A3      tstop    an Assertion block (in memory) stops the run with an error at t = 2 s.
%     B2, B3      refusal  lab_settings.m edited after the load: the InitFcn refuses (arm_freeze_banner:stale).
%     C2, C3      watchdog the fake link holds the 40th ROME line 0.4 s (robot_link_stall.txt): a gap > 0.25 s.
%     D2, D3      stopall  the copy's ROMECommand.m without its STOP_ALL write.
%     E2, E3      saved    a FILE named runs_lab where save_run's folder should be: the save fails.
%     F2, F3      wheels   CommandGuard's limit multiplied by 1e6 (chart, in memory) and max_rpm = 0.001 in the copy's
%                          lab_settings.m: wheel commands above max_rpm reach the robot.
%   Checks (all evaluated, then one error test_run_verdict:fail listing the failed ones):
%     (p)  P2 and P3: verdict PASS, the six judged items PASS; P3's arm item UNSOURCED with a measured number; the run
%          file exists in the copy's runs_lab, named *_s1_mode<n>.mat, and holds out and the snapshot of the stop
%          (snap.run_mode = n, snap.lab.max_rpm = 80); the model's StopFcn is rome_run_end(bdroot).
%     (f)  each fault run: verdict FAIL and its item(s) FAIL, every other judged item PASS (B: the refusal
%          verdict of run_refused.m, refusal FAIL naming arm_freeze_banner:stale; nothing ran, so tstop and saved FAIL
%          and the rest NOT MEASURED). C (watchdog fault): exactly the watchdog item FAILs, every other item PASS.
%          A, D, E, F (a non-watchdog fault): the injected item FAILs and the NON-TIMING items (tstop, refusal, stopall,
%          saved, wheels, minus the injected one) PASS; the watchdog item is printed but not required to PASS.
%          RULING (controller, 20 Nov 2026, verbatim): "test_run_verdict (f) cases with a non-watchdog injected fault
%          must assert their injected item FAILs and the non-timing items PASS; the watchdog item is reported, not
%          required PASS (watchdog has its own C2/C3 cases) -- the wall-clock item depends on the PC -- cost if wrong:
%          (f) no longer catches a fault that wrongly trips watchdog; C2/C3 still cover it."
%          Evidence: docs/verification/steps_final_3cc11ea/step24.txt:583 and :585, D2 (stopall) and F2 (wheels) each got
%          an extra watchdog FAIL from a 0.327 s real wall-clock gap on this PC, with no watchdog fault injected.
%          (p) is unchanged: a fault-free run still needs watchdog PASS (it can still be tripped by a slow PC).
%     (s)  save_run(): after P2 it returns P2's file without writing another (already saved); after m1 (no automatic
%          save) it writes *_s1_mode1.mat whose snapshot is the one of m1's stop (run_mode 1) although the copy was
%          switched to mode 2 before the call; a second call returns the same file; with no out in the base it errors
%          save_run:noout.
%     (q)  a mode-2 run started by sim() from code (as the other tests start theirs) leaves no verdict timer, prints no
%          verdict within 3 s, and its stop record says by_code = true.
%     (o)  a verdict timer whose stop record names the output already in the base (start stamp = prev_out_start, its
%          own stop stamp as the stop time) ends without a verdict: the previous output is never taken for a new run.
%     (t)  no rome_run_finish timer is left at the end.
%   MAKE-IT-FAIL: test_run_verdict('<id>') applies one break to the copy and must FAIL at the named check;
%   test_run_verdict('list') returns the table; tests/manual_makefail_f14a.m runs them.
%   Flat function without local functions (CLAUDE.md section 5, crash 2). Clears the base workspace.
if nargin < 1 || isempty(brk), brk = ''; end
here = fileparts(mfilename('fullpath'));  root = fileparts(here);
mdl = 'ROME_9DOF';  lf = char(10);
Tshort = 4;                                        % shortened run length (s)
B = {
  'mf_nostopfcn', 'p f s', {'param', 'StopFcn', ''}, 'the model has no StopFcn: no verdict, no automatic save';
  'mf_nosave',    'p s',   {'file', 'rome_run_finish.m', '    F.saved_file = save_run(o, R.snap, tag, R.dirOut);', '    F.saved_file = '''';'}, 'the run is never saved';
  'mf_snapnow',   's',     {'file', 'save_run.m', 'match = strcmp(md.ModelInfo.ModelName, R.mdl) && (sameStart || (dts >= -2 && dts <= 15));', 'match = false;'}, 'save_run() ignores the stop record (snapshot of now, no already-saved)';
  'mf_bycode',    'q',     {'file', 'rome_run_end.m', 'R.by_code = st(2).line <= numel(Lc) && contains(Lc{st(2).line}, ''sim('');', 'R.by_code = false;'}, 'runs started by sim() from code start a verdict timer';
  'mf_oldout',    'o',     {'file', 'rome_run_finish.m', 'if ~isa(o, ''Simulink.SimulationOutput''), return; end', 'ok = isa(o, ''Simulink.SimulationOutput''); return'}, 'the verdict takes the previous run''s output';
  'mf_extrafail', 'f',     {'file', 'run_verdict_core.m', 'pf(isfinite(mr) && pk <= mr)', 'pf(isfinite(mr) && pk <= mr && ~(haveLog && ~okStop))'}, 'an injected stopall fault also fails the wheels item (a non-timing item)';
};
if strcmp(brk, 'list'), varargout{1} = B; return; end
ib = [];
if ~isempty(brk)
    ib = find(strcmp(B(:,1), brk));
    assert(numel(ib) == 1, 'test_run_verdict:brk', 'unknown break %s', brk);
    fprintf('MAKE-IT-FAIL break %s (check (%s) must FAIL): %s\n', brk, B{ib,2}, B{ib,4});
end

% ---- copy, stub, path ------------------------------------------------------------------------------------------------
tmp = tempname;  startdir = pwd;  path0 = path;
if bdIsLoaded(mdl), close_system(mdl, 0); end
mkdir(tmp);  copyfile(root, tmp);
prerun_fixture('signoff', tmp);              % F2 wiring: Part F/C sign-off dates in the COPY's lab_settings.m (prerun_check refuses robot runs without)
mdlcell = {mdl};
cleanup = onCleanup(@() cellfun(@(c) c(), {@() delete(timerfind('Name', 'rome_run_finish')), @() cd(startdir), @() path(path0), ...
          @() cellfun(@(m) close_system(m, 0), mdlcell(bdIsLoaded(mdl))), @() evalin('base', 'clear all'), @() rmdir(tmp, 's')})); %#ok<NASGU>
fid = fopen(fullfile(root, 'ROMECommand.m'), 'r');  realtxt = char(fread(fid, Inf, '*uint8').');  fclose(fid);
old = 'tcpclient(obj.ESP32_IP, obj.ESP32_Port, "ConnectTimeout", 5)';
assert(numel(strfind(realtxt, old)) == 1, 'test_run_verdict:stub', 'tcpclient constructor found %d times', numel(strfind(realtxt, old)));
stub = strrep(realtxt, old, 'RobotLinkEcho(obj.ESP32_IP, obj.ESP32_Port, "ConnectTimeout", 5)');   % the fake-robot ROMECommand (char)
STOPW = 'write(obj.DeviceHandle, uint8(char("STOP_ALL" + newline)));';
assert(numel(strfind(stub, STOPW)) == 1, 'test_run_verdict:stub', 'STOP_ALL write found %d times', numel(strfind(stub, STOPW)));
stubNoStop = strrep(stub, STOPW, '% STOP_ALL write removed (test_run_verdict fault D)');
for f = {'RobotLinkRecorder.m', 'RobotLinkEcho.m', 'MotiveDataCounting.m'}
    copyfile(fullfile(here, 'stubs_hw', f{1}), fullfile(tmp, f{1}), 'f');
end
if ~isempty(ib) && strcmp(B{ib,3}{1}, 'file')
    ed = B{ib,3};  f = fullfile(tmp, ed{2});
    s = fileread(f);  n = numel(strfind(s, ed{3}));
    assert(n == 1, 'test_run_verdict:brk', 'break edit in %s matches %d times (expected 1)', ed{2}, n);
    fid = fopen(f, 'w');  fwrite(fid, uint8(strrep(s, ed{3}, ed{4})));  fclose(fid);
    fprintf('  break edit applied to the copy: %s\n', ed{2});
end
addpath(tmp, fullfile(tmp, 'Ground Vehicle'), fullfile(tmp, 'Optitrack'));
addpath(here, '-end');                             % tests: dc_set_mode, read_link_record
cd(tmp);  rehash;
dcf = fullfile(tmp, 'define_constants.m');  labf = fullfile(tmp, 'lab_settings.m');  slx = fullfile(tmp, [mdl '.slx']);
cmdf = fullfile(tmp, 'ROMECommand.m');  recf = fullfile(tmp, 'robot_link_record.txt');  stallf = fullfile(tmp, 'robot_link_stall.txt');
runsd = fullfile(tmp, 'runs_lab');
dcBase = fileread(dcf);  labBase = fileread(labf);
assert(numel(strfind(labBase, [lf 'max_rpm = 80;'])) == 1, 'test_run_verdict:lab', 'max_rpm line found %d times', numel(strfind(labBase, [lf 'max_rpm = 80;'])));
for c = {'rome_run_end', 'rome_run_finish', 'run_verdict_core', 'save_run', 'define_constants', 'RobotLinkEcho'}
    assert(strcmp(fileparts(which(c{1})), tmp), 'test_run_verdict:path', '%s resolves to %s', c{1}, which(c{1}));
end
blkM = [mdl '/Enabled' newline 'Subsystem/MATLAB System'];   % the MotiveData block

RUNS = {'m1', 1, ''; 'P2', 2, ''; 'A2', 2, 'tstop'; 'B2', 2, 'refusal'; 'C2', 2, 'watchdog'; 'D2', 2, 'stopall'; 'E2', 2, 'saved'; 'F2', 2, 'wheels'; ...
        'P3', 3, ''; 'A3', 3, 'tstop'; 'B3', 3, 'refusal'; 'C3', 3, 'watchdog'; 'D3', 3, 'stopall'; 'E3', 3, 'saved'; 'F3', 3, 'wheels'};
Res = struct();
prevOut = [];                                      % the previous run's output, put back before each run
for r = 1:size(RUNS, 1)
    [nm, n, flt] = RUNS{r,:};
    fprintf('\n---- run %s: mode %d, fault %s ----\n', nm, n, flt);
    % files of the copy for this run
    lab = labBase;
    if strcmp(flt, 'wheels'), lab = strrep(lab, [lf 'max_rpm = 80;'], [lf 'max_rpm = 0.001;']); end
    fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc_set_mode(dcBase, n)));  fclose(fid);
    fid = fopen(labf, 'w');  fwrite(fid, uint8(lab));  fclose(fid);
    sb = stub;  if strcmp(flt, 'stopall'), sb = stubNoStop; end
    fid = fopen(cmdf, 'w');  fwrite(fid, uint8(sb));  fclose(fid);
    if strcmp(flt, 'watchdog'), fid = fopen(stallf, 'w');  fprintf(fid, '40 0.4\n');  fclose(fid); elseif isfile(stallf), delete(stallf); end
    if isfolder(runsd) && strcmp(flt, 'saved'), movefile(runsd, [runsd '_kept']); end
    if strcmp(flt, 'saved'), fid = fopen(runsd, 'w');  fprintf(fid, 'not a folder\n');  fclose(fid); end
    if isfile(recf), delete(recf); end
    % fresh base, model loaded (PostLoadFcn = define_constants of the copy)
    if evalin('base', 'exist(''out'', ''var'')') == 1, prevOut = evalin('base', 'out'); end
    evalin('base', 'clear all');  evalin('base', 'clear classes');
    clear define_constants lab_settings ROMECommand RobotLinkEcho RobotLinkRecorder;  rehash;
    evalc('load_system(slx)');
    set_param(mdl, 'SimulationMode', 'normal');
    sim0 = get_param(blkM, 'SimulateUsing');
    set_param(blkM, 'System', 'MotiveDataCounting');  set_param(blkM, 'SimulateUsing', sim0);
    if ~isempty(ib) && strcmp(B{ib,3}{1}, 'param'), set_param(mdl, B{ib,3}{2}, B{ib,3}{3}); end
    stopFcn = get_param(mdl, 'StopFcn');
    if strcmp(flt, 'tstop')
        load_system('simulink');
        set_param(mdl, 'AssertControl', 'EnableAll');
        add_block('simulink/Logic and Bit Operations/Compare To Constant', [mdl '/VerdictErrAt'], 'relop', '<', 'const', '2');
        add_block('simulink/Model Verification/Assertion', [mdl '/VerdictInjectedError']);
        add_line(mdl, 'Clock9/1', 'VerdictErrAt/1');  add_line(mdl, 'VerdictErrAt/1', 'VerdictInjectedError/1');
    end
    if strcmp(flt, 'wheels')
        ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/CommandGuard']);
        assert(numel(ch) == 1 && numel(strfind(ch.Script, 'lim  = abs(max_rpm);')) == 1, 'test_run_verdict:chart', 'CommandGuard limit line not found once');
        ch.Script = strrep(ch.Script, 'lim  = abs(max_rpm);', 'lim  = 1e6*abs(max_rpm);');
    end
    assignin('base', 'T_stop', Tshort);
    assignin('base', 'rome_send_recorder', @() read_link_record(recf));
    if ~isempty(prevOut), assignin('base', 'out', prevOut); end
    if strcmp(flt, 'refusal')
        fid = fopen(labf, 'a');  fprintf(fid, '%% test_run_verdict: edited after define_constants ran\n');  fclose(fid);
    end
    prerun_fixture('park', tmp);             % F2/F11 wiring: park record of the copy's settings, as calibrateROMEArm writes it
    filesBefore = dir(fullfile(runsd, '*.mat'));
    % Run, the way the Run button does it
    t0 = tic;
    try, set_param(mdl, 'SimulationCommand', 'start'); catch ex, fprintf('[%s] start: %s\n', nm, ex.message); end
    pause(0.5);
    while ~strcmp(get_param(mdl, 'SimulationStatus'), 'stopped') && toc(t0) < 180, pause(0.1); end
    tStop = toc(t0);
    V = [];
    if n >= 2
        t1 = tic;
        while toc(t1) < 20 && evalin('base', 'exist(''rome_last_verdict'', ''var'')') ~= 1, pause(0.2); end
        if evalin('base', 'exist(''rome_last_verdict'', ''var'')') == 1, V = evalin('base', 'rome_last_verdict'); end
    else
        pause(1);
    end
    R = [];  if evalin('base', 'exist(''rome_last_run'', ''var'')') == 1, R = evalin('base', 'rome_last_run'); end
    filesAfter = dir(fullfile(runsd, '*.mat'));
    S = struct('name', nm, 'mode', n, 'fault', flt, 'V', V, 'R', R, 'stopFcn', stopFcn, 'wall', tStop, ...
               'newFiles', {setdiff({filesAfter.name}, {filesBefore.name})}, 'haveOut', evalin('base', 'exist(''out'', ''var'')') == 1, 'outStart', '');
    try, S.outStart = evalin('base', 'out.SimulationMetadata.TimingInfo.WallClockTimestampStart'); catch, end   % this run's output (s stamp)
    if ~isempty(V)
        fprintf('[%s] %s\n', nm, strrep(V.text, newline, [newline '[' nm '] ']));
    else
        fprintf('[%s] no verdict in the base after %.1f s\n', nm, tStop);
    end
    fprintf('[%s] stopped after %.1f s wall; new run files: %s\n', nm, tStop, strjoin(S.newFiles, ', '));
    if strcmp(nm, 'P2') && ~isempty(R)
        % (s) save_run() after an automatically saved run
        nB = numel(dir(fullfile(runsd, '*.mat')));
        txtS = '';  fS = '';  errS = '';
        try, txtS = evalc('fS = save_run();'); catch ex, errS = ex.identifier; end
        S.saveAgain = struct('f', fS, 'txt', txtS, 'err', errS, 'nBefore', nB, 'nAfter', numel(dir(fullfile(runsd, '*.mat'))));
        fprintf('[P2] save_run(): %s', txtS);
    end
    if strcmp(nm, 'm1')
        % (s) save_run() for a run with no automatic save, after the copy was switched to mode 2 and re-run
        fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc_set_mode(dcBase, 2)));  fclose(fid);
        clear define_constants;  rehash;
        evalc('evalin(''base'', ''define_constants'')');
        rmB = evalin('base', 'run_mode');
        fS = '';  errS = '';  txtS = '';  fS2 = '';  txtS2 = '';
        try, txtS = evalc('fS = save_run();'); catch ex, errS = [ex.identifier ': ' ex.message]; end
        try, txtS2 = evalc('fS2 = save_run();'); catch ex, errS = [errS ' | ' ex.identifier]; end
        L = [];  if ~isempty(fS) && isfile(fS), L = load(fS); end
        o1 = evalin('base', 'out');  evalin('base', 'clear out');
        idNo = '';  try, evalc('save_run()'); catch ex, idNo = ex.identifier; end
        assignin('base', 'out', o1);
        S.save1 = struct('f', fS, 'f2', fS2, 'txt', [txtS txtS2], 'err', errS, 'L', L, 'idNo', idNo, 'rmBase', rmB);
        fprintf('[m1] base run_mode %d before save_run(); save_run(): %s second: %s no out: %s\n', rmB, strtrim(txtS), strtrim(txtS2), idNo);
    end
    Res.(nm) = S;
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    if strcmp(flt, 'saved')
        delete(runsd);
        if isfolder([runsd '_kept']), movefile([runsd '_kept'], runsd); end
    end
end
% (o) the previous run's output is never taken for a new run: a verdict timer whose stop record says the output in the
% base is the one that was there at the stop (prev_out_start = its start stamp; stop_wall = its own stop stamp, so the
% time window alone would accept it) must end without a verdict.
okO = false;  oTxt = 'no base out after the last run';
if evalin('base', 'exist(''out'', ''var'')') == 1 && evalin('base', 'exist(''rome_last_run'', ''var'')') == 1
    oo = evalin('base', 'out');  R2 = evalin('base', 'rome_last_run');
    R2.prev_out_start = oo.SimulationMetadata.TimingInfo.WallClockTimestampStart;
    R2.stop_wall = oo.SimulationMetadata.TimingInfo.WallClockTimestampStop;
    evalin('base', 'clear rome_last_verdict');
    tO = timer('Name', 'rome_run_finish', 'ExecutionMode', 'fixedSpacing', 'Period', 0.2, 'TasksToExecute', 10, ...
               'BusyMode', 'drop', 'UserData', R2, 'TimerFcn', @rome_run_finish, 'StopFcn', @(tt, ~) delete(tt));
    start(tO);
    t1 = tic;  while toc(t1) < 4 && ~isempty(timerfind('Name', 'rome_run_finish')), pause(0.2); end
    gotV = evalin('base', 'exist(''rome_last_verdict'', ''var'')') == 1;
    okO = ~gotV && isempty(timerfind('Name', 'rome_run_finish'));
    oTxt = sprintf('verdict produced from the old output %d, timer ended %d', gotV, isempty(timerfind('Name', 'rome_run_finish')));
end
Res.o = struct('ok', okO, 'txt', oTxt);
% (q) a robot run started by sim() from code (as the other tests start theirs) starts no verdict timer and prints no
% verdict: its output goes to the caller, and the timer would outlive the caller's temporary copy.
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc_set_mode(dcBase, 2)));  fclose(fid);
fid = fopen(labf, 'w');  fwrite(fid, uint8(labBase));  fclose(fid);
fid = fopen(cmdf, 'w');  fwrite(fid, uint8(stub));  fclose(fid);
if isfile(stallf), delete(stallf); end
evalin('base', 'clear all');  evalin('base', 'clear classes');  clear define_constants lab_settings ROMECommand;  rehash;
evalc('load_system(slx)');
set_param(mdl, 'SimulationMode', 'normal');
sim0 = get_param(blkM, 'SimulateUsing');  set_param(blkM, 'System', 'MotiveDataCounting');  set_param(blkM, 'SimulateUsing', sim0);
if ~isempty(ib) && strcmp(B{ib,3}{1}, 'param'), set_param(mdl, B{ib,3}{2}, B{ib,3}{3}); end
assignin('base', 'T_stop', Tshort);
assignin('base', 'rome_send_recorder', @() read_link_record(recf));
prerun_fixture('park', tmp);             % F2/F11 wiring: park record of the copy's settings, as calibrateROMEArm writes it
errQ = '';
try, evalc('oQ = sim(mdl, ''ReturnWorkspaceOutputs'', ''on'');'); catch ex, errQ = ex.message; end
nTq = numel(timerfind('Name', 'rome_run_finish'));     % timers right after sim() returned (count)
t1 = tic;  while toc(t1) < 3, pause(0.2); end
gotVq = evalin('base', 'exist(''rome_last_verdict'', ''var'')') == 1;
Rq = [];  if evalin('base', 'exist(''rome_last_run'', ''var'')') == 1, Rq = evalin('base', 'rome_last_run'); end
okQ = isempty(errQ) && nTq == 0 && ~gotVq && isstruct(Rq) && isfield(Rq, 'by_code') && Rq.by_code && Rq.robot;
Res.q = struct('ok', okQ, 'txt', sprintf('sim() error ''%s''; timers right after %d; verdict after 3 s %d; stop record by_code %s robot %s', ...
               errQ, nTq, gotVq, mat2str(isstruct(Rq) && isfield(Rq, 'by_code') && Rq.by_code), mat2str(isstruct(Rq) && Rq.robot)));
delete(timerfind('Name', 'rome_run_finish'));
if bdIsLoaded(mdl), close_system(mdl, 0); end
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dcBase));  fclose(fid);

% ---- checks ------------------------------------------------------------------------------------------------------------
fails = {};
ids = {'tstop', 'refusal', 'watchdog', 'stopall', 'saved', 'wheels'};
stOf = @(V, id) V.items(strcmp({V.items.id}, id)).status;
% (p)
okP = true;
for nm = {'P2', 'P3'}
    S = Res.(nm{1});  V = S.V;  ok = isstruct(V) && strcmp(S.stopFcn, 'rome_run_end(bdroot)');
    if ok
        ok = strcmp(V.verdict, 'PASS') && all(cellfun(@(i) strcmp(stOf(V, i), 'PASS'), ids));
        if S.mode == 3
            ok = ok && strcmp(stOf(V, 'arm'), 'UNSOURCED') && contains(V.items(strcmp({V.items.id}, 'arm')).detail, 'max |telemetry - command|');
        end
        fl = S.R.file;  okF = ~isempty(fl) && isfile(fl) && ~isempty(regexp(fl, sprintf('_s1_mode%d\\.mat$', S.mode), 'once'));
        if okF
            L = load(fl);
            okF = isfield(L, 'out') && isfield(L, 'snap') && isequal(L.snap.run_mode, S.mode) && isequal(L.snap.lab.max_rpm, 80) && ...
                  strcmp(L.out.SimulationMetadata.TimingInfo.WallClockTimestampStart, S.outStart) && strcmp(S.R.out_start, S.outStart);   % the NEW output, not the previous run's
        end
        ok = ok && okF;
        fprintf('(p) %s: verdict %s, items %s, file %s -> %s\n', nm{1}, V.verdict, strjoin(cellfun(@(i) stOf(V, i), ids, 'UniformOutput', false), ' '), fl, mat2str(ok));
    else
        fprintf('(p) %s: no verdict (StopFcn ''%s'') -> false\n', nm{1}, S.stopFcn);
    end
    okP = okP && ok;
end
if ~okP, fails{end+1} = '(p) a fault-free robot run does not PASS with its file saved'; end
% (f)
want = struct('tstop', {{'tstop'}}, 'watchdog', {{'watchdog'}}, 'stopall', {{'stopall'}}, 'saved', {{'saved'}}, 'wheels', {{'wheels'}});
okFl = true;
for r = 1:size(RUNS, 1)
    [nm, n, flt] = RUNS{r,:};
    if isempty(flt), continue; end
    V = Res.(nm).V;
    if ~isstruct(V)
        fprintf('(f) %s: no verdict -> false\n', nm);  okFl = false;  continue
    end
    st = cellfun(@(i) stOf(V, i), ids, 'UniformOutput', false);
    if strcmp(flt, 'refusal')
        ok = strcmp(V.verdict, 'FAIL') && isequal(st, {'FAIL', 'FAIL', 'NOT MEASURED', 'NOT MEASURED', 'FAIL', 'NOT MEASURED'}) && ...
             contains(V.items(strcmp({V.items.id}, 'refusal')).detail, 'arm_freeze_banner:stale') && isequal(V.mode, n);
    else
        w = repmat({'PASS'}, 1, numel(ids));  w(ismember(ids, want.(flt))) = {'FAIL'};
        judged = true(1, numel(ids));                  % items compared (logical)
        if ~strcmp(flt, 'watchdog'), judged = ~strcmp(ids, 'watchdog'); end   % ruling of 20 Nov 2026 (header): reported, not required
        ok = strcmp(V.verdict, 'FAIL') && isequal(st(judged), w(judged));
    end
    wdNote = '';
    if ~any(strcmp(flt, {'watchdog', 'refusal'})), wdNote = sprintf(' (watchdog %s: reported, not judged)', stOf(V, 'watchdog')); end
    fprintf('(f) %s (%s): verdict %s, items %s%s -> %s\n', nm, flt, V.verdict, strjoin(st, ' '), wdNote, mat2str(ok));
    okFl = okFl && ok;
end
if ~okFl, fails{end+1} = '(f) an injected fault does not fail exactly its item'; end
% (s)
s1 = Res.m1.save1;  s2 = Res.P2;
okS1 = isempty(s1.err) && ~isempty(s1.f) && ~isempty(regexp(s1.f, '_s1_mode1\.mat$', 'once')) && isstruct(s1.L) && isequal(s1.L.snap.run_mode, 1) && ...
       isequal(s1.rmBase, 2) && strcmp(s1.f2, s1.f) && contains(s1.txt, 'already saved') && strcmp(s1.idNo, 'save_run:noout') && isempty(Res.m1.V);
okS2 = isfield(s2, 'saveAgain') && isempty(s2.saveAgain.err) && strcmp(s2.saveAgain.f, s2.R.file) && s2.saveAgain.nAfter == s2.saveAgain.nBefore && ...
       contains(s2.saveAgain.txt, 'already saved');
fprintf('(s) m1: save_run() -> %s (snapshot run_mode %s with the base at %d), again -> same %d, no out -> %s; P2: save_run() returns the automatic file %d, no new file %d -> %s\n', ...
        s1.f, mat2str(s1.L.snap.run_mode), s1.rmBase, strcmp(s1.f2, s1.f), s1.idNo, isfield(s2, 'saveAgain') && strcmp(s2.saveAgain.f, s2.R.file), ...
        isfield(s2, 'saveAgain') && s2.saveAgain.nAfter == s2.saveAgain.nBefore, mat2str(okS1 && okS2));
if ~(okS1 && okS2), fails{end+1} = '(s) save_run() with no arguments'; end
% (q)
fprintf('(q) a robot run started by sim() from code: %s -> %s\n', Res.q.txt, mat2str(Res.q.ok));
if ~Res.q.ok, fails{end+1} = '(q) a run started by sim() from code started a verdict timer or printed a verdict'; end
% (o)
fprintf('(o) a verdict timer facing only the previous output: %s -> %s\n', Res.o.txt, mat2str(Res.o.ok));
if ~Res.o.ok, fails{end+1} = '(o) the verdict took the previous run''s output'; end
% (t)
nT = numel(timerfind('Name', 'rome_run_finish'));
fprintf('(t) rome_run_finish timers left: %d\n', nT);
if nT ~= 0, fails{end+1} = '(t) a verdict timer is left running'; end
if ~isempty(fails)
    tag = '';  if ~isempty(ib), tag = sprintf(' [break %s]', B{ib,1}); end
    error('test_run_verdict:fail', 'test_run_verdict FAILED: %s%s', strjoin(fails, '; '), tag);
end
fprintf('\ntest_run_verdict PASS: (p) (f) (s) (q) (o) (t)\n');
end
