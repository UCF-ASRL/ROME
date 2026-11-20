function varargout = test_romecommand_sendlog(brk)
%TEST_ROMECOMMAND_SENDLOG  The send log added to ROMECommand.m changes nothing the robot sees (20 Nov 2026).
%   Author, 20 Nov 2026: "IT's okay to edit the ROMECommand, but my worry is that we change too much and it fails. SO
%   we need intelligent ways to ensure this will indeed work based on the setup that has worked in the pass".
%   The setup that worked: ROMECommand.m of lab commit 63a10aa (Keanu, ran on the robot 24 Sep; unchanged up to lab
%   origin/main), git blob 05dcb7b42fbcb738942d2d16bee652bb7fca92d1, stored here as tests/ROMECommand_lab_05dcb7b.txt
%   (LF, as git stores it; its blob id is asserted, so the test runs without the lab repo).
%   NO NETWORK. Every run uses a TEMPORARY COPY of ROME_merged in which ROMECommand.m is GENERATED from a text with ONLY
%   tcpclient(...) replaced by a fake link of tests/stubs_hw (one match asserted): RobotLinkEcho records every write
%   (robot_link_record.txt: simulation time, java nanoTime stamp, bytes) and answers each ROME line with ARM telemetry.
%   The MotiveData block points at MotiveDataCounting (no socket), as in test_run_verdict. Runs are started by sim().
%   Runs (scenario 1, the case-1 lap to the model's T_stop unless noted; paced, EnableHardware = 1):
%     O2, O3   OLD = the baseline text (CRLF, as the lab checks it out), modes 2 and 3
%     N2, N3   NEW = ROME_merged/ROMECommand.m, send log on (lab_settings romecommand_sendlog = 1)
%     F2, F3   NEW, send log off (the copy's lab_settings line set to romecommand_sendlog = 0)
%     T2       NEW with the inserted step-path statements timed (tic/toc around each, copy only), mode 2
%     X2       NEW, mode 2, T_stop 4 s, the fake link FAILS at STOP_ALL (RobotLinkStopFail: write errors, nothing sent)
%     Y2       NEW, mode 2, T_stop 4 s, the STOP_ALL write and its log line removed from the copy (no STOP_ALL sent)
%   Checks (all evaluated, then one error test_romecommand_sendlog:fail listing the failed ones):
%     P1 diff     the baseline's blob id is 05dcb7b...; ROMECommand.m has CRLF line ends only; walking its lines,
%                 every baseline line appears, in order, unchanged (trailing spaces included); every other line is an
%                 inserted line marked [sendlog]; and removing the marked lines gives the baseline (CRLF) byte for byte
%                 (no newline after the last 'end', as in the baseline).
%     P2 stream   per mode, the bytes the fake robot received (every W record, in order) and the connect record are
%                 identical for OLD, NEW-on and NEW-off; NEW-on left rome_send_log in the base, OLD and NEW-off did not;
%                 the copy's ROMECommand resolves in the copy. Wall time and send gaps are reported (may differ).
%     P3 log      N2, N3: (a) rome_send_log.line equals the received lines exactly (text with LF, order) and the last
%                 entry is STOP_ALL; (b) the log's longest ROME gap >= the fake robot's longest ROME gap - TOLLONG (1 ms):
%                 the log never under-reports the quantity the verdict's watchdog item judges (longest ROME gap vs 250 ms).
%                 RULING (controller, 20 Nov 2026), replacing the first rule "every gap within 10 ms": that tolerance was
%                 chosen before any measurement and has no source; the first run measured per-gap differences of 26-32 ms
%                 with the log's longest gap above the robot's in all four runs (72.8 vs 56.7, 82.7 vs 56.9, 1006.9 vs
%                 976.4, 955.4 vs 923.2 ms), as expected when the log stamps after write() returns (one slow write
%                 lengthens one gap and shortens the next). The per-gap differences stay printed, information only.
%                 COST IF WRONG: on the real robot the TCP/serial link adds latency the fake robot does not model, so (b)
%                 is NOT VERIFIED on hardware. Simulation time of each ROME line (step index x dt_9dof) versus the fake
%                 robot's is printed (information).
%     P4 overhead T2: added time per step = sum of the timed inserted statements of that step (connect step 0 apart,
%                 it holds the buffer allocation and Keanu's pause(0.5)). Bound: median <= 1 ms and max <= 10 ms per
%                 step, against dt_9dof = 50 ms. The release-time dump runs after STOP_ALL and is not a step cost.
%     P5 verdict  rome_run_finish(out, rome_last_run) with no recorder installed (as on the real link): N2, N3 stopall
%                 PASS and watchdog PASS or FAIL (judged, not NOT MEASURED), and N2's saved run file holds snap.send_log;
%                 O2 and F2 (no log): stopall NOT MEASURED (control: the log is what turns the item); X2 and Y2:
%                 stopall FAIL, with X2's log present and without STOP_ALL.
%   MAKE-IT-FAIL: test_romecommand_sendlog('<id>') applies one break (each edit asserted to match once) and runs only
%   the runs that break needs; it must FAIL at the named check. test_romecommand_sendlog('list') returns the table.
%   Flat function without local functions (CLAUDE.md section 5, crash 2). Clears the base workspace.
if nargin < 1 || isempty(brk), brk = ''; end
here = fileparts(mfilename('fullpath'));  root = fileparts(here);
mdl = 'ROME_9DOF';  lf = char(10);  cr = char(13);  crlf = [cr lf];
blobWant = '05dcb7b42fbcb738942d2d16bee652bb7fca92d1';   % git blob of UK-Simulink-v3/ROMECommand.m, lab 63a10aa = origin/main
MARK = '[sendlog]';                                % marker of every inserted line (char)
TOLLONG = 0.001;                                   % P3 (b): log's longest ROME gap may be at most this much below the robot's (s)
OVH_MED = 0.001;  OVH_MAX = 0.010;                 % P4 bounds per step (s)
Tshort = 4;                                        % X2, Y2 run length (s)
OLDC = 'tcpclient(obj.ESP32_IP, obj.ESP32_Port, "ConnectTimeout", 5)';   % the one transport line replaced (char)
SWON = [lf 'romecommand_sendlog = 1;'];  SWOFF = [lf 'romecommand_sendlog = 0;'];   % lab_settings switch line (char)
LSTOP = ['                write(obj.DeviceHandle, uint8(char("STOP_ALL" + newline)));' crlf ...
         '                if obj.sendlog_on_, try, sendlogAdd_(obj, char("STOP_ALL" + newline)); catch, end, end % [sendlog] the line just written' crlf];
B = {
  'mf_p1', 'P1', {},           {'p1', 'ESP32_Port = 3333;', 'ESP32_Port = 3334;'}, 'one character of an original line changed (3333 -> 3334) in the text P1 reads';
  'mf_p2', 'P2', {'O2', 'N2'}, {'new', 'try, sendlogInit_(obj); ', 'write(obj.DeviceHandle, uint8(10)); try, sendlogInit_(obj); '}, 'the inserted START_GV log line also writes one byte (LF) to the robot';
  'mf_p3', 'P3', {'N2'},       {'new', 'sendlogAdd_(obj, msg)', 'sendlogAdd_(obj, msg(1:end-1))'}, 'the ROME log entry drops the LF the robot received';
  'mf_p3gap', 'P3', {'N2'},    {'new', 'obj.sendlog_tw_(n) = toc(obj.sendlog_t0_);', 'obj.sendlog_tw_(n) = 0.5 * toc(obj.sendlog_t0_);'}, 'the log time stamps are halved: every gap shrinks, the log under-reports the longest gap (P3 (b))';
  'mf_p4', 'P4', {'T2'},       {'new', 'obj.sendlog_n_ = n; % [sendlog]', 'obj.sendlog_n_ = n; pause(0.02); % [sendlog]'}, '20 ms added to every log entry';
  'mf_p5', 'P5', {'N2'},       {'file', 'rome_run_finish.m', 'if evalin(''base'', ''exist(''''rome_send_log'''', ''''var'''')'') == 1', 'if false'}, 'rome_run_finish no longer reads rome_send_log';
};
if strcmp(brk, 'list'), varargout{1} = B; return; end
ib = [];
if ~isempty(brk)
    ib = find(strcmp(B(:,1), brk));
    assert(numel(ib) == 1, 'test_romecommand_sendlog:brk', 'unknown break %s', brk);
    fprintf('MAKE-IT-FAIL break %s (check %s must FAIL): %s\n', brk, B{ib,2}, B{ib,5});
end
RUNS = {'O2', 2, 'old', []; 'N2', 2, 'new', []; 'F2', 2, 'off', []; 'O3', 3, 'old', []; 'N3', 3, 'new', []; 'F3', 3, 'off', []; ...
        'T2', 2, 'timed', []; 'X2', 2, 'stopfail', Tshort; 'Y2', 2, 'nostop', Tshort};
if ~isempty(ib), RUNS = RUNS(ismember(RUNS(:,1), B{ib,3}), :); end
fails = {};

% ---- P1: ROMECommand.m = the baseline with inserted, marked lines only ------------------------------------------------
fid = fopen(fullfile(here, 'ROMECommand_lab_05dcb7b.txt'), 'r');  base = char(fread(fid, Inf, '*uint8').');  fclose(fid);
fid = fopen(fullfile(root, 'ROMECommand.m'), 'r');  newt = char(fread(fid, Inf, '*uint8').');  fclose(fid);
newRun = newt;                                     % the NEW text every run uses (char): never the P1 break
if ~isempty(ib) && strcmp(B{ib,4}{1}, 'p1')
    ed = B{ib,4};  n = numel(strfind(newt, ed{2}));
    assert(n == 1, 'test_romecommand_sendlog:brk', 'P1 break edit matches %d times (expected 1)', n);
    newt = strrep(newt, ed{2}, ed{3});
end
md = java.security.MessageDigest.getInstance('SHA-1');
blob = lower(reshape(dec2hex(typecast(md.digest([uint8(sprintf('blob %d', numel(base))) uint8(0) uint8(base)]), 'uint8'), 2).', 1, []));
okBlob = strcmp(blob, blobWant) && ~any(base == cr);
baseC = strrep(base, lf, crlf);                    % the baseline as the lab checks it out (CRLF) (char)
okCRLF = isequal(newt, strrep(strrep(newt, crlf, lf), lf, crlf));   % every LF has its CR, no other CR
bl = strsplit(base, lf, 'CollapseDelimiters', false);              % baseline lines (cellstr)
nl = strsplit(strrep(newt, crlf, lf), lf, 'CollapseDelimiters', false);   % new lines (cellstr)
j = 1;  insBad = {};  nIns = 0;                    % next baseline line to find; inserted lines without the marker
for k = 1:numel(nl)
    if j <= numel(bl) && strcmp(nl{k}, bl{j})
        j = j + 1;
    else
        nIns = nIns + 1;
        if ~contains(nl{k}, MARK), insBad{end+1} = sprintf('new line %d: %s', k, nl{k}); end %#ok<AGROW>
    end
end
okOrder = j == numel(bl) + 1;                      % every baseline line found, in order
stripped = strjoin(nl(~contains(nl, MARK)), crlf); % new text without the marked lines (char)
okStrip = isequal(stripped, baseC);
okP1 = okBlob && okCRLF && okOrder && isempty(insBad) && okStrip;
fprintf('P1  baseline blob %s (want %s, no CR %d); ROMECommand.m %d bytes, CRLF only %d; baseline lines found in order %d of %d; inserted lines %d, unmarked %d; marked lines removed == baseline CRLF byte for byte %d -> %s\n', ...
        blob, blobWant, ~any(base == cr), numel(newt), okCRLF, j - 1, numel(bl), nIns, numel(insBad), okStrip, mat2str(okP1));
for k = 1:min(5, numel(insBad)), fprintf('    %s\n', insBad{k}); end
if ~okOrder, fprintf('    first baseline line not found in order: %d: %s\n', j, bl{min(j, numel(bl))}); end
if ~okP1, fails{end+1} = 'P1 ROMECommand.m is not the baseline plus marked inserted lines'; end

Res = struct();
if ~isempty(RUNS)
% ---- copy, stubs, path ------------------------------------------------------------------------------------------------
tmp = tempname;  startdir = pwd;  path0 = path;
if bdIsLoaded(mdl), close_system(mdl, 0); end
mkdir(tmp);  copyfile(root, tmp);
prerun_fixture('signoff', tmp);                    % Part F/C sign-off dates in the COPY's lab_settings.m
mdlcell = {mdl};
cleanup = onCleanup(@() cellfun(@(c) c(), {@() cd(startdir), @() path(path0), ...
          @() cellfun(@(m) close_system(m, 0), mdlcell(bdIsLoaded(mdl))), @() evalin('base', 'clear all'), @() rmdir(tmp, 's')})); %#ok<NASGU>
for f = {'RobotLinkRecorder.m', 'RobotLinkEcho.m', 'MotiveDataCounting.m'}
    copyfile(fullfile(here, 'stubs_hw', f{1}), fullfile(tmp, f{1}), 'f');
end
fid = fopen(fullfile(tmp, 'RobotLinkStopFail.m'), 'w');
fprintf(fid, '%s\n', 'classdef RobotLinkStopFail < RobotLinkEcho', ...
    '    % test_romecommand_sendlog X2: the fake robot whose link fails at STOP_ALL (the write errors, nothing is recorded).', ...
    '    methods', '        function obj = RobotLinkStopFail(varargin)', '            obj@RobotLinkEcho(varargin{:});', '        end', ...
    '        function write(obj, data, varargin)', ...
    '            if startsWith(char(data(:).''), ''STOP_ALL''), error(''RobotLinkStopFail:lost'', ''link lost at STOP_ALL (injected)''); end', ...
    '            write@RobotLinkEcho(obj, data, varargin{:});', '        end', '    end', 'end');
fclose(fid);
fid = fopen(fullfile(tmp, 'sendlog_ovh.m'), 'w');
fprintf(fid, '%s\n', 'function out = sendlog_ovh(x)', '% test_romecommand_sendlog T2: collects [step type seconds] rows; sendlog_ovh(''get'') returns and clears them.', ...
    'persistent T', 'out = [];', 'if ischar(x), out = T; T = []; return; end', 'T(end+1, :) = x;', 'end');
fclose(fid);
if ~isempty(ib) && strcmp(B{ib,4}{1}, 'file')
    ed = B{ib,4};  f = fullfile(tmp, ed{2});  s = fileread(f);  n = numel(strfind(s, ed{3}));
    assert(n == 1, 'test_romecommand_sendlog:brk', 'break edit in %s matches %d times (expected 1)', ed{2}, n);
    fid = fopen(f, 'w');  fwrite(fid, uint8(strrep(s, ed{3}, ed{4})));  fclose(fid);
end
addpath(tmp, fullfile(tmp, 'Ground Vehicle'), fullfile(tmp, 'Optitrack'));
addpath(here, '-end');                             % tests: dc_set_mode, read_link_record, prerun_fixture
cd(tmp);  rehash;
dcf = fullfile(tmp, 'define_constants.m');  labf = fullfile(tmp, 'lab_settings.m');  slx = fullfile(tmp, [mdl '.slx']);
cmdf = fullfile(tmp, 'ROMECommand.m');  recf = fullfile(tmp, 'robot_link_record.txt');
dcBase = fileread(dcf);  labBase = fileread(labf);
assert(numel(strfind(labBase, SWON)) == 1, 'test_romecommand_sendlog:lab', 'lab_settings.m: ''romecommand_sendlog = 1;'' found %d times', numel(strfind(labBase, SWON)));
for c = {'rome_run_end', 'rome_run_finish', 'run_verdict_core', 'save_run', 'define_constants', 'RobotLinkEcho', 'RobotLinkStopFail', 'sendlog_ovh'}
    assert(strcmp(fileparts(which(c{1})), tmp), 'test_romecommand_sendlog:path', '%s resolves to %s', c{1}, which(c{1}));
end
% the run texts: one asserted replacement of the transport constructor
newE = newRun;
if ~isempty(ib) && strcmp(B{ib,4}{1}, 'new')
    ed = B{ib,4};  n = numel(strfind(newE, ed{2}));
    assert(n == 1, 'test_romecommand_sendlog:brk', 'break edit matches %d times in ROMECommand.m (expected 1)', n);
    newE = strrep(newE, ed{2}, ed{3});
end
assert(numel(strfind(baseC, OLDC)) == 1 && numel(strfind(newE, OLDC)) == 1, 'test_romecommand_sendlog:stub', 'tcpclient constructor not found exactly once');
TX = struct();
TX.old = strrep(baseC, OLDC, strrep(OLDC, 'tcpclient', 'RobotLinkEcho'));
TX.new = strrep(newE, OLDC, strrep(OLDC, 'tcpclient', 'RobotLinkEcho'));
TX.off = TX.new;
TX.stopfail = strrep(newE, OLDC, strrep(OLDC, 'tcpclient', 'RobotLinkStopFail'));
assert(numel(strfind(TX.new, LSTOP)) == 1, 'test_romecommand_sendlog:stub', 'STOP_ALL write + log lines found %d times', numel(strfind(TX.new, LSTOP)));
TX.nostop = strrep(TX.new, LSTOP, '');
IN = {'try, sendlogInit_(obj); sendlogAdd_(obj, char("START_GV" + newline)); catch, obj.sendlog_on_ = false; end', ...
      'sl_t = tic; try, sendlogInit_(obj); sendlogAdd_(obj, char("START_GV" + newline)); catch, obj.sendlog_on_ = false; end; sendlog_ovh([0 0 toc(sl_t)]);';
      'if obj.sendlog_on_, try, sendlogAdd_(obj, msg); catch, end, end', ...
      'sl_t = tic; if obj.sendlog_on_, try, sendlogAdd_(obj, msg); catch, end, end; sendlog_ovh([obj.sendlog_k_ 1 toc(sl_t)]);';
      'if obj.sendlog_on_, obj.sendlog_k_ = obj.sendlog_k_ + 1; end', ...
      'sl_t = tic; if obj.sendlog_on_, obj.sendlog_k_ = obj.sendlog_k_ + 1; end; sendlog_ovh([obj.sendlog_k_-1 2 toc(sl_t)]);'};   % timed forms of the step-path insertions
TX.timed = TX.new;
for k = 1:size(IN, 1)
    if any(strcmp(RUNS(:,3), 'timed'))
        assert(numel(strfind(TX.timed, IN{k,1})) == 1, 'test_romecommand_sendlog:stub', 'timing edit %d matches %d times', k, numel(strfind(TX.timed, IN{k,1})));
    end
    TX.timed = strrep(TX.timed, IN{k,1}, IN{k,2});
end
blkM = [mdl '/Enabled' newline 'Subsystem/MATLAB System'];   % the MotiveData block

% ---- runs --------------------------------------------------------------------------------------------------------------
for r = 1:size(RUNS, 1)
    [nm, n, var, Ts] = RUNS{r,:};
    fprintf('\n---- run %s: mode %d, %s, T_stop %s ----\n', nm, n, var, mat2str(Ts));
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    lab = labBase;  if strcmp(var, 'off'), lab = strrep(lab, SWON, SWOFF); end
    fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc_set_mode(dcBase, n)));  fclose(fid);
    fid = fopen(labf, 'w');  fwrite(fid, uint8(lab));  fclose(fid);
    fid = fopen(cmdf, 'w');  fwrite(fid, uint8(TX.(var)));  fclose(fid);
    if isfile(recf), delete(recf); end
    evalin('base', 'clear all');  evalin('base', 'clear classes');
    clear define_constants lab_settings ROMECommand RobotLinkEcho RobotLinkRecorder RobotLinkStopFail sendlog_ovh;  rehash;
    fid = fopen(which('ROMECommand'), 'r');  got = char(fread(fid, Inf, '*uint8').');  fclose(fid);
    okFile = strcmp(fileparts(which('ROMECommand')), tmp) && isequal(got, TX.(var));
    evalc('load_system(slx)');
    set_param(mdl, 'SimulationMode', 'normal');
    sim0 = get_param(blkM, 'SimulateUsing');  set_param(blkM, 'System', 'MotiveDataCounting');  set_param(blkM, 'SimulateUsing', sim0);
    if ~isempty(Ts), assignin('base', 'T_stop', Ts); end
    sw = NaN;  try, sw = evalin('base', 'romecommand_sendlog'); catch, end
    prerun_fixture('park', tmp);
    o = [];  errS = '';  t0 = tic;
    try
        txtS = evalc('o = sim(mdl, ''ReturnWorkspaceOutputs'', ''on'', ''CaptureErrors'', ''on'');'); %#ok<NASGU>
    catch ex
        errS = ex.message;
    end
    wall = toc(t0);
    try, if isempty(errS) && ~isempty(o.ErrorMessage), errS = char(o.ErrorMessage); end, catch, end
    ovh = [];  if strcmp(var, 'timed'), ovh = sendlog_ovh('get'); end
    rec = read_link_record(recf);
    C = {};  if isfile(recf), Cr = textscan(fileread(recf), '%s %f %f %s', 'Delimiter', '\t');  C = Cr{4}(strcmp(Cr{1}, 'C')); end
    SL = [];  if evalin('base', 'exist(''rome_send_log'', ''var'')') == 1, SL = evalin('base', 'rome_send_log'); end
    R = [];  if evalin('base', 'exist(''rome_last_run'', ''var'')') == 1, R = evalin('base', 'rome_last_run'); end
    V = [];  Lsnap = [];  txtV = '';
    if ismember(nm, {'N2', 'N3', 'O2', 'F2', 'X2', 'Y2'}) && isstruct(R)
        if ~isa(o, 'Simulink.SimulationOutput'), o = struct('ErrorMessage', errS); end
        evalin('base', 'clear rome_send_recorder');   % the real link has no recorder: the verdict must use the log
        try, txtV = evalc('V = rome_run_finish(o, R);'); catch ex, txtV = ['rome_run_finish error: ' ex.message]; end
        try, R2 = evalin('base', 'rome_last_run');  Lsnap = load(R2.file, 'snap'); catch, end
    end
    S = struct('name', nm, 'mode', n, 'var', var, 'okFile', okFile, 'sw', sw, 'wall', wall, 'err', errS, 'rec', rec, 'C', {C}, ...
               'SL', SL, 'R', R, 'V', V, 'Lsnap', Lsnap, 'ovh', ovh, 'txtV', txtV);
    Res.(nm) = S;
    gR = diff(rec.tw(startsWith(rec.line, 'ROME,')));   % fake robot gaps between ROME lines (s)
    if isempty(gR), gR = NaN; end
    lastTxt = '';  if ~isempty(rec.line), lastTxt = strtrim(rec.line{end}); end
    nEnt = 0;  if isstruct(SL), nEnt = numel(SL.line); end
    fprintf('[%s] file as intended %d, romecommand_sendlog %s; wall %.2f s; error ''%s''; received %d lines (last ''%s''); ROME gaps median %.1f ms max %.1f ms; base rome_send_log %d (%d entries)\n', ...
            nm, okFile, mat2str(sw), wall, errS, numel(rec.line), lastTxt, 1e3 * median(gR), 1e3 * max(gR), isstruct(SL), nEnt);
    if ~isempty(V), fprintf('[%s] %s\n', nm, strrep(V.text, newline, [newline '[' nm '] '])); elseif ~isempty(txtV), fprintf('[%s] %s\n', nm, txtV); end
    if bdIsLoaded(mdl), close_system(mdl, 0); end
end
end

% ---- P2: identical streams ---------------------------------------------------------------------------------------------
okP2 = true;  ranP2 = false;
for n = [2 3]
    o_ = sprintf('O%d', n);  v = {sprintf('N%d', n), sprintf('F%d', n)};
    if ~isfield(Res, o_), continue; end
    for k = 1:2
        if ~isfield(Res, v{k}), continue; end
        ranP2 = true;  A = Res.(o_);  Bn = Res.(v{k});
        same = isequal(A.rec.line, Bn.rec.line) && isequal(A.C, Bn.C) && ~isempty(A.rec.line) && A.okFile && Bn.okFile;
        logOk = ~isstruct(A.SL) && (isstruct(Bn.SL) == strcmp(Bn.var, 'new'));
        nd = NaN;
        if ~same
            m = min(numel(A.rec.line), numel(Bn.rec.line));  nd = find(~cellfun(@isequal, A.rec.line(1:m), Bn.rec.line(1:m)), 1);
            if isempty(nd), nd = m + 1; end
        end
        fprintf('P2  mode %d: %s (%d lines, %.2f s wall) vs %s (%d lines, %.2f s wall): identical bytes and order %d (first difference at line %s); log only when on %d\n', ...
                n, o_, numel(A.rec.line), A.wall, v{k}, numel(Bn.rec.line), Bn.wall, same, mat2str(nd), logOk);
        okP2 = okP2 && same && logOk;
    end
end
if ranP2 && ~okP2, fails{end+1} = 'P2 the robot stream differs between the old and the new ROMECommand.m'; end
if ~ranP2, fprintf('P2  NOT RUN (break subset)\n'); end

% ---- P3: log = what the robot received -----------------------------------------------------------------------------
okP3 = true;  ranP3 = false;
for nm = {'N2', 'N3'}
    if ~isfield(Res, nm{1}), continue; end
    ranP3 = true;  S = Res.(nm{1});  ok = isstruct(S.SL);
    if ok
        L = S.SL.line(:);  rl = S.rec.line(:);
        sameL = isequal(L, rl);
        lastStop = ~isempty(L) && strcmp(L{end}, ['STOP_ALL' lf]);
        dg = NaN;  if numel(L) == numel(rl) && numel(L) > 1, dg = max(abs(diff(S.SL.tw(:)) - diff(S.rec.tw(:)))); end
        iL = startsWith(L, 'ROME,');  iRr = startsWith(rl, 'ROME,');
        gL = max(diff(S.SL.tw(iL)));  gRr = max(diff(S.rec.tw(iRr)));
        dts = NaN;  if isequal(iL, iRr), dts = max(abs(S.SL.ts(iL) - S.rec.ts(iRr))); end
        okLong = gL >= gRr - TOLLONG;           % (b) the log never under-reports the longest ROME gap
        ok = sameL && lastStop && okLong && isempty(S.SL.err);
        if numel(L) == numel(rl) && numel(L) > 1   % per-gap differences, information only (P3 ruling, 20 Nov)
            dd = diff(S.SL.tw(:)) - diff(S.rec.tw(:));  [~, iw] = sort(abs(dd), 'descend');
            for q = iw(1:min(5, end)).'
                fprintf('    gap %d->%d: log %.2f ms, robot %.2f ms, difference %.2f ms; line %d ''%s''\n', q, q + 1, ...
                        1e3 * (S.SL.tw(q + 1) - S.SL.tw(q)), 1e3 * (S.rec.tw(q + 1) - S.rec.tw(q)), 1e3 * dd(q), q + 1, strtrim(L{q + 1}(1:min(end, 30))));
            end
            fprintf('    |difference| > 1 ms in %d of %d gaps, > 5 ms in %d\n', sum(abs(dd) > 1e-3), numel(dd), sum(abs(dd) > 5e-3));
        end
        fprintf('P3  %s: (a) log %d entries = received %d lines exactly %d, last entry STOP_ALL %d; (b) longest ROME gap log %.1f ms >= robot %.1f ms - %.0f ms: %d; max |gap(log) - gap(robot)| %.3f ms (information); ROME ts (step x dt) vs robot sim time max diff %.3g s (information); log err ''%s'' -> %s\n', ...
                nm{1}, numel(L), numel(rl), sameL, lastStop, 1e3 * gL, 1e3 * gRr, 1e3 * TOLLONG, okLong, 1e3 * dg, dts, S.SL.err, mat2str(ok));
    else
        fprintf('P3  %s: no rome_send_log in the base -> false\n', nm{1});
    end
    okP3 = okP3 && ok;
end
if ranP3 && ~okP3, fails{end+1} = 'P3 the send log is not what the robot received'; end
if ~ranP3, fprintf('P3  NOT RUN (break subset)\n'); end

% ---- P4: overhead per step ---------------------------------------------------------------------------------------------
if isfield(Res, 'T2')
    X = Res.T2.ovh;  okP4 = ~isempty(X);
    if okP4
        k0 = X(:, 1) == 0;  st = accumarray(X(~k0, 1), X(~k0, 3));   % per step k >= 1 (s)
        st = st(st > 0);  c0 = sum(X(k0, 3));  ss = sort(st);
        okP4 = ~isempty(st) && median(st) <= OVH_MED && max(st) <= OVH_MAX;
        fprintf('P4  T2: %d steps timed: added time per step median %.1f us, 95th pct %.1f us, max %.1f us (bound median <= %.0f us, max <= %.0f us; dt 50 ms); connect step %.2f ms (buffer allocation; that step already holds pause(0.5)) -> %s\n', ...
                numel(st), 1e6 * median(st), 1e6 * ss(ceil(0.95 * numel(ss))), 1e6 * max(st), 1e6 * OVH_MED, 1e6 * OVH_MAX, 1e3 * c0, mat2str(okP4));
        if isfield(Res, 'N2'), fprintf('    T2 stream equal to N2 %d (information)\n', isequal(Res.T2.rec.line, Res.N2.rec.line)); end
    else
        fprintf('P4  T2: no timing rows -> false\n');
    end
    if ~okP4, fails{end+1} = 'P4 the send log adds more than the bound per step'; end
else
    fprintf('P4  NOT RUN (break subset)\n');
end

% ---- P5: the verdict reads the log ---------------------------------------------------------------------------------------
stOf = @(V, id) V.items(strcmp({V.items.id}, id)).status;
want = {'N2', 'judged'; 'N3', 'judged'; 'O2', 'NOT MEASURED'; 'F2', 'NOT MEASURED'; 'X2', 'FAIL'; 'Y2', 'FAIL'};
okP5 = true;  ranP5 = false;
for k = 1:size(want, 1)
    nm = want{k,1};
    if ~isfield(Res, nm), continue; end
    ranP5 = true;  S = Res.(nm);  V = S.V;
    if ~isstruct(V), fprintf('P5  %s: no verdict (%s) -> false\n', nm, S.txtV);  okP5 = false;  continue; end
    sa = stOf(V, 'stopall');  wd = stOf(V, 'watchdog');
    switch want{k,2}
        case 'judged'
            ok = strcmp(sa, 'PASS') && ismember(wd, {'PASS', 'FAIL'}) && isstruct(S.Lsnap) && isfield(S.Lsnap, 'snap') && isfield(S.Lsnap.snap, 'send_log');
        case 'NOT MEASURED'
            ok = strcmp(sa, 'NOT MEASURED') && strcmp(wd, 'NOT MEASURED');
        case 'FAIL'
            ok = strcmp(sa, 'FAIL') && ismember(wd, {'PASS', 'FAIL'});
            if strcmp(nm, 'X2'), ok = ok && isstruct(S.SL) && ~any(strcmp(strtrim(S.SL.line), 'STOP_ALL')) && numel(S.SL.line) > 1; end
    end
    fprintf('P5  %s: verdict %s, stopall %s, watchdog %s (want %s); %s -> %s\n', nm, V.verdict, sa, wd, want{k,2}, ...
            V.items(strcmp({V.items.id}, 'stopall')).detail, mat2str(ok));
    okP5 = okP5 && ok;
end
if ranP5 && ~okP5, fails{end+1} = 'P5 the verdict does not judge STOP_ALL and the watchdog from the send log'; end
if ~ranP5, fprintf('P5  NOT RUN (break subset)\n'); end

if ~isempty(fails)
    tag = '';  if ~isempty(ib), tag = sprintf(' [break %s]', B{ib,1}); end
    error('test_romecommand_sendlog:fail', 'test_romecommand_sendlog FAILED: %s%s', strjoin(fails, '; '), tag);
end
if ~isempty(ib)
    error('test_romecommand_sendlog:notcaught', 'break %s did NOT make check %s fail', B{ib,1}, B{ib,2});
end
fprintf('\ntest_romecommand_sendlog PASS: P1 P2 P3 P4 P5\n');
end
