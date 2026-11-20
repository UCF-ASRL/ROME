function varargout = test_hw_nocam(brk)
%TEST_HW_NOCAM  Hardware mode WITHOUT cameras (EnableHardware = 1, EnableMotive = 0), end to end against a
%   [F7 fix round 1: the user-block default is now EnableMotive = 1 (the author's four modes all run the camera
%   pipeline); this test forces EnableMotive = 0 in every run as the regression of the pre-F7 loop.]
%   recording fake robot (Plan 2 Task 1c). Author, 19 Nov 2026: "we need to ensure that if the model is
%   running without the camera but with the actual hardware, that this setup works corectly as well."
%
%   NO NETWORK, EVER. The test runs a TEMPORARY COPY of ROME_merged (tempname) in which
%     ROMECommand.m        is tests/stubs_hw/ROMECommand.m: Keanu's ROMECommand.m with ONLY the transport
%                          constructor replaced (tcpclient -> RobotLinkRecorder). Asserted below, byte for
%                          byte, against the real ROME_merged/ROMECommand.m, so the stub cannot drift.
%     RobotLinkRecorder.m  the fake link: opens no socket, appends every line ROMECommand writes to a file
%                          in the copy, with the simulation time and a wall-clock stamp of the write.
%     MotiveDataCounting.m the REAL MotiveData class with only its two network seams overridden (openSocket
%                          counts its calls in a file and opens nothing; receiveBytes returns no datagram).
%                          The copy's Enabled Subsystem MotiveData block is pointed at it in memory.
%   ROME_merged/ROMECommand.m and ROME_merged/define_constants.m are never written. The copy's
%   define_constants.m is rewritten by the test (one flag line per run, each edit asserted to match once).
%
%   Mode under test (wiring re-verified 19 Nov 2026 on the current slx, docs/verification/
%   2026-11-19_p2t1c_wiring.log; map in 2026-11-19_plan2_interface_map.md): StateSource (Switch SID 218,
%   u2 > 0.5 on UseMotive = EnableMotive) takes SensorModel (SID 224), which reads StateDelay (UnitDelay
%   SID 199, IC q0_9dof) of q_cmd. The loop is open: no camera or arm telemetry enters it. CommandGuard
%   (SID 214) feeds ROMECommand (SID 216) port 1 and To Workspace wheel_sent (SID 227) directly; its joint
%   output (port 2) goes to To Workspace joint_guard_rad (SID 240) and to ArmFreezeSwitch (SID 239, focus plan
%   F1, docs/verification/2026-11-19_f1_wire_arm_freeze.log), whose output feeds ROMECommand port 2 and To
%   Workspace joint_sent_rad (SID 228); the Constant 'Enable Hardware' (SID 217) is ROMECommand's Enable (port 3).
%
%   Four simulations of scenario 1 (define_constants as in ROME_merged), each loaded fresh in the copy. ref, hw
%   and hw_err run with arm_freeze = 0 (copy's user-block line 'arm_freeze = 1;' -> 0), so the arm command moves
%   and check (i) compares a moving joint stream; hw_frz keeps the file's default arm_freeze = 1.
%     ref     EnableHardware = 0 and sensor_model = 0 (copy's user-block line 'sensor_model = 1;' -> 0).
%             sensor_model = 0 because the hardware run forces it: with the noise on, the commands would
%             legitimately differ. Run as fast as the machine goes (pacing off).
%     hw      EnableHardware = 1 (copy's user-block line). define_constants then forces sensor_model = 0 and
%             apply_pacing turns pacing on (PostLoadFcn, InitFcn, StartFcn): the path a student takes.
%             Runs to the model's StopTime T_stop.
%     hw_err  as hw, with an Assertion block added in memory (Clock9 < t_err), so the run ends in an ERROR
%             raised outside ROMECommand at t_err.
%     hw_frz  as hw, with arm_freeze = 1 (Plan 2 Task 5 / focus plan F1: ArmFreezeSwitch upstream of ROMECommand,
%             reading arm_freeze_user; tools/wire_arm_freeze.m).
%   Each sim is run inside evalc so the model's InitFcn banner can be checked; the captured text is printed.
%   Checks (all evaluated, then one error listing the failed ones):
%     (0)  ref: the EnableHardware = 0 run connected to nothing and sent nothing (ROMECommand Enable gating).
%     (i)  hw: the ROME,... lines equal, step by step, ref's wheel_sent and joint_sent_rad passed through the
%          same map (arm_sign, arm_offset_deg) and clamp (arm_fw_switch / arm_fw_other / arm_fw_margin_deg),
%          every field within half a unit of the %.3f print; one line per logged step, at the logged step
%          times; q_cmd of hw bit-identical to ref; and the same against hw's own wheel_sent/joint_sent_rad.
%     (ii) hw: pacing on: wall-clock span between the first and the last ROME line >= 0.9 x their simulated
%          span (compilation excluded), and EnablePacing 'on' at PacingRate 1 after the run.
%     (iii) hw: every sent wheel |w| <= max_rpm and every sent joint inside the firmware window minus the
%          margin, limits taken from ROME_merged/define_constants.m (the user's file, not the run's
%          workspace, so a broken copy cannot move its own oracle).
%     (iv) hw: sensor_model is effectively 0: the logged q_meas equals q_cmd one step late exactly, with
%          q_meas(t=0) = q0_9dof, and sensor_model = 0 in the workspace; and MotiveData's openSocket was
%          called 0 times over all four runs (counter file).
%     (v)  hw and hw_err: STOP_ALL is the last line sent, exactly once, START_GV the first, exactly once,
%          every write one LF-terminated line; hw_err ended in the injected error, sent ROME lines before
%          it and none after t_err. Fix round 1 (I1): STOP_ALL is recorded BEFORE sim() returns (or throws), i.e.
%          before close_system, with the simulation time of the final step (hw: last logged step; hw_err: t_err),
%          so it is sent when the simulation terminates, not only when the model is closed.
%   The hw run's record and logs are saved to <tempdir>/test_hw_nocam_hw_record.mat (pacing-deficit arithmetic).
%   On failure the error message lists the failed checks and which runs returned from sim().
%     (vi) hw_frz (arm_freeze = 1): every ROME line's six joint fields equal arm_freeze_deg within half a unit of
%          the %.3f print, for the whole run (one line per logged step); arm_freeze_deg equals q0_9dof(4:9) through
%          the map and clamp of the user's define_constants.m (1e-9 deg); WHILE the controller's joint command
%          (joint_guard_rad, CommandGuard out 2) moves >= 10 deg on some joint; the wheel fields equal hw's (the
%          base still does everything); START_GV first, STOP_ALL last; banner 'ARM FROZEN at arm_freeze_deg' once
%          in hw_frz and 'ARM WILL MOVE' once in hw (and not the other one in either).
%
%   MAKE-IT-FAIL: test_hw_nocam('<id>') applies one break of the table below to the COPY (each text edit
%   asserted to match exactly once) and must then error naming its check. test_hw_nocam('list') returns
%   the table. tests/manual_makefail_hw_nocam.m runs every break and scores it.
%   Flat function without local functions (CLAUDE.md section 5, crash 2). Clears the base workspace.
if nargin < 1 || isempty(brk), brk = ''; end
here    = fileparts(mfilename('fullpath'));   % ROME_merged/tests (char)
root    = fileparts(here);                    % ROME_merged (char)
stubdir = fullfile(here, 'stubs_hw');         % recording stub and its two helpers (char)
mdl     = 'ROME_9DOF';                        % model name (char)
lf      = char(10);                           % line feed, the end of every line ROMECommand sends (char)
crlf    = char([13 10]);                      % line ending of ROMECommand.m (char)
t_err   = 3.0;                                % simulation time at which hw_err is made to fail (s)
tolField = 5e-4 + 1e-9;                       % half a unit of the %.3f print (rpm or deg) plus float slack
ratioMin = 0.9;                               % pacing: wall span >= ratioMin x simulated span (-)

% ---- break table: id | check that must FAIL | edits on the copy | what the break violates ----------------
% edit: {file relative to the copy, old, new} (exactly one match) or {'chart', chart name, old, new} (the
% MATLAB Function chart's script in the copy's model, exactly one match, model saved) or {'tool', mode}
% (the copy's tools/wire_arm_freeze.m run on the copy's model in that mode; it re-reads and asserts the result)
% or {'param', block in the model, parameter, value} (set on the copy's model, saved).
B = cell(0, 4);
B(end+1,:) = {'mf_i_swap', 'i', {{'ROMECommand.m', 'wheelRPM(1), wheelRPM(2), wheelRPM(3), wheelRPM(4), ...', ...
                                                   'wheelRPM(2), wheelRPM(1), wheelRPM(3), wheelRPM(4), ...'}}, ...
    'wheel fields 1 and 2 swapped in the message: the right speeds reach the wrong motors'};
B(end+1,:) = {'mf_ii_pacing', 'ii', {{'apply_pacing.m', 'state = {''off'', ''on''};', 'state = {''off'', ''off''};'}, ...
                                     {'apply_pacing.m', 'if live && ~(strcmp(got, ''on'') && isequal(double(rate), 1))', 'if false'}}, ...
    'apply_pacing turns pacing off for a hardware run and does not refuse it: commands stream faster than real time'};
B(end+1,:) = {'mf_iii_joint', 'iii', {{'ROMECommand.m', 'if obj.has_arm_clamp_', 'if false'}, ...
                                      {'define_constants.m', 'arm_offset_deg = [ 0 180  90   0   0   0];', 'arm_offset_deg = [ 0 180 150   0   0   0];'}}, ...
    'firmware clamp skipped and J3 offset 60 deg off: J3 is sent past the firmware window'};
B(end+1,:) = {'mf_iii_wheel', 'iii', {{'chart', 'CommandGuard', 'lim  = abs(max_rpm);', 'lim  = 10*abs(max_rpm);'}, ...
                                      {'define_constants.m', 'speed_factor = 1.0;', 'speed_factor = 2.0;'}}, ...
    'CommandGuard scales to 10 x max_rpm and the path is flown at twice the speed: wheels are sent above max_rpm'};
B(end+1,:) = {'mf_iv_sensor', 'iv', {{'define_constants.m', '    sensor_model = 0;  % forced', '    sensor_model = 1;  % forced'}, ...
                                     {'apply_pacing.m', '    if ~smok', '    if false'}}, ...
    'sensor_model left on in a hardware run (override and refusal both removed): synthetic noise in the state the controller reads'};
B(end+1,:) = {'mf_iv_motive', 'iv', {{'emrun', 1}, ...
                                     {'define_constants.m', '    ''measured'', false);', '    ''measured'', true);'}, ...
                                     {'srcrun', 'optitrack'}}, ...
    'EnableMotive = 1: the MotiveData block runs and opens its socket (counted, no network). F5b: motive_cal.measured = true too, else motive_cal_check refuses the hardware runs before the reader runs. F7: motive_source = ''optitrack'' too, else the reader stays shut (emulator). {''emrun'', 1}: the runs keep EnableMotive = 1 instead of the forced 0'};
B(end+1,:) = {'mf_vi_noswitch', 'vi', {{'tool', 'bypass'}}, ...
    'arm-freeze switch removed (tools/wire_arm_freeze.m bypass on the copy): CommandGuard feeds ROMECommand directly; the banner refuses the hw_frz run'};
B(end+1,:) = {'mf_vi_neverfreeze', 'vi', {{'param', 'ArmFreezeSwitch', 'Threshold', '1.5'}, {'arm_freeze_banner.m', 'if ~isempty(prob)', 'if false'}}, ...
    'switch kept but Threshold 1.5 (never freezes), banner wiring refusal off in the copy: hw_frz runs and sends the moving arm'};
B(end+1,:) = {'mf_v_nostop', 'v', {{'ROMECommand.m', 'write(obj.DeviceHandle, uint8(char("STOP_ALL" + newline)));', '% STOP_ALL dropped (make-it-fail)'}}, ...
    'releaseImpl sends no STOP_ALL: the robot keeps its last command after the model stops'};
B(end+1,:) = {'mf_0_enable', '0', {{'ROMECommand.m', 'if Enable && obj.initial_run', 'if obj.initial_run'}}, ...
    'connect no longer gated on Enable (ROMECommand.m:85): the EnableHardware = 0 run opens the link and sends START_GV'};
B(end+1,:) = {'mf_stubdrift', 'stubdrift', {{'stubbyte', 'pause(0.5);', 'pause(0.6);'}}, ...
    'one byte of the stub''s non-transport code changed (pause 0.5 -> 0.6 s): the stub is no longer Keanu''s file; must raise test_hw_nocam:stubdrift'};
if strcmp(brk, 'list'), varargout{1} = B; return; end
ib = [];                                      % row of the break in B (empty: no break)
emRun = 0;                                    % EnableMotive in every run (flag): 0, the regression; a break may set 1
srcRun = 'emulator';                          % motive_source in every run (char); break mf_iv_motive sets 'optitrack' (F12: was a define_constants line edit)
if ~isempty(brk)
    ib = find(strcmp(B(:,1), brk));
    assert(numel(ib) == 1, 'test_hw_nocam:brk', 'unknown break ''%s''; test_hw_nocam(''list'') gives the table', brk);
    fprintf('MAKE-IT-FAIL break %s (check (%s) must FAIL): %s\n', B{ib,1}, B{ib,2}, B{ib,4});
end

% ---- 1. the stub is the real ROMECommand.m with only the transport constructor replaced ---------------------
realf = fullfile(root, 'ROMECommand.m');      % Keanu's class, never modified (char)
stubf = fullfile(stubdir, 'ROMECommand.m');   % recording stub (char)
assert(isfile(stubf), 'test_hw_nocam:nostub', 'recording stub %s does not exist', stubf);
if ~isempty(ib)
    % make-it-fail mf_stubdrift (fix round 1, I2): a drifted COPY of the stub, never the committed file
    for e = 1:numel(B{ib,3})
        ed = B{ib,3}{e};
        if strcmp(ed{1}, 'stubbyte')
            fid = fopen(stubf, 'r');  s = char(fread(fid, Inf, '*uint8').');  fclose(fid);   % stub bytes (char)
            n = numel(strfind(s, ed{2}));
            assert(n == 1, 'test_hw_nocam:brk', 'stub byte edit matches %d times (expected 1)', n);
            drifted = [tempname '_ROMECommand.m'];   % drifted copy of the stub in tempdir (char)
            fid = fopen(drifted, 'w');  fwrite(fid, uint8(strrep(s, ed{2}, ed{3})));  fclose(fid);
            cleanDrift = onCleanup(@() delete(drifted)); %#ok<NASGU>
            stubf = drifted;
            fprintf('  break edit applied to a temp copy of the stub: %s -> %s (%s)\n', ed{2}, ed{3}, drifted);
        end
    end
end
fid = fopen(realf, 'r');  realtxt = char(fread(fid, Inf, '*uint8').');  fclose(fid);   % real file bytes (char)
fid = fopen(stubf, 'r');  stubtxt = char(fread(fid, Inf, '*uint8').');  fclose(fid);   % stub bytes (char)
rep = {['classdef ROMECommand < matlab.System' crlf], ...
       ['classdef ROMECommand < matlab.System' crlf ...
        '    % RECORDING STUB (Plan 2 Task 1c): Keanu''s ROMECommand.m with ONLY the transport constructor' crlf ...
        '    % replaced (tcpclient -> RobotLinkRecorder, which opens no socket). tests/test_hw_nocam.m asserts' crlf ...
        '    % that this file equals ROMECommand.m with exactly these two replacements. Never on the MATLAB path.' crlf];
       'tcpclient(obj.ESP32_IP, obj.ESP32_Port, "ConnectTimeout", 5)', ...
       'RobotLinkRecorder(obj.ESP32_IP, obj.ESP32_Port, "ConnectTimeout", 5)'};   % {old, new} per row (char)
gen = realtxt;                                % the real file with the replacements applied (char)
for k = 1:size(rep, 1)
    n = numel(strfind(gen, rep{k,1}));        % matches of the old text (count), must be 1
    assert(n == 1, 'test_hw_nocam:stubdrift', 'replacement %d matches %d times in %s (expected 1): the real file changed', k, n, realf);
    gen = strrep(gen, rep{k,1}, rep{k,2});
end
if ~strcmp(gen, stubtxt)
    d = find(gen(1:min(end, numel(stubtxt))) ~= stubtxt(1:min(end, numel(gen))), 1);   % first differing byte (index)
    if isempty(d), d = min(numel(gen), numel(stubtxt)) + 1; end
    error('test_hw_nocam:stubdrift', 'stub %s differs from the real ROMECommand.m plus the transport replacements at byte %d (lengths %d / %d)', ...
          stubf, d, numel(stubtxt), numel(gen));
end
fprintf('stub identity: %s = %s with %d replacements, %d bytes, byte-identical\n', stubf, realf, size(rep, 1), numel(stubtxt));
assert(isempty(regexp(stubtxt, 'tcpclient\s*\(', 'once')), 'test_hw_nocam:network', 'the stub still calls tcpclient');

% ---- 2. limits and map from the user's define_constants.m (independent of any break) ------------------------
dc0 = fileread(fullfile(root, 'define_constants.m'));   % ROME_merged/define_constants.m text (char)
L = struct();                                 % user-block values: max_rpm (rpm), arm_* (deg or sign); arm_freeze is derived from run_mode since F12 (not read here, unused)
for nm = {'max_rpm', 'arm_sign', 'arm_offset_deg', 'arm_fw_switch', 'arm_fw_other', 'arm_fw_margin_deg'}
    tok = regexp(dc0, ['(?m)^' nm{1} '\s*=\s*([^;%]+);'], 'tokens');   % right-hand side of the one user-block line
    assert(numel(tok) == 1, 'test_hw_nocam:dc', '%s: %d user-block lines in define_constants.m (expected 1)', nm{1}, numel(tok));
    L.(nm{1}) = eval(tok{1}{1});
end
fwLo = min(L.arm_fw_switch, L.arm_fw_other) + L.arm_fw_margin_deg;   % 1x6 lowest joint command allowed (firmware deg)
fwHi = max(L.arm_fw_switch, L.arm_fw_other) - L.arm_fw_margin_deg;   % 1x6 highest joint command allowed (firmware deg)
fprintf('limits from %s: max_rpm %g rpm, joint windows lo %s hi %s deg (margin %g)\n', fullfile(root, 'define_constants.m'), ...
        L.max_rpm, mat2str(fwLo), mat2str(fwHi), L.arm_fw_margin_deg);

% ---- 3. temporary copy ------------------------------------------------------------------------------------
tmp = tempname;                               % temporary copy of ROME_merged (char)
startdir = pwd;                               % restored at exit (char)
path0 = path;                                 % MATLAB path at entry, restored at exit (char)
if bdIsLoaded(mdl), close_system(mdl, 0); end
mkdir(tmp);
copyfile(root, tmp);
copyfile(stubf, fullfile(tmp, 'ROMECommand.m'), 'f');
copyfile(fullfile(stubdir, 'RobotLinkRecorder.m'),  fullfile(tmp, 'RobotLinkRecorder.m'), 'f');
copyfile(fullfile(stubdir, 'MotiveDataCounting.m'), fullfile(tmp, 'MotiveDataCounting.m'), 'f');
mdlcell = {mdl};                              % model name as a cell, for the conditional close below
cleanup = onCleanup(@() cellfun(@(c) c(), {@() cd(startdir), @() path(path0), ...
          @() cellfun(@(m) close_system(m, 0), mdlcell(bdIsLoaded(mdl))), @() rmdir(tmp, 's')})); %#ok<NASGU>
addpath(tmp, fullfile(tmp, 'Ground Vehicle'), fullfile(tmp, 'Optitrack'), here);   % here: tests/dc_set_mode.m
cd(tmp);
evalin('base', 'clear all; clear classes');
recf = fullfile(tmp, 'robot_link_record.txt');   % RobotLinkRecorder's record file (char)
cntf = fullfile(tmp, 'motive_open_count.txt');   % MotiveDataCounting's openSocket counter file (char)

% no-network guard: nothing in the copy's link path can reach a socket, and the classes resolve in the copy
for f = {fullfile(tmp, 'ROMECommand.m'), fullfile(tmp, 'RobotLinkRecorder.m'), fullfile(tmp, 'MotiveDataCounting.m')}
    txt = fileread(f{1});                     % source text (char)
    bad = regexp(txt, '(tcpclient|MulticastSocket|DatagramSocket|serialport|serial\(|tcpip|udpport|natnet)\s*\(', 'match');
    assert(isempty(bad), 'test_hw_nocam:network', '%s contains a network call: %s', f{1}, strjoin(bad, ', '));
end
for c = {'ROMECommand', 'RobotLinkRecorder', 'MotiveDataCounting'}
    assert(strcmp(fileparts(which(c{1})), tmp), 'test_hw_nocam:stub', '%s resolves to %s, not the copy %s', c{1}, which(c{1}), tmp);
end
assert(strcmp(fileparts(which('MotiveData')), fullfile(tmp, 'Optitrack')), 'test_hw_nocam:stub', 'MotiveData resolves to %s', which('MotiveData'));
mc = meta.class.fromName('MotiveDataCounting');  % metadata of the counting subclass
for m = {'openSocket', 'receiveBytes'}
    mm = findobj(mc.MethodList, 'Name', m{1});   % method entries of that name
    assert(numel(mm) == 1 && strcmp(mm.DefiningClass.Name, 'MotiveDataCounting'), 'test_hw_nocam:stub', ...
           'MotiveDataCounting does not override %s', m{1});
end
assert(strcmp(mc.SuperclassList(1).Name, 'MotiveData'), 'test_hw_nocam:stub', 'MotiveDataCounting is not a MotiveData subclass');
fprintf('guard: ROMECommand -> %s\n       RobotLinkRecorder -> %s\n       MotiveDataCounting -> %s (overrides openSocket, receiveBytes of %s)\n', ...
        which('ROMECommand'), which('RobotLinkRecorder'), which('MotiveDataCounting'), which('MotiveData'));

% ---- 4. the break, applied to the copy only -------------------------------------------------------------------
if ~isempty(ib)
    edits = B{ib,3};                          % cell of edits (see the table)
    for e = 1:numel(edits)
        ed = edits{e};
        if strcmp(ed{1}, 'stubbyte'), continue; end   % applied in step 1
        if strcmp(ed{1}, 'emrun'), emRun = ed{2}; fprintf('  break: EnableMotive = %d in every run\n', emRun); continue; end   % read in step 5
        if strcmp(ed{1}, 'srcrun'), srcRun = ed{2}; fprintf('  break: motive_source = %s in every run\n', srcRun); continue; end   % read in step 5
        if strcmp(ed{1}, 'tool')
            addpath(fullfile(tmp, 'tools'));
            wire_arm_freeze(fullfile(tmp, [mdl '.slx']), ed{2});
        elseif strcmp(ed{1}, 'param')
            evalc('load_system(fullfile(tmp, [mdl ''.slx'']))');
            blk = [mdl '/' ed{2}];            % block in the copy's model (char)
            fprintf('  break: %s %s ''%s'' -> ''%s''\n', blk, ed{3}, get_param(blk, ed{3}), ed{4});
            set_param(blk, ed{3}, ed{4});
            save_system(mdl);
            close_system(mdl, 0);
        elseif strcmp(ed{1}, 'chart')
            load_system(fullfile(tmp, [mdl '.slx']));
            rt = sfroot;
            ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/' ed{2}]);   % the chart object
            assert(numel(ch) == 1, 'test_hw_nocam:brk', 'chart %s found %d times', ed{2}, numel(ch));
            s = ch.Script;                    % chart script (char)
            n = numel(strfind(s, ed{3}));
            assert(n == 1, 'test_hw_nocam:brk', 'break edit in chart %s matches %d times (expected 1)', ed{2}, n);
            ch.Script = strrep(s, ed{3}, ed{4});
            save_system(mdl);
            close_system(mdl, 0);
        else
            f = fullfile(tmp, ed{1});         % file in the copy (char)
            fid = fopen(f, 'r');  s = char(fread(fid, Inf, '*uint8').');  fclose(fid);
            n = numel(strfind(s, ed{2}));
            assert(n == 1, 'test_hw_nocam:brk', 'break edit in %s matches %d times (expected 1)', ed{1}, n);
            fid = fopen(f, 'w');  fwrite(fid, uint8(strrep(s, ed{2}, ed{3})));  fclose(fid);
        end
        fprintf('  break edit %d applied to the copy: %s\n', e, ed{1 + any(strcmp(ed{1}, {'chart', 'tool', 'param'}))});
    end
    evalin('base', 'clear all; clear classes');
    rehash;
end

% ---- 5. the four runs -----------------------------------------------------------------------------------------
dcf = fullfile(tmp, 'define_constants.m');    % the copy's define_constants.m (char)
fid = fopen(dcf, 'r');  dcBase = char(fread(fid, Inf, '*uint8').');  fclose(fid);   % after the break (char)
nSM = numel(strfind(dcBase, [lf 'sensor_model = 1;']));     % user-block sensor line (count), must be 1
assert(nSM == 1, 'test_hw_nocam:dc', 'copy define_constants.m: ''sensor_model = 1;'' x%d (expected 1)', nSM);
% F12: EnableHardware / EnableMotive / motive_source / arm_freeze come from run_mode; dc_set_mode count-asserts the
% run_mode line and adds TEST OVERRIDE lines for what no mode gives (EnableMotive = 0 is this test's regression path).
if isfile(cntf), delete(cntf); end
blkM = [mdl '/Enabled' newline 'Subsystem/MATLAB System'];   % the MotiveData block (SID 105)
rows = @(d, N) reshape(permute(d, [find(size(d) == N, 1, 'last'), setdiff(1:ndims(d), find(size(d) == N, 1, 'last'))]), N, []);   % logged Data -> N x n
cfg = {'ref', 0, false, 0; 'hw', 1, false, 0; 'hw_err', 1, true, 0; 'hw_frz', 1, false, 1};   % run name | EnableHardware | error injected at t_err | arm_freeze
R = cell(1, size(cfg, 1));                    % one struct per run
for r = 1:size(cfg, 1)
    nm = cfg{r,1};  EH = cfg{r,2};  inj = cfg{r,3};  AF = cfg{r,4};
    if ~EH, nRun = 1; elseif AF, nRun = 2; else, nRun = 3; end   % run mode with this EnableHardware and arm_freeze (fake camera)
    if strcmp(srcRun, 'optitrack') && EH && ~AF, nRun = 4; end
    txt = dc_set_mode(dcBase, nRun, struct('EnableHardware', EH, 'EnableMotive', emRun, 'motive_source', srcRun, 'arm_freeze', AF));
    if ~EH, txt = strrep(txt, [lf 'sensor_model = 1;'],  [lf 'sensor_model = 0;']); end   % (counted once above)
    fid = fopen(dcf, 'w');  fwrite(fid, uint8(txt));  fclose(fid);
    if isfile(recf), delete(recf); end
    assert(~isfile(recf), 'test_hw_nocam:rec', 'record file %s could not be cleared', recf);
    evalin('base', 'clear all');
    clear define_constants
    rehash;
    load_system(fullfile(tmp, [mdl '.slx']));   % PostLoadFcn = define_constants of the copy
    assert(isequal(evalin('base', 'EnableHardware'), EH), 'test_hw_nocam:dc', '%s: EnableHardware = %g after load, expected %d', nm, evalin('base', 'EnableHardware'), EH);
    % F12 fix round 1 (I5): the run mode and its TEST OVERRIDE lines landed as intended
    assert(isequal(evalin('base', 'EnableMotive'), emRun), 'test_hw_nocam:dc', '%s: EnableMotive = %s after load, expected %d', nm, mat2str(evalin('base', 'EnableMotive')), emRun);
    assert(isequal(evalin('base', 'arm_freeze_user'), AF), 'test_hw_nocam:dc', '%s: arm_freeze_user = %s after load, expected %d', nm, mat2str(evalin('base', 'arm_freeze_user')), AF);
    assert(strcmp(evalin('base', 'motive_source'), srcRun) && isequal(evalin('base', 'run_mode'), nRun), 'test_hw_nocam:dc', ...
           '%s: motive_source %s / run_mode %s after load, expected %s / %d', nm, evalin('base', 'motive_source'), mat2str(evalin('base', 'run_mode')), srcRun, nRun);
    sim0 = get_param(blkM, 'SimulateUsing');  % the real block's setting, 'Interpreted execution' (wiring log)
    set_param(blkM, 'System', 'MotiveDataCounting');
    set_param(blkM, 'SimulateUsing', sim0);   % changing System reset it to code generation (measured 19 Nov, mf_iv_motive rerun log)
    assert(strcmp(get_param(blkM, 'System'), 'MotiveDataCounting') && strcmp(get_param(blkM, 'SimulateUsing'), 'Interpreted execution'), ...
           'test_hw_nocam:stub', 'MotiveData block not switched to MotiveDataCounting in interpreted execution (System %s, SimulateUsing %s)', ...
           get_param(blkM, 'System'), get_param(blkM, 'SimulateUsing'));
    af0 = evalin('base', 'arm_freeze');       % arm_freeze as define_constants left it, before the run (flag)
    if inj
        load_system('simulink');
        set_param(mdl, 'AssertControl', 'EnableAll');
        add_block('simulink/Logic and Bit Operations/Compare To Constant', [mdl '/HwNocamErrAt'], 'relop', '<', 'const', num2str(t_err));
        add_block('simulink/Model Verification/Assertion', [mdl '/HwNocamInjectedError']);
        add_line(mdl, 'Clock9/1', 'HwNocamErrAt/1');
        add_line(mdl, 'HwNocamErrAt/1', 'HwNocamInjectedError/1');
    end
    evalin('base', 'clear ukd_current');
    err = '';  haveOut = false;  out = [];    % error text of sim (char), sim returned (logical), its output
    simtxt = '';                              % Command Window text of the sim, incl. the InitFcn banner (char)
    t0 = tic;
    try
        simtxt = evalc('out = sim(mdl, ''SimulationMode'', ''normal'', ''ReturnWorkspaceOutputs'', ''on'');');
        haveOut = true;
    catch ex
        err = [ex.identifier ': ' ex.message];
        for c1 = 1:numel(ex.cause)
            err = [err ' | ' ex.cause{c1}.message]; %#ok<AGROW>
            for c2 = 1:numel(ex.cause{c1}.cause), err = [err ' | ' ex.cause{c1}.cause{c2}.message]; end %#ok<AGROW>
        end
    end
    twEnd = double(java.lang.System.nanoTime()) * 1e-9;   % wall stamp right after sim() returned or threw, BEFORE close_system (s; same clock as the recorder)
    fprintf('%s', simtxt);
    S = struct('name', nm, 'EH', EH, 'wall', toc(t0), 'err', err, 'haveOut', haveOut, 'af0', af0, 'AF', AF, 'simtxt', simtxt, 'twEnd', twEnd);   % this run
    S.pac  = get_param(mdl, 'EnablePacing');  % pacing as the run left it (char)
    S.rate = double(get_param(mdl, 'PacingRate'));   % x
    for v = {'sensor_model', 'arm_freeze', 'q0_9dof', 'T_stop', 'dt_9dof', 'EnableMotive', 'arm_freeze_user', 'arm_freeze_deg', 'r', 'l', 'alphas'}
        try, S.(v{1}) = evalin('base', v{1}); catch, S.(v{1}) = []; end
    end
    close_system(mdl, 0);
    S.twClose = double(java.lang.System.nanoTime()) * 1e-9;   % wall stamp after close_system (s)
    S.K = {};  S.TS = zeros(0, 1);  S.TW = zeros(0, 1);  S.TX = {};   % record: kind C/W, sim time (s), wall (s), bytes (char)
    if isfile(recf)
        C = textscan(fileread(recf), '%s %f %f %s', 'Delimiter', '\t');
        S.K = C{1};  S.TS = C{2};  S.TW = C{3};
        S.TX = cellfun(@(h) char(sscanf(h, '%2x').'), C{4}, 'UniformOutput', false);
    end
    S.t = zeros(0, 1);  S.Q = [];  S.QM = [];  S.W = [];  S.J = [];  S.G = [];  S.JG = [];   % logs (s, rad/m, rpm, rad, flag, rad)
    if haveOut
        S.t = out.q_cmd.Time;  N = numel(S.t);
        S.Q = rows(out.q_cmd.Data, N);  S.QM = rows(out.q_meas.Data, N);
        S.W = rows(out.wheel_sent.Data, N);  S.J = rows(out.joint_sent_rad.Data, N);  S.G = rows(out.guard_flag.Data, N);
        if any(strcmp(who(out), 'joint_guard_rad')), S.JG = rows(out.joint_guard_rad.Data, N); end   % CommandGuard joint output (rad)
    end
    fprintf('run %-6s EnableHardware=%d: %d steps logged, %d records (%d writes), sim error: %s; wall %.2f s for T_stop %g s; EnablePacing %s rate %g; sensor_model %s; arm_freeze %s before, %s after; arm_freeze_user %s\n', ...
            nm, EH, numel(S.t), numel(S.K), sum(strcmp(S.K, 'W')), S.err, S.wall, S.T_stop, S.pac, S.rate, mat2str(S.sensor_model), mat2str(af0), mat2str(S.arm_freeze), mat2str(S.arm_freeze_user));
    R{r} = S;
    if strcmp(nm, 'hw') && isempty(ib)
        % record and logs of the paced hw run, for the pacing-deficit arithmetic
        % (docs/verification/2026-11-19_p2t1c_pacing_deficit.md); outside the repository, overwritten each run
        hwmat = fullfile(tempdir, 'test_hw_nocam_hw_record.mat');   % (char)
        save(hwmat, '-struct', 'S');
        fprintf('hw run record and logs saved to %s\n', hwmat);
    end
end
nOpen = 0;                                    % MotiveData openSocket calls over the four runs (count)
if isfile(cntf), nOpen = numel(strfind(fileread(cntf), 'openSocket')); end

% ---- 6. checks --------------------------------------------------------------------------------------------------
F = R{1};  H = R{2};  E = R{3};  Z = R{4};     % ref, hw, hw_err, hw_frz
START = ['START_GV' lf];  STOP = ['STOP_ALL' lf];   % framing lines (char)
% (0) the EnableHardware = 0 run connected to nothing and sent nothing
c0 = isempty(F.K) && F.haveOut;
fprintf('\n(0)   ref (EnableHardware = 0): %d link records, sim returned %d -> %s\n', numel(F.K), F.haveOut, mat2str(c0));

isW = strcmp(H.K, 'W');                       % hw records that are writes (logical)
ln = H.TX(isW);  ts = H.TS(isW);  tw = H.TW(isW);   % hw lines (char), their sim time (s) and wall stamp (s)
isRome = startsWith(ln, 'ROME,');             % ROME command lines (logical)
oneLine = all(cellfun(@(s) ~isempty(s) && s(end) == lf && sum(s == lf) == 1, ln));   % every write is one line
fmtOk = all(~cellfun(@isempty, regexp(ln(isRome), '^ROME(,-?\d+\.\d{3}){10}\n$', 'once')));   % ROME,w1..w4,j1..j6 at %.3f
V = zeros(0, 10);                             % parsed ROME fields: wheels (rpm) 1-4, joints (firmware deg) 5-10
if fmtOk && any(isRome)
    V = cell2mat(cellfun(@(s) sscanf(s, 'ROME,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f').', ln(isRome), 'UniformOutput', false));
end
nR = size(V, 1);                              % ROME lines sent (count)
tR = ts(isRome);                              % their simulation times (s)
twR = tw(isRome);                             % their wall stamps (s)

% (i) step-by-step equality with the EnableHardware = 0 run, within the print precision
clampf = @(x) min(max(x, fwLo), fwHi);        % firmware window, the arm_clamp formula (firmware deg)
dW0 = Inf;  dJ0 = Inf;  dW1 = Inf;  dJ1 = Inf;  dT = Inf;  nExact = 0;   % max field differences (rpm, deg), max time offset (s)
sameQ = F.haveOut && H.haveOut && isequal(F.Q, H.Q);   % q_cmd of hw bit-identical to ref (logical)
countOk = nR > 0 && nR == numel(F.t) && nR == numel(H.t);
if countOk
    dT  = max(abs(tR - H.t));
    Jx0 = clampf(L.arm_sign .* rad2deg(F.J) + L.arm_offset_deg);   % ref joints through map and clamp (firmware deg)
    Jx1 = clampf(L.arm_sign .* rad2deg(H.J) + L.arm_offset_deg);   % hw's own logged joints, same path (firmware deg)
    dW0 = max(abs(V(:,1:4) - F.W), [], 'all');  dJ0 = max(abs(V(:,5:10) - Jx0), [], 'all');
    dW1 = max(abs(V(:,1:4) - H.W), [], 'all');  dJ1 = max(abs(V(:,5:10) - Jx1), [], 'all');
    expTxt = arrayfun(@(k) sprintf('ROME,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', F.W(k,:), Jx0(k,:)), (1:nR).', 'UniformOutput', false);
    nExact = sum(strcmp(expTxt, ln(isRome)));
end
ci = countOk && fmtOk && sameQ && dT <= 1e-9 && dW0 <= tolField && dJ0 <= tolField && dW1 <= tolField && dJ1 <= tolField;
fprintf('(i)   %d ROME lines, %d / %d logged steps (hw / ref), format ok %d, line times vs logged max %.3g s; q_cmd hw == ref bit-identical %d\n', ...
        nR, numel(H.t), numel(F.t), fmtOk, dT, sameQ);
fprintf('      max |sent - ref| wheels %.3g rpm, joints %.3g deg; vs hw''s own log wheels %.3g, joints %.3g; tolerance %.4g (half of 0.001); %d / %d lines string-identical to the ref -> %s\n', ...
        dW0, dJ0, dW1, dJ1, tolField, nExact, nR, mat2str(ci));

% (ii) pacing
ratio = NaN;  spanSim = NaN;  spanWall = NaN; % wall/simulated span of the ROME lines (-), spans (s)
if nR > 1
    spanSim = tR(end) - tR(1);  spanWall = twR(end) - twR(1);  ratio = spanWall / spanSim;
end
cii = nR > 1 && ratio >= ratioMin && strcmp(H.pac, 'on') && H.rate == 1;
fprintf('(ii)  ROME lines span %.3f s simulated, %.3f s wall (ratio %.3f, need >= %.2f); whole sim() %.2f s wall for T_stop %g s; EnablePacing %s rate %g -> %s\n', ...
        spanSim, spanWall, ratio, ratioMin, H.wall, H.T_stop, H.pac, H.rate, mat2str(cii));
if nR > 1
    % Diagnostic only (not part of the verdict): how the wall clock follows the simulated clock line by line.
    lag = (twR - twR(1)) - (tR - tR(1));      % wall minus simulated time elapsed since the first ROME line (s); < 0 = ahead of real time
    [lagMin, kMin] = min(lag);                % most ahead (s) and at which line (index)
    dtw = diff(twR);                          % wall interval between consecutive ROME lines (s)
    dts = diff(tR);                           % simulated interval between them (s)
    kFast = find(dtw < 0.5 * dts);            % intervals sent at more than twice the real-time rate (index)
    fprintf(['      per line: wall-minus-simulated elapsed time min %.3f s at t = %.2f s, end %.3f s; wall interval median %.4f s ' ...
             '(simulated %.4f s), min %.4f s; %d of %d intervals shorter than half the step (first at t = %s s, last at t = %s s)\n'], ...
            lagMin, tR(kMin), lag(end), median(dtw), median(dts), min(dtw), numel(kFast), numel(dtw), ...
            mat2str(tR(kFast(1:min(1, end))) , 4), mat2str(tR(kFast(max(1, end):end)), 4));
    fprintf('      first ROME line %.3f s wall after the connect record; wall span of the first 2 s simulated: %.3f s\n', ...
            twR(1) - H.TW(find(strcmp(H.K, 'C'), 1)), twR(find(tR <= tR(1) + 2, 1, 'last')) - twR(1));
end

% (iii) limits on every sent value
wPeak = NaN;  jRoom = NaN;                    % largest |wheel| sent (rpm), least room to a joint window end (deg)
if nR > 0
    wPeak = max(abs(V(:,1:4)), [], 'all');
    jRoom = min(min(V(:,5:10) - fwLo, [], 'all'), min(fwHi - V(:,5:10), [], 'all'));
end
ciii = nR > 0 && wPeak <= L.max_rpm && jRoom >= 0;
fprintf('(iii) largest |wheel| sent %.3f rpm (max_rpm %g, headroom %.3f); least room inside the joint windows %.3f deg -> %s\n', ...
        wPeak, L.max_rpm, L.max_rpm - wPeak, jRoom, mat2str(ciii));
if nR > 0
    fprintf('      per joint: lowest sent %s, highest sent %s deg\n', mat2str(min(V(:,5:10), [], 1), 6), mat2str(max(V(:,5:10), [], 1), 6));
end

% (iv) no sensor noise and no MotiveData socket
smZero = isequal(H.sensor_model, 0);          % workspace sensor_model of the hw run (logical)
qm0 = H.haveOut && ~isempty(H.QM) && isequal(H.QM(1,:), H.q0_9dof(:).');            % q_meas(0) = q0_9dof
qmd = H.haveOut && size(H.QM, 1) > 1 && isequal(H.QM(2:end,:), H.Q(1:end-1,:));      % q_meas = q_cmd one step late
dQM = NaN;                                    % max |q_meas(k) - q_cmd(k-1)| (m, rad)
if H.haveOut && size(H.QM, 1) > 1, dQM = max(abs(H.QM(2:end,:) - H.Q(1:end-1,:)), [], 'all'); end
civ = smZero && qm0 && qmd && nOpen == 0;
fprintf('(iv)  sensor_model %s; q_meas(0) == q0_9dof %d; q_meas(k) == q_cmd(k-1) exactly %d (max diff %.3g); MotiveData openSocket calls over 4 runs %d -> %s\n', ...
        mat2str(H.sensor_model), qm0, qmd, dQM, nOpen, mat2str(civ));

% (v) STOP_ALL last, at a normal end and at an error stop
nStart = sum(strcmp(ln, START));  nStop = sum(strcmp(ln, STOP));   % framing lines in hw (count)
other = sum(~isRome & ~strcmp(ln, START) & ~strcmp(ln, STOP));     % unexpected lines in hw (count)
cvN = ~isempty(ln) && oneLine && nStart == 1 && strcmp(ln{1}, START) && nStop == 1 && strcmp(ln{end}, STOP) && other == 0 && nR > 0;
eW = strcmp(E.K, 'W');  lnE = E.TX(eW);  tsE = E.TS(eW);           % hw_err lines (char) and sim times (s)
eR = startsWith(lnE, 'ROME,');                % hw_err ROME lines (logical)
injOk = contains(E.err, 'HwNocamInjectedError');   % hw_err ended in the injected assertion (logical)
lastRomeE = NaN;  if any(eR), lastRomeE = max(tsE(eR)); end        % last ROME line of hw_err (s)
cvE = injOk && ~E.haveOut && any(eR) && lastRomeE <= t_err + 1e-9 && sum(strcmp(lnE, STOP)) == 1 && strcmp(lnE{end}, STOP) ...
      && sum(strcmp(lnE, START)) == 1 && strcmp(lnE{1}, START) && all(cellfun(@(s) s(end) == lf && sum(s == lf) == 1, lnE));
% WHEN STOP_ALL is sent (fix round 1, I1): at simulation termination, i.e. recorded before sim() returned or threw
% (twEnd, taken before close_system), with the simulation time of the final step (hw: the last logged step;
% hw_err: t_err). A STOP_ALL sent only when the model is closed would come after twEnd: on the robot the model stays
% open after a run, so the last ROME command would be held until then.
iStH = find(strcmp(H.K, 'W') & strcmp(H.TX, STOP), 1, 'last');    % hw record index of STOP_ALL (index)
iStE = find(strcmp(E.K, 'W') & strcmp(E.TX, STOP), 1, 'last');    % hw_err record index of STOP_ALL (index)
stWH = NaN;  stSH = NaN;  stWE = NaN;  stSE = NaN;                  % STOP_ALL wall stamp (s) and sim time (s), hw / hw_err
if ~isempty(iStH), stWH = H.TW(iStH);  stSH = H.TS(iStH); end
if ~isempty(iStE), stWE = E.TW(iStE);  stSE = E.TS(iStE); end
tFinH = NaN;  if ~isempty(H.t), tFinH = H.t(end); end              % final logged step of hw (s)
holdH = NaN;  if nR > 0, holdH = stWH - twR(end); end              % wall time the last ROME line was the robot's command (s)
lastWE = NaN;  if any(eR), lastWE = max(E.TW(strcmp(E.K, 'W') & startsWith(E.TX, 'ROME,'))); end   % wall stamp of hw_err's last ROME line (s)
cvTN = stWH < H.twEnd && abs(stSH - tFinH) <= 1e-9;                % hw: STOP_ALL at termination, final step (logical)
cvTE = stWE < E.twEnd && abs(stSE - t_err) <= 1e-9;                % hw_err: same at the error step (logical)
cv = cvN && cvE && cvTN && cvTE;
fprintf(['      STOP_ALL timing, normal end: sim time %.4f s (final logged step %.4f s), wall %.4f s BEFORE sim() returned ' ...
         '(%.4f s before close_system ended); last ROME line held %.4f s wall -> %s\n'], ...
        stSH, tFinH, H.twEnd - stWH, H.twClose - stWH, holdH, mat2str(cvTN));
fprintf(['      STOP_ALL timing, error stop: sim time %.4f s (t_err %g s), wall %.4f s BEFORE sim() threw ' ...
         '(%.4f s before close_system ended); last ROME line held %.4f s wall -> %s\n'], ...
        stSE, t_err, E.twEnd - stWE, E.twClose - stWE, stWE - lastWE, mat2str(cvTE));
firstTxt = '<none>';  lastTxt = '<none>';     % first and last line sent in hw, without the LF (char)
if ~isempty(ln), firstTxt = strtrim(ln{1});  lastTxt = strtrim(ln{end}); end
posStop = find(strcmp(ln, STOP), 1, 'last');  % position of the (last) STOP_ALL among the hw writes (index), 0 if none
if isempty(posStop), posStop = 0; end
fprintf('(v)   normal end: %d writes, first %s, last %s (START_GV x%d, STOP_ALL x%d at write %d of %d, other lines %d, one line per write %d) -> %s\n', ...
        numel(ln), firstTxt, lastTxt(1:min(end, 60)), nStart, nStop, posStop, numel(ln), other, oneLine, mat2str(cvN));
if isempty(lnE)
    fprintf('      error stop: no writes recorded; sim error: %s -> false\n', E.err);
else
    fprintf('      error stop: sim error from the injected block %d (%s); %d ROME lines, last at t = %.3f s (t_err %g); last write %s (STOP_ALL x%d) -> %s\n', ...
            injOk, E.err(1:min(end, 160)), sum(eR), lastRomeE, t_err, strtrim(lnE{end}), sum(strcmp(lnE, STOP)), mat2str(cvE));
end

% (vi) arm_freeze = 1 holds the arm at arm_freeze_deg on the robot path (Plan 2 Task 5 / focus plan F1)
zW = strcmp(Z.K, 'W');  lnZ = Z.TX(zW);       % hw_frz lines (char)
zR = startsWith(lnZ, 'ROME,');                % its ROME lines (logical)
fmtZ = all(~cellfun(@isempty, regexp(lnZ(zR), '^ROME(,-?\d+\.\d{3}){10}\n$', 'once')));
VZ = zeros(0, 10);                            % parsed hw_frz ROME fields: wheels (rpm) 1-4, joints (firmware deg) 5-10
if fmtZ && any(zR)
    VZ = cell2mat(cellfun(@(s) sscanf(s, 'ROME,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f').', lnZ(zR), 'UniformOutput', false));
end
nZ = size(VZ, 1);                             % ROME lines sent in hw_frz (count)
afd = NaN(1, 6);  if numel(Z.arm_freeze_deg) == 6, afd = Z.arm_freeze_deg(:).'; end   % hw_frz workspace arm_freeze_deg (firmware deg)
orZ = NaN(1, 6);  if numel(Z.q0_9dof) == 9, orZ = clampf(L.arm_sign .* rad2deg(Z.q0_9dof(4:9)).' + L.arm_offset_deg); end   % oracle (firmware deg)
dOrZ = max(abs(afd - orZ));                   % arm_freeze_deg vs the oracle (deg)
dFz = Inf;  if nZ > 0, dFz = max(abs(VZ(:,5:10) - afd), [], 'all'); end   % max |joint field - arm_freeze_deg| (deg)
mvZ = NaN;  if size(Z.JG, 2) == 6, mvZ = max(max(rad2deg(Z.JG), [], 1) - min(rad2deg(Z.JG), [], 1)); end   % controller joint command range (deg)
whlSame = nZ > 0 && nZ == nR && isequal(VZ(:,1:4), V(:,1:4));   % wheel fields of hw_frz == hw (logical)
frameZ = ~isempty(lnZ) && strcmp(lnZ{1}, START) && strcmp(lnZ{end}, STOP) && sum(strcmp(lnZ, START)) == 1 && sum(strcmp(lnZ, STOP)) == 1;
banZ = [numel(strfind(Z.simtxt, 'ARM FROZEN at arm_freeze_deg')), numel(strfind(Z.simtxt, 'ARM WILL MOVE'))];   % banner counts in hw_frz
banH = [numel(strfind(H.simtxt, 'ARM FROZEN at arm_freeze_deg')), numel(strfind(H.simtxt, 'ARM WILL MOVE'))];   % banner counts in hw
cvi = Z.haveOut && fmtZ && nZ > 0 && nZ == numel(Z.t) && dFz <= tolField && dOrZ <= 1e-9 && mvZ >= 10 && whlSame && frameZ ...
      && isequal(banZ, [1 0]) && isequal(banH, [0 1]);
jMoveH = NaN(1, 6);  if nR > 0, jMoveH = max(V(:,5:10), [], 1) - min(V(:,5:10), [], 1); end   % hw (arm_freeze = 0) joint field ranges (deg)
jMoveZ = NaN(1, 6);  if nZ > 0, jMoveZ = max(VZ(:,5:10), [], 1) - min(VZ(:,5:10), [], 1); end  % hw_frz joint field ranges (deg)
fprintf('(vi)  hw_frz (arm_freeze = 1, arm_freeze_user %s; workspace arm_freeze %s before, %s after): %d ROME lines for %d logged steps, format ok %d; max |joint field - arm_freeze_deg| %.3g deg (tol %.4g); |arm_freeze_deg - oracle| %.3g deg\n', ...
        mat2str(Z.arm_freeze_user), mat2str(Z.af0), mat2str(Z.arm_freeze), nZ, numel(Z.t), fmtZ, dFz, tolField, dOrZ);
fprintf('      controller joint command (joint_guard_rad) moved %.2f deg (need >= 10); wheel fields == hw %d; START_GV first / STOP_ALL last %d; banner [FROZEN MOVE] hw_frz %s (want [1 0]), hw %s (want [0 1]) -> %s\n', ...
        mvZ, whlSame, frameZ, mat2str(banZ), mat2str(banH), mat2str(cvi));
fprintf('      sent joint field ranges: hw (arm_freeze = 0) %s deg; hw_frz %s deg; arm_freeze_deg %s\n', mat2str(jMoveH, 4), mat2str(jMoveZ, 4), mat2str(afd, 6));

names = {'0', 'i', 'ii', 'iii', 'iv', 'v', 'vi'};   % check ids
pass  = [c0, ci, cii, ciii, civ, cv, cvi];    % verdict per check (logical)
lab   = {'FAIL', 'PASS'};                     % verdict labels, indexed by 1 + logical
fprintf('\nsummary: %s\n', strjoin(cellfun(@(n, p) sprintf('(%s) %s', n, lab{1 + p}), names, num2cell(pass), 'UniformOutput', false), ', '));
if ~all(pass)
    tag = '';                                 % the break, if any (char)
    if ~isempty(ib), tag = sprintf(' [break %s]', B{ib,1}); end
    error('test_hw_nocam:fail', 'test_hw_nocam FAILED checks: %s%s [returned: ref %d, hw %d, hw_err %d, hw_frz %d]', ...
          strjoin(names(~pass), ', '), tag, F.haveOut, H.haveOut, E.haveOut, Z.haveOut);
end
fprintf('\ntest_hw_nocam PASS: checks (0) (i) (ii) (iii) (iv) (v) (vi)\n');
end
