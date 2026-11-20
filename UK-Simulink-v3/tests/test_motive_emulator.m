function varargout = test_motive_emulator(brk)
%TEST_MOTIVE_EMULATOR  The camera emulator ("fake camera", focus plan F7) in ROME_9DOF.slx, in simulation and against a fake robot.
%   Modes (author, 19 Nov 2026): 1 sim (EnableHardware 0, EnableMotive 1, 'emulator'); 2 robot, arm frozen (EnableHardware 1,
%   arm_freeze 1, 'emulator'); 3 robot, arm moving (arm_freeze 0, 'emulator'); 4 = 3 with 'optitrack'. EnableMotive 0 is a
%   regression setting only (h1, s0, s3).
%   Model under test (tools/wire_motive_emulator.m): root MotiveEmulator (Optitrack/motive_emulator_block.m:
%   floor_to_motive of the StateDelay base; MocapNoise only with motive_emulate_noise AND sensor_model) -> Enabled
%   Subsystem input 2 -> PoseSource switch (PoseSelect = motive_source_check(motive_source)) -> MotiveFloor (F5b:
%   motive_to_floor + yaw_unwrap) -> tracked_position -> MuxTracked in 1. MuxTracked in 2 = JointSource: TelemetryDelay
%   (arm telemetry, IC q0_9dof(4:9)) on the robot, SensorModel joints in simulation. Reader gate MotiveReaderEnable =
%   EnableMotive && EnableHardware && 'optitrack'.
%
%   NO NETWORK, EVER. A TEMPORARY COPY of ROME_merged in which
%     ROMECommand.m        is GENERATED here from Keanu's ROMECommand.m with ONLY tcpclient(...) replaced by
%                          RobotLinkEcho(...) (one match asserted), so the real telemetry parser runs;
%     RobotLinkEcho.m      (tests/stubs_hw) the recording fake robot (RobotLinkRecorder) + a fake arm-telemetry source:
%                          after each ROME line one "ARM,<6 firmware deg>,READY" line, either replayed from
%                          robot_link_replay.txt (written here) or echoing the line's own joint fields;
%     MotiveDataCounting.m the real MotiveData with openSocket counted (file) and no datagram; the copy's MotiveData
%                          block is pointed at it in memory in every run.
%   The copy's define_constants.m is edited per run (flag lines, each edit asserted to match once); ROME_merged is never
%   written. The state the controller reads (StateSource output = UKDynamics in 1) is logged by a To Workspace added in
%   memory (state_in).
%
%   Runs (scenario 1, full T_stop unless stated):
%     s0  sim, EnableMotive 0, sensor_model 0                    s1  sim, EnableMotive 1 'emulator', sensor_model 0
%     s3  sim, EnableMotive 0, sensor_model 1                    s4  sim, EnableMotive 1 'emulator', sensor_model 1,
%     s2  sim, EnableMotive 1 'emulator', sensor_model 1,             motive_emulate_noise 1
%         motive_emulate_noise 0 (reported only)
%     h1  robot, EnableMotive 0 (regression, the pre-F7 loop): EnableHardware 1, arm_freeze 0, fake robot (echo)
%     h2  robot, mode 3: EnableHardware 1, EnableMotive 1, 'emulator', arm_freeze 0, motive_cal.measured false (the
%         file's), motive_emulate_noise 1 (no noise may reach the robot run: sensor_model is forced 0), fake arm telemetry REPLAYING h1's commanded joints q_cmd(4:9) at %.17g (an arm that reaches exactly
%         what h1 commanded)
%     h3  robot, mode 2 (arm_freeze 1), fake arm telemetry ECHOING the parked posture: runs to the end (author 19 Nov:
%         "Arm from model if frozen")
%     h4  robot, mode 3, arm_freeze 0, fake arm telemetry ECHOING each line's %.3f joint fields
%     r1 r2 fr1 fr2  sim, EnableMotive 1 'emulator', sensor_model 0, StopTime 2 s, the emulated pose replaced IN MEMORY
%         by a sweep (F7Sweep: floor_to_motive([q0(1:2); q0(3) + 2 pi 1.25 t / 2 s])), on ONE loaded model: r1, r2
%         normal; fr1, fr2 with Fast Restart on, which apply_pacing must refuse (EnableMotive ~= 0, F7 fix round 1).
%   Checks (all evaluated, then one error listing the failed ones):
%     (i)   exact plumbing, per step, in s1: state_in(1) = q0_9dof and state_in(k) base = q_cmd(k-1) base (1e-12); the
%           state's base equals q_track exactly, its joints equal SensorModel's (q_meas) exactly, openSocket 0 calls.
%           max |q_cmd(s1) - q_cmd(s0)| over the run <= 1e-9 (backstop, controller ruling Q-F7-c; printed at step 1,
%           t/4, end): the heading round trip is exact to one ulp, which the closed loop amplifies (7.7e-11 over 621
%           steps, first run 19 Nov 2026), so a whole-run 1e-12 is not met.
%     (ii)  paper noise: in s4 the state's base equals SensorModel's noisy base q_meas(1:3) per step (1e-12; the same
%           MocapNoise draws) and its joints q_meas joints exactly. Table 3 through the emulator: test_suite steps 7-8
%           (run_9dof_cases runs mode 1, the defaults, and prints the flags; test_v3_table3), not re-run here.
%           s4 vs s3 whole run <= 1e-9 (backstop); s2 (noise-free emulator) vs s3 is printed only.
%     (iii) robot modes 2 and 3 against the fake robot:
%           a  openSocket called 0 times over h1, h2, h3, h4 (MotiveDataCounting counter);
%           b  h2 returned (motive_cal.measured false is NOT refused with 'emulator'); state_in(k) base = q_cmd(k-1)
%              base and state_in(1) = q0_9dof (1e-12); state_in(k) joints = decode of the telemetry answered at step
%              k-1 = ((fw - arm_offset_deg) ./ arm_sign) pi/180 (1e-12), state_in(1) joints = q0_9dof(4:9);
%           c  h2's ROME lines equal h1's (EnableMotive 0 stream), same count, every field within half a unit of the %.3f
%              print (test_hw_nocam tolerance); string-identical count and first difference printed;
%           d  h2's ROME lines equal h2's own wheel_sent / joint_sent_rad through map and clamp, half a unit;
%           e  pacing: h2 wall span of the ROME lines >= 0.9 x simulated span, EnablePacing on at PacingRate 1;
%           f  h2 and h4: START_GV first and STOP_ALL last, once each, STOP_ALL before sim() returned at the final
%              step's simulation time;
%           g  h3 (mode 2, frozen) runs to the end; its controller reads the MODEL's joints (state_in joints = q_meas
%              joints exactly, JointSource on SimJoints because arm_freeze_user = 1), so q_cmd(h3) = q_cmd(s1) within 1e-9
%              (bounded; the joint-command range is printed), every joint field = arm_freeze_deg (half a unit), banner
%              'ARM FROZEN' once. History: before the author's ruling the controller read the parked telemetry here and
%              its joint commands grew to 1657 deg (run-1 log, `V/2026-11-19_f7_test_motive_emulator_run1.log`); h4:
%              state_in(k) joints = decode of the echo at k-1 (1e-12) while differing from q_cmd(k-1) joints by
%              > 1e-9 rad somewhere (the %.3f print: the joints come from telemetry, not from StateDelay); banner
%              'ARM WILL MOVE' in h2. Peak wheel is PRINTED (wheel <= max_rpm cannot fail: CommandGuard enforces it).
%     (r)   unwrap state reset (F5b review I2): r1's q_track heading sweeps >= 2 pi past q0(3) (premise); normal
%           restart: r2 starts at q0(3) (1e-12) and equals r1 at every step (1e-12); Fast Restart: fr1 and fr2 are both
%           refused with apply_pacing:fastrestartcam and return nothing (ruling: Fast Restart refused whenever
%           EnableMotive ~= 0). SimulationMode is set once on the model (Fast Restart rejects it as a sim() argument);
%           ROMECommand is commented out in memory for these runs (it errors on a second Fast Restart run).
%   MAKE-IT-FAIL: test_motive_emulator('<id>') applies one break to the COPY and must then error naming its check;
%   test_motive_emulator('list') returns the table.
%   Flat function without local functions (CLAUDE.md section 5, crash 2). Clears the base workspace.
if nargin < 1 || isempty(brk), brk = ''; end
here = fileparts(mfilename('fullpath'));      % ROME_merged/tests (char)
root = fileparts(here);                       % ROME_merged (char)
mdl  = 'ROME_9DOF';                           % model name (char)
lf   = char(10);                              % line feed (char)
crlf = char([13 10]);                         % CR LF (char)
tolQ = 1e-12;                                 % exact-plumbing tolerance, per step (m, rad)
tolRun = 1e-9;                                % whole-run backstop on |q_cmd(emulator) - q_cmd(EnableMotive 0)| (m, rad); controller ruling Q-F7-c, 19 Nov 2026 (measured 7.7e-11 / 1.5e-11)
tolField = 5e-4 + 1e-9;                       % half a unit of the %.3f print plus float slack (rpm, deg)
ratioMin = 0.9;                               % pacing: wall span >= ratioMin x simulated span (-)
Tsw  = 2.0;                                   % StopTime of the reset runs (s)
B = { ...
  'mf_const', 'i',   {{'constpose'}}, 'emulator wired to a constant pose: Constant q0_9dof -> MotiveEmulator input 1 instead of StateDelay'; ...
  'mf_jointeh', 'iii', {{'param', 'JointFromTelemetry', 'Value', 'double(EnableHardware ~= 0)'}}, 'JointSource on EnableHardware only: a frozen arm (mode 2) feeds the parked telemetry to the controller again (wind-up)'; ...
  'mf_gate',  'iii', {{'param', 'MotiveReaderEnable', 'Value', 'double(EnableMotive ~= 0 && EnableHardware ~= 0)'}}, 'reader gate ignores motive_source (the F5b expression): mode 2 on the robot opens the socket'; ...
  'mf_frcam', 'r',   {{'file', 'apply_pacing.m', 'if camOn && bdIsLoaded(mdl)', 'if false && camOn && bdIsLoaded(mdl)'}}, 'apply_pacing no longer refuses Fast Restart with EnableMotive ~= 0'; ...
  'mf_carry', 'r',   {{'chart', 'MotiveFloor', ['persistent psi_prev' lf], ['persistent psi_prev' lf 'coder.extrinsic(''evalin'', ''setappdata'');' lf]}, ...
                      {'chart', 'MotiveFloor', 'psi_prev = q0_9dof(3);', 'psi_prev = q0_9dof(3); cv = 0; cv = evalin(''base'', ''max([NaN getappdata(0, ''''f7_carry'''')])''); if isfinite(cv), psi_prev = cv; end'}, ...
                      {'chart', 'MotiveFloor', ['    psi_prev = psi;' lf], ['    psi_prev = psi; setappdata(0, ''f7_carry'', psi);' lf]}}, ...
              'unwrap state carried from one run to the next (stored in root appdata, as a persistent that is never reset would)'; ...
  'mf_tolrun', 'i', {{'tolrun', 1e-12}}, 'whole-run backstop tightened to 1e-12 (below the measured 7.7e-11): shows the backstop of (i) and (ii) can fail (F7 re-review)'; ...
  'mf_h4tel', 'iii', {{'param', 'JointFromTelemetry', 'Value', '0'}}, 'JointSource never takes telemetry: in h4 (mode 3, arm moving) the controller reads the model''s joints instead of the arm telemetry (F7 re-review)'; ...
  'mf_noisegate', 'iii', {{'chart', 'MotiveEmulator', 'if motive_emulate_noise ~= 0 && sensor_model ~= 0', 'if motive_emulate_noise ~= 0'}}, 'emulator noise gate without its sensor_model term: h2 (robot, motive_emulate_noise 1) gets synthetic camera noise (F7 re-review)'};
if strcmp(brk, 'list'), varargout{1} = B; return; end
ib = [];                                      % row of the break in B (empty: no break)
if ~isempty(brk)
    ib = find(strcmp(B(:,1), brk));
    assert(numel(ib) == 1, 'test_motive_emulator:brk', 'unknown break ''%s''; test_motive_emulator(''list'') gives the table', brk);
    fprintf('MAKE-IT-FAIL break %s (check (%s) must FAIL): %s\n', B{ib,1}, B{ib,2}, B{ib,4});
end
if isappdata(0, 'f7_carry'), rmappdata(0, 'f7_carry'); end

% ---- limits and map from the user's define_constants.m (independent of any break) --------------------------------
dc0 = fileread(fullfile(root, 'define_constants.m'));   % ROME_merged/define_constants.m text (char)
L = struct();                                 % user-block values (rpm, deg, sign, flag)
for nm = {'max_rpm', 'arm_sign', 'arm_offset_deg', 'arm_fw_switch', 'arm_fw_other', 'arm_fw_margin_deg'}
    tok = regexp(dc0, ['(?m)^' nm{1} '\s*=\s*([^;%]+);'], 'tokens');
    assert(numel(tok) == 1, 'test_motive_emulator:dc', '%s: %d user-block lines in define_constants.m (expected 1)', nm{1}, numel(tok));
    L.(nm{1}) = eval(tok{1}{1});
end
fwLo = min(L.arm_fw_switch, L.arm_fw_other) + L.arm_fw_margin_deg;   % 1x6 lowest joint command allowed (firmware deg)
fwHi = max(L.arm_fw_switch, L.arm_fw_other) - L.arm_fw_margin_deg;   % 1x6 highest (firmware deg)
clampf = @(x) min(max(x, fwLo), fwHi);        % firmware window, the arm_clamp formula (firmware deg)
decode = @(fw) ((fw - L.arm_offset_deg) ./ L.arm_sign) * (pi/180);   % ROMECommand telemetry decode then ArmDeg2Rad (rad), rows

% ---- temporary copy, echo stub generated from the real ROMECommand.m -----------------------------------------------
tmp = tempname;                               % temporary copy of ROME_merged (char)
startdir = pwd;  path0 = path;                % restored at exit
if bdIsLoaded(mdl), close_system(mdl, 0); end
mkdir(tmp);
copyfile(root, tmp);
realtxt = fileread(fullfile(root, 'ROMECommand.m'));   % Keanu's class (char)
old = 'tcpclient(obj.ESP32_IP, obj.ESP32_Port, "ConnectTimeout", 5)';
n = numel(strfind(realtxt, old));
assert(n == 1, 'test_motive_emulator:stub', 'transport constructor matches %d times in ROMECommand.m (expected 1)', n);
fid = fopen(fullfile(tmp, 'ROMECommand.m'), 'w');
fwrite(fid, uint8(strrep(realtxt, old, 'RobotLinkEcho(obj.ESP32_IP, obj.ESP32_Port, "ConnectTimeout", 5)')));  fclose(fid);
for f = {'RobotLinkRecorder.m', 'RobotLinkEcho.m', 'MotiveDataCounting.m'}
    copyfile(fullfile(here, 'stubs_hw', f{1}), fullfile(tmp, f{1}), 'f');
end
mdlcell = {mdl};
cleanup = onCleanup(@() cellfun(@(c) c(), {@() cd(startdir), @() path(path0), ...
          @() cellfun(@(m) close_system(m, 0), mdlcell(bdIsLoaded(mdl))), @() rmdir(tmp, 's')})); %#ok<NASGU>
addpath(tmp, fullfile(tmp, 'Ground Vehicle'), fullfile(tmp, 'Optitrack'), fullfile(tmp, 'tests'));
cd(tmp);
evalin('base', 'clear all; clear classes');
for f = {'ROMECommand.m', 'RobotLinkRecorder.m', 'RobotLinkEcho.m', 'MotiveDataCounting.m'}
    txt = fileread(fullfile(tmp, f{1}));
    bad = regexp(txt, '(tcpclient|MulticastSocket|DatagramSocket|serialport|serial\(|tcpip|udpport|natnet)\s*\(', 'match');
    assert(isempty(bad), 'test_motive_emulator:network', '%s contains a network call: %s', f{1}, strjoin(bad, ', '));
end
for c = {'ROMECommand', 'RobotLinkRecorder', 'RobotLinkEcho', 'MotiveDataCounting', 'motive_source_check', 'test_v3_table3', 'run_9dof_cases'}
    assert(strncmp(fileparts(which(c{1})), tmp, numel(tmp)), 'test_motive_emulator:stub', '%s resolves to %s, not the copy %s', c{1}, which(c{1}), tmp);
end
recf = fullfile(tmp, 'robot_link_record.txt');   % fake robot record (char)
repf = fullfile(tmp, 'robot_link_replay.txt');   % replay file of RobotLinkEcho (char)
cntf = fullfile(tmp, 'motive_open_count.txt');   % MotiveDataCounting's counter (char)
slx  = fullfile(tmp, [mdl '.slx']);           % the copy's model (char)

% ---- the break, on the copy only -----------------------------------------------------------------------------------
if ~isempty(ib)
    for e = 1:numel(B{ib,3})
        ed = B{ib,3}{e};
        if strcmp(ed{1}, 'tolrun'), tolRun = ed{2}; fprintf('  break: tolRun = %g\n', tolRun); continue; end   % the test's own backstop (mf_tolrun)
        if strcmp(ed{1}, 'file')              % text edit of a file in the copy (exactly one match), no model involved
            f = fullfile(tmp, ed{2});
            fid = fopen(f, 'r');  s = char(fread(fid, Inf, '*uint8').');  fclose(fid);
            n = numel(strfind(s, ed{3}));
            assert(n == 1, 'test_motive_emulator:brk', 'break edit in %s matches %d times (expected 1)', ed{2}, n);
            fid = fopen(f, 'w');  fwrite(fid, uint8(strrep(s, ed{3}, ed{4})));  fclose(fid);
            clear(ed{2}(1:end-2));  rehash;
            fprintf('  break edit %d applied to the copy: %s\n', e, ed{2});
            continue
        end
        evalc('load_system(slx)');
        switch ed{1}
            case 'constpose'
                delete_line(mdl, 'StateDelay/1', 'MotiveEmulator/1');
                add_block('simulink/Sources/Constant', [mdl '/F7ConstPose'], 'Value', 'q0_9dof');
                add_line(mdl, 'F7ConstPose/1', 'MotiveEmulator/1');
            case 'param'
                fprintf('  break: %s %s ''%s'' -> ''%s''\n', ed{2}, ed{3}, get_param([mdl '/' ed{2}], ed{3}), ed{4});
                set_param([mdl '/' ed{2}], ed{3}, ed{4});
            case 'chart'
                rt = sfroot;
                p = [mdl '/' ed{2}];  if strcmp(ed{2}, 'MotiveFloor'), p = [mdl '/Enabled' newline 'Subsystem/MotiveFloor']; end
                ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', p);
                assert(numel(ch) == 1, 'test_motive_emulator:brk', 'chart %s found %d times', ed{2}, numel(ch));
                s = strrep(ch.Script, crlf, lf);
                n = numel(strfind(s, ed{3}));
                assert(n == 1, 'test_motive_emulator:brk', 'break edit in chart %s matches %d times (expected 1)', ed{2}, n);
                ch.Script = strrep(s, ed{3}, ed{4});
        end
        save_system(mdl);  close_system(mdl, 0);
        fprintf('  break edit %d applied to the copy: %s\n', e, ed{1});
    end
    evalin('base', 'clear all');
end

% ---- define_constants flag lines of the copy -----------------------------------------------------------------------
dcf = fullfile(tmp, 'define_constants.m');    % (char)
fid = fopen(dcf, 'r');  dcBase = char(fread(fid, Inf, '*uint8').');  fclose(fid);
FL = {'', '', '', 'sensor_model = 1;', '', 'motive_emulate_noise = 1;'};   % flag lines as in the file (char); 1-3 and 5 are derived from run_mode since F12 (dc_set_mode)
nFL = cellfun(@(s) numel(strfind(dcBase, [lf s])), FL([4 6]));
assert(all(nFL == 1), 'test_motive_emulator:dc', 'copy define_constants.m flag lines matched %s times (expected 1 each)', mat2str(nFL));
setfl = @(t, k, v) strrep(t, [lf FL{k}], [lf regexprep(FL{k}, '=\s*.*;$', ['= ' v ';'])]);   % set flag line k to value text v

rows = @(d, N) reshape(permute(d, [find(size(d) == N, 1, 'last'), setdiff(1:ndims(d), find(size(d) == N, 1, 'last'))]), N, []);   % logged Data -> N x n
blkM = [mdl '/Enabled' newline 'Subsystem/MATLAB System'];   % the MotiveData block (SID 105)
% name | EH | EM | sensor_model (sim only) | motive_emulate_noise | arm_freeze | telemetry: '' echo, 'replay'
cfg = {'s0', 0, 0, 0, 0, 1, ''; 's1', 0, 1, 0, 0, 1, ''; 's3', 0, 0, 1, 0, 1, ''; 's4', 0, 1, 1, 1, 1, ''; 's2', 0, 1, 1, 0, 1, ''; ...
       'h1', 1, 0, 0, 0, 0, ''; 'h2', 1, 1, 0, 1, 0, 'replay'; 'h3', 1, 1, 0, 0, 1, ''; 'h4', 1, 1, 0, 0, 0, ''};   % h2 motive_emulate_noise 1: the gate must keep the noise off on the robot (sensor_model forced 0; F7 re-review)
R = struct();
if isfile(cntf), delete(cntf); end
for r = 1:size(cfg, 1)
    [nm, EH, EM, SM, NZ, AF, TL] = cfg{r,:};
    if ~EH, nRun = 1; elseif AF, nRun = 2; else, nRun = 3; end   % run mode with this EnableHardware and arm_freeze (F12)
    t = dc_set_mode(dcBase, nRun, struct('EnableHardware', EH, 'EnableMotive', EM, 'arm_freeze', AF));  t = setfl(t, 6, num2str(NZ));
    if ~EH, t = setfl(t, 4, num2str(SM)); end
    fid = fopen(dcf, 'w');  fwrite(fid, uint8(t));  fclose(fid);
    if isfile(recf), delete(recf); end
    if strcmp(TL, 'replay')
        Q1 = R.h1.Q;                          % h1's q_cmd (N x 9): the joints the EnableMotive 0 run commanded
        fw = L.arm_sign .* rad2deg(Q1(:, 4:9)) + L.arm_offset_deg;   % firmware degrees, unclamped (N x 6)
        fid = fopen(repf, 'w');  fprintf(fid, 'ARM,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,READY\n', fw.');  fclose(fid);
    elseif isfile(repf)
        delete(repf);
    end
    nOpen0 = 0;  if isfile(cntf), nOpen0 = numel(strfind(fileread(cntf), 'openSocket')); end
    evalin('base', 'clear all');
    clear define_constants;  rehash;
    evalc('load_system(slx)');                % PostLoadFcn = the copy's define_constants
    sim0 = get_param(blkM, 'SimulateUsing');
    set_param(blkM, 'System', 'MotiveDataCounting');
    set_param(blkM, 'SimulateUsing', sim0);
    assert(strcmp(get_param(blkM, 'System'), 'MotiveDataCounting') && strcmp(get_param(blkM, 'SimulateUsing'), 'Interpreted execution'), ...
           'test_motive_emulator:stub', 'MotiveData block not switched to MotiveDataCounting');
    load_system('simulink');
    add_block('simulink/Sinks/To Workspace', [mdl '/F7StateIn'], 'VariableName', 'state_in', 'SaveFormat', 'Timeseries');
    add_line(mdl, 'StateSource/1', 'F7StateIn/1');
    evalin('base', 'clear ukd_current');
    err = '';  haveOut = false;  out = [];  simtxt = '';
    t0 = tic;
    try
        simtxt = evalc('out = sim(mdl, ''SimulationMode'', ''normal'', ''ReturnWorkspaceOutputs'', ''on'');');
        haveOut = true;
    catch ex
        err = [ex.identifier ': ' ex.message];
        for c1 = 1:numel(ex.cause)
            err = [err ' | ' ex.cause{c1}.identifier ': ' ex.cause{c1}.message]; %#ok<AGROW>
            for c2 = 1:numel(ex.cause{c1}.cause), err = [err ' | ' ex.cause{c1}.cause{c2}.identifier ': ' ex.cause{c1}.cause{c2}.message]; end %#ok<AGROW>
        end
    end
    twEnd = double(java.lang.System.nanoTime()) * 1e-9;   % wall stamp right after sim() (s; the recorder's clock)
    S = struct('name', nm, 'EH', EH, 'EM', EM, 'wall', toc(t0), 'err', err, 'haveOut', haveOut, 'simtxt', simtxt, 'twEnd', twEnd);
    S.pac = get_param(mdl, 'EnablePacing');  S.rate = double(get_param(mdl, 'PacingRate'));
    for v = {'sensor_model', 'motive_source', 'motive_emulate_noise', 'EnableMotive', 'EnableHardware', 'q0_9dof', 'arm_freeze_user', 'arm_freeze_deg', 'arm_home', 'max_rpm'}
        try, S.(v{1}) = evalin('base', v{1}); catch, S.(v{1}) = []; end
    end
    close_system(mdl, 0);
    S.nOpen = 0;  if isfile(cntf), S.nOpen = numel(strfind(fileread(cntf), 'openSocket')) - nOpen0; end
    S.K = {};  S.TS = zeros(0, 1);  S.TW = zeros(0, 1);  S.TX = {};
    if isfile(recf)
        C = textscan(fileread(recf), '%s %f %f %s', 'Delimiter', '\t');
        S.K = C{1};  S.TS = C{2};  S.TW = C{3};
        S.TX = cellfun(@(h) char(sscanf(h, '%2x').'), C{4}, 'UniformOutput', false);
    end
    S.t = zeros(0, 1);  S.Q = [];  S.QM = [];  S.SI = [];  S.QT = [];  S.W = [];  S.J = [];
    if haveOut
        S.t = out.q_cmd.Time;  N = numel(S.t);
        S.Q = rows(out.q_cmd.Data, N);  S.QM = rows(out.q_meas.Data, N);  S.SI = rows(out.state_in.Data, N);
        S.QT = rows(out.q_track.Data, N);  S.W = rows(out.wheel_sent.Data, N);  S.J = rows(out.joint_sent_rad.Data, N);
        S.G = rows(out.guard_flag.Data, N);   % CommandGuard flag per step (0 pass, 1 wheels scaled, 2 non-finite)
    end
    fprintf('run %s EnableHardware %d EnableMotive %d (%s) sensor_model %s noise %s arm_freeze_user %s telemetry %s: returned %d, %d steps, %d records, openSocket x%d, wall %.1f s, pacing %s %g\n', ...
            nm, EH, EM, char(string(S.motive_source)), mat2str(S.sensor_model), mat2str(S.motive_emulate_noise), mat2str(S.arm_freeze_user), ...
            char(string(TL) + ""), haveOut, numel(S.t), numel(S.K), S.nOpen, S.wall, S.pac, S.rate);
    if ~isempty(err), fprintf('     stopped: %s\n', err(1:min(end, 600))); end
    R.(nm) = S;
end
if isfile(repf), delete(repf); end
fails = {};                                   % one message per failed check (cell of char)
q0 = R.s0.q0_9dof(:);                         % 9x1 start configuration (m, rad)

% ---- (i) exact plumbing ---------------------------------------------------------------------------------------------
% Per step, the state the controller reads must be the model's own predicted pose: state_in(1) = q0_9dof and
% state_in(k) base = q_cmd(k-1) base (the round trip floor_to_motive -> motive_to_floor -> unwrap), 1e-12. The
% whole-run |q_cmd(s1) - q_cmd(s0)| is PRINTED, not asserted: the heading round trip is exact only to about one ulp of
% pi (4.4e-16, first run of this test, h2), and the closed loop amplifies that over 621 steps (7.7e-11 measured in the
% first run, 19 Nov 2026), so a whole-run 1e-12 cannot be met through any quaternion round trip (F7 report).
A = R.s0;  E = R.s1;
dI = Inf;  dBT = Inf;  dJM = Inf;  dBQ = Inf;  dS1 = Inf;   % max |q_cmd s1 - s0|, |state base - q_track|, |state joints - q_meas joints|, |state(k) base - q_cmd(k-1) base|, |state(1) - q0|
okI = A.haveOut && E.haveOut && isequal(size(A.Q), size(E.Q)) && E.nOpen == 0 && numel(E.t) > 10;
if okI
    dI = max(abs(E.Q - A.Q), [], 'all');
    dBT = max(abs(E.SI(:, 1:3) - E.QT), [], 'all');
    dJM = max(abs(E.SI(:, 4:9) - E.QM(:, 4:9)), [], 'all');
    dBQ = max(abs(E.SI(2:end, 1:3) - E.Q(1:end-1, 1:3)), [], 'all');
    dS1 = max(abs(E.SI(1, :).' - q0));
    okI = dBQ <= tolQ && dS1 <= tolQ && dBT == 0 && dJM == 0 && dI <= tolRun;   % whole-run backstop (controller ruling Q-F7-c)
    N1 = numel(E.t);  kq = max(1, round(N1/4));   % steps at which the growth is printed
    fprintf('\n(i)   s1 vs s0 whole run: |dq_cmd| at step 1 %.3e, at t = %.2f s %.3e, at the end %.3e, max %.3e over %d steps (need <= %g, backstop)\n', ...
            max(abs(E.Q(1,:) - A.Q(1,:))), E.t(kq), max(abs(E.Q(kq,:) - A.Q(kq,:))), max(abs(E.Q(end,:) - A.Q(end,:))), dI, N1, tolRun);
end
fprintf('(i)   s1 (EnableMotive 1, emulator, sensor_model 0): state(k) base - q_cmd(k-1) base %.3e, state(1) - q0_9dof %.3e (need <= %g); state base - q_track %.3e, state joints - q_meas joints %.3e (need 0); openSocket x%d -> %s\n', ...
        dBQ, dS1, tolQ, dBT, dJM, E.nOpen, mat2str(okI));
if ~okI, fails{end+1} = sprintf('(i) exact plumbing (state base vs q_cmd(k-1) %.3e, state(1) vs q0 %.3e, base-q_track %.3e, joints-q_meas %.3e, openSocket %d)', dBQ, dS1, dBT, dJM, E.nOpen); end

% ---- (ii) paper noise ------------------------------------------------------------------------------------------------
dN = Inf;  dN0 = NaN;                         % max |q_cmd s4 - s3|, |q_cmd s2 - s3| (m, rad)
if R.s3.haveOut && R.s4.haveOut && isequal(size(R.s3.Q), size(R.s4.Q)), dN = max(abs(R.s4.Q - R.s3.Q), [], 'all'); end
if R.s3.haveOut && R.s2.haveOut && isequal(size(R.s3.Q), size(R.s2.Q))
    dN0 = max(abs(R.s2.Q - R.s3.Q), [], 'all');
    fprintf('(ii)  noise-free emulator s2 vs noisy s3 (reported only): max |dq_cmd| %.3e; base x/y %.3e m, yaw %.3e rad, joints %.3e rad\n', dN0, ...
            max(abs(R.s2.Q(:,1:2) - R.s3.Q(:,1:2)), [], 'all'), max(abs(R.s2.Q(:,3) - R.s3.Q(:,3))), max(abs(R.s2.Q(:,4:9) - R.s3.Q(:,4:9)), [], 'all'));
end
% Per step, s4's state base must equal SensorModel's noisy base q_meas(1:3) (the same MocapNoise draws), 1e-12; the
% whole-run s4 vs s3 difference is printed only (same one-ulp amplification as (i)).
F4 = R.s4;  dNB = Inf;  dNJ = Inf;            % max |state base - q_meas base|, |state joints - q_meas joints| in s4
okNa = F4.haveOut && numel(F4.t) > 10 && F4.nOpen == 0;
if okNa
    dNB = max(abs(F4.SI(:, 1:3) - F4.QM(:, 1:3)), [], 'all');
    dNJ = max(abs(F4.SI(:, 4:9) - F4.QM(:, 4:9)), [], 'all');
    okNa = dNB <= tolQ && dNJ == 0 && dN <= tolRun;   % whole-run backstop (controller ruling Q-F7-c)
end
fprintf('(ii)  s4 (emulator, sensor_model 1, motive_emulate_noise 1): state base - SensorModel noisy base %.3e (need <= %g), joints %.3e (need 0); whole run vs s3 (EnableMotive 0) max |dq_cmd| %.3e (need <= %g, backstop) -> %s\n', ...
        dNB, tolQ, dNJ, dN, tolRun, mat2str(okNa));
% Table 3 through the emulator is NOT re-run here (F7 fix round 1): since the author's mode decision the user-block
% defaults ARE mode 1 (EnableMotive 1, 'emulator', motive_emulate_noise 1, sensor_model 1), run_9dof_cases forces
% mode 1 and prints the flags it ran with, and test_suite steps 7-8 run it and test_v3_table3. The in-test copy of
% that run crashed MATLAB (exit 139 / 0xc0000409) in 4 of 6 attempts on 19 Nov, so it was removed from this test.
if ~okNa, fails{end+1} = sprintf('(ii) paper noise through the emulator (s4 state base vs q_meas %.3e, whole run %.3e)', dNB, dN); end

% ---- (iii) robot modes 2 and 3 against the fake robot---------------------------------------------------------------------
H1 = R.h1;  H2 = R.h2;  H3 = R.h3;  H4 = R.h4;
nOpenHw = H1.nOpen + H2.nOpen + H3.nOpen + H4.nOpen;
ca = nOpenHw == 0;
fprintf('\n(iii) a  openSocket calls h1 %d, h2 %d, h3 %d, h4 %d -> %s\n', H1.nOpen, H2.nOpen, H3.nOpen, H4.nOpen, mat2str(ca));
% parse the ROME lines and the telemetry answers of each robot run
P = struct();
for nmc = {'h1', 'h2', 'h3', 'h4'}
    S = R.(nmc{1});
    w = strcmp(S.K, 'W');  ln = S.TX(w);  ts = S.TS(w);  tw = S.TW(w);
    isR = startsWith(ln, 'ROME,');
    fmt = all(~cellfun(@isempty, regexp(ln(isR), '^ROME(,-?\d+\.\d{3}){10}\n$', 'once')));
    V = zeros(0, 10);
    if fmt && any(isR), V = cell2mat(cellfun(@(s) sscanf(s, 'ROME,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f').', ln(isR), 'UniformOutput', false)); end
    ra = S.TX(strcmp(S.K, 'R'));              % telemetry answers, in order (char)
    TA = NaN(0, 6);
    if ~isempty(ra), TA = cell2mat(cellfun(@(s) sscanf(s, 'ARM,%f,%f,%f,%f,%f,%f').', ra, 'UniformOutput', false)); end
    P.(nmc{1}) = struct('ln', {ln}, 'ts', ts, 'tw', tw, 'isR', isR, 'fmt', fmt, 'V', V, 'TA', TA, 'tR', ts(isR), 'twR', tw(isR));
end
% b: the state the controller read in h2
eB = Inf;  eJ = Inf;  eJ1 = Inf;  e0 = Inf;   % max |base - q_cmd(k-1) base|, |joints - decode|, |joints - h1 q_cmd(k-1)|, |state(1) - q0|
N2 = numel(H2.t);
cb = H2.haveOut && N2 > 10 && size(P.h2.TA, 1) >= N2 - 1;
if cb
    eB = max(abs(H2.SI(2:end, 1:3) - H2.Q(1:end-1, 1:3)), [], 'all');
    eJ = max(abs(H2.SI(2:end, 4:9) - decode(P.h2.TA(1:N2-1, :))), [], 'all');
    if isequal(size(H1.Q), size(H2.Q)), eJ1 = max(abs(H2.SI(2:end, 4:9) - H1.Q(1:end-1, 4:9)), [], 'all'); end
    e0 = max(abs(H2.SI(1, :).' - q0));
    cb = eB <= tolQ && eJ <= tolQ && e0 <= tolQ;
end
fprintf('(iii) b  h2 returned %d (motive_cal.measured %s, emulator: not refused); state(k) base - q_cmd(k-1) base %.3e; joints - decoded telemetry(k-1) %.3e (vs h1 q_cmd(k-1) joints %.3e); state(1) - q0_9dof %.3e (need <= %g) -> %s\n', ...
        H2.haveOut, 'false (file)', eB, eJ, eJ1, e0, tolQ, mat2str(cb));
fprintf('         |arm_home - q0_9dof(4:9)| = %s deg (the old TelemetryDelay IC; premise: > 0 so the IC change matters)\n', mat2str(abs(rad2deg(H2.arm_home(:) - q0(4:9))).', 4));
% c: h2's lines equal h1's
V1 = P.h1.V;  V2 = P.h2.V;  dC = Inf;  nSame = 0;  kDiff = NaN;
cc = P.h1.fmt && P.h2.fmt && size(V1, 1) > 0 && isequal(size(V1), size(V2)) && size(V2, 1) == N2;
if cc
    dC = max(abs(V2 - V1), [], 'all');
    l1 = P.h1.ln(P.h1.isR);  l2 = P.h2.ln(P.h2.isR);
    same = strcmp(l1, l2);  nSame = sum(same);  kDiff = find(~same, 1);
    cc = dC <= tolField;
    if ~isempty(kDiff), fprintf('         first differing pair (line %d): h1 %s         h2 %s', kDiff, l1{kDiff}, l2{kDiff}); end
    fprintf('(iii) c  h2 (mode 3) vs h1 (EnableMotive 0): %d ROME lines each; max |field diff| wheels %.3g rpm, joints %.3g deg (tol %.4g); %d / %d lines string-identical, first difference at line %s -> %s\n', ...
            size(V2, 1), max(abs(V2(:,1:4) - V1(:,1:4)), [], 'all'), max(abs(V2(:,5:10) - V1(:,5:10)), [], 'all'), tolField, nSame, size(V2, 1), mat2str(kDiff), mat2str(cc));
else
    fprintf('(iii) c  h2 vs h1: line counts %d / %d (logged steps %d), formats %d %d -> false\n', size(V2, 1), size(V1, 1), N2, P.h2.fmt, P.h1.fmt);
end
% d: h2's lines equal its own logs
dD = Inf;
cd_ = H2.haveOut && size(V2, 1) == N2 && N2 > 0;
if cd_
    dD = max([max(abs(V2(:,1:4) - H2.W), [], 'all'), max(abs(V2(:,5:10) - clampf(L.arm_sign .* rad2deg(H2.J) + L.arm_offset_deg)), [], 'all')]);
    cd_ = dD <= tolField;
end
fprintf('(iii) d  h2 lines vs its own wheel_sent / joint_sent_rad (map + clamp): max %.3g (tol %.4g) -> %s\n', dD, tolField, mat2str(cd_));
% e: pacing
ratio = NaN;
if numel(P.h2.tR) > 1, ratio = (P.h2.twR(end) - P.h2.twR(1)) / (P.h2.tR(end) - P.h2.tR(1)); end
ce = ratio >= ratioMin && strcmp(H2.pac, 'on') && H2.rate == 1;
fprintf('(iii) e  h2 pacing: wall / simulated span of the ROME lines %.3f (need >= %.2f), EnablePacing %s rate %g -> %s\n', ratio, ratioMin, H2.pac, H2.rate, mat2str(ce));
% f: framing and STOP_ALL timing, h2 and h4
cf = true;
for nmc = {'h2', 'h4'}
    S = R.(nmc{1});  p = P.(nmc{1});  ln = p.ln;
    START = ['START_GV' lf];  STOP = ['STOP_ALL' lf];
    iSt = find(strcmp(S.K, 'W') & strcmp(S.TX, STOP), 1, 'last');
    stW = NaN;  stS = NaN;  if ~isempty(iSt), stW = S.TW(iSt);  stS = S.TS(iSt); end
    tFin = NaN;  if ~isempty(S.t), tFin = S.t(end); end
    ok = S.haveOut && ~isempty(ln) && strcmp(ln{1}, START) && sum(strcmp(ln, START)) == 1 && strcmp(ln{end}, STOP) && sum(strcmp(ln, STOP)) == 1 ...
         && sum(~p.isR & ~strcmp(ln, START) & ~strcmp(ln, STOP)) == 0 && stW < S.twEnd && abs(stS - tFin) <= 1e-9;
    fprintf('(iii) f  %s: %d writes, START_GV first x%d, STOP_ALL last x%d, STOP_ALL at sim time %.4f s (final step %.4f s), %.4f s wall before sim() returned -> %s\n', ...
            nmc{1}, numel(ln), sum(strcmp(ln, START)), sum(strcmp(ln, STOP)), stS, tFin, S.twEnd - stW, mat2str(ok));
    cf = cf && ok;
end
% g: (1) h3, mode 2 (arm frozen, author 19 Nov: "Arm from model if frozen"): runs to the end; the controller's joints
% are the model's (state_in joints = q_meas joints exactly: JointSource on SimJoints because arm_freeze_user = 1), so
% q_cmd(h3) = q_cmd(s1) (the mode-1 run, same sensor_model 0) within tolRun: bounded. History: before the ruling the
% controller read the parked telemetry here and its joint commands grew to 1657 deg (run-1 log); a JointSource on
% EnableHardware alone (break mf_jointeh) brings that back. (2) h4, mode 3 (arm moving), the fake arm ECHOING each
% line's %.3f joint fields: the state's joints equal the decoded echo (1e-12) and differ from q_cmd(k-1)'s joints
% (> 1e-9 rad somewhere, the %.3f print), so they come from the telemetry, not from StateDelay.
V3 = P.h3.V;  N3 = numel(H3.t);
afd = NaN(1, 6);  if numel(H3.arm_freeze_deg) == 6, afd = H3.arm_freeze_deg(:).'; end
dF3 = Inf;  eM3 = Inf;  dQ3 = Inf;  rg3 = NaN(1, 6);   % joint fields - arm_freeze_deg (deg), state joints - q_meas joints, |q_cmd h3 - s1| (m, rad), joint-command range (deg)
refH3 = H3.haveOut && P.h3.fmt && size(V3, 1) == N3 && N3 > 10 && isequal(size(H3.Q), size(R.s1.Q));
if refH3
    dF3 = max(abs(V3(:, 5:10) - afd), [], 'all');
    eM3 = max(abs(H3.SI(:, 4:9) - H3.QM(:, 4:9)), [], 'all');
    dQ3 = max(abs(H3.Q - R.s1.Q), [], 'all');
    rg3 = rad2deg(max(H3.Q(:, 4:9)) - min(H3.Q(:, 4:9)));
    refH3 = dF3 <= tolField && eM3 == 0 && dQ3 <= tolRun;
end
banH3 = [numel(strfind(H3.simtxt, 'ARM FROZEN at arm_freeze_deg')), numel(strfind(H3.simtxt, 'ARM WILL MOVE'))];
refH3 = refH3 && isequal(banH3, [1 0]);
V4 = P.h4.V;  N4 = numel(H4.t);
eJ4 = Inf;  sep4 = NaN;  wPk = NaN;           % |state joints - decoded echo| (rad), max |decoded echo - q_cmd(k-1) joints| (rad), peak wheel (rpm)
cg4 = H4.haveOut && P.h4.fmt && size(V4, 1) == N4 && N4 > 10 && size(P.h4.TA, 1) >= N4 - 1;
if cg4
    dec4 = decode(P.h4.TA(1:N4-1, :));
    eJ4 = max(abs(H4.SI(2:end, 4:9) - dec4), [], 'all');
    sep4 = max(abs(dec4 - H4.Q(1:end-1, 4:9)), [], 'all');
    wPk = max(abs([V4(:, 1:4); V2(:, 1:4)]), [], 'all');
    cg4 = eJ4 <= tolQ && sep4 > 1e-9;         % wPk <= max_rpm is printed only: CommandGuard enforces it, so it cannot fail
end
banH2 = [numel(strfind(H2.simtxt, 'ARM FROZEN at arm_freeze_deg')), numel(strfind(H2.simtxt, 'ARM WILL MOVE'))];
cg = refH3 && cg4 && isequal(banH2, [0 1]);
fprintf(['(iii) g  h3 (mode 2, arm frozen): returned %d; joint fields - arm_freeze_deg %.3g deg (tol %.4g); state joints - model joints %.3e (need 0); ' ...
         '|q_cmd h3 - q_cmd s1 (mode 1)| %.3e (need <= %g); joint-command range %s deg; banner [FROZEN MOVE] %s -> %s\n'], ...
        H3.haveOut, dF3, tolField, eM3, dQ3, tolRun, mat2str(rg3, 4), mat2str(banH3), mat2str(refH3));
if ~isempty(H3.err), fprintf('         h3 stopped: %s\n', H3.err(1:min(end, 400))); end
fprintf('(iii) g  h4 (arm moving, echo): state joints - decoded echo %.3e (need <= %g); decoded echo vs q_cmd(k-1) joints up to %.3e rad (need > 1e-9: telemetry, not StateDelay); peak wheel h2/h4 %.3f rpm (max_rpm %g, printed only); banner [FROZEN MOVE] h2 %s -> %s\n', ...
        eJ4, tolQ, sep4, wPk, L.max_rpm, mat2str(banH2), mat2str(cg));
okIII = ca && cb && cc && cd_ && ce && cf && cg;
if ~okIII
    tags = {'a', 'b', 'c', 'd', 'e', 'f', 'g'};
    fails{end+1} = sprintf('(iii) robot modes 2 and 3 against the fake robot: failed parts %s', strjoin(tags(~[ca cb cc cd_ ce cf cg]), ' '));
end

% ---- (r) unwrap state reset between runs, normal and Fast Restart -----------------------------------------------------
t = setfl(dc_set_mode(dcBase, 1), 4, '0');   % run_mode 1 (EnableMotive 1, emulator, EnableHardware 0), sensor_model 0
fid = fopen(dcf, 'w');  fwrite(fid, uint8(t));  fclose(fid);
evalin('base', 'clear all');  clear define_constants;  rehash;
evalc('load_system(slx)');
sim0 = get_param(blkM, 'SimulateUsing');
set_param(blkM, 'System', 'MotiveDataCounting');  set_param(blkM, 'SimulateUsing', sim0);
load_system('simulink');
ssN = ['Enabled' newline 'Subsystem'];
delete_line(mdl, 'MotiveEmulator/1', [ssN '/2']);
add_block('simulink/Sources/Clock', [mdl '/F7SweepClock']);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/F7Sweep']);
rt = sfroot;
fc = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/F7Sweep']);
fc.Script = sprintf('function pose7 = F7Sweep(t, q0_9dof, motive_cal)\n%% test_motive_emulator (r): heading sweeps 1.25 turns in %g s from q0_9dof(3)\npose7 = floor_to_motive([q0_9dof(1:2); q0_9dof(3) + 2*pi*1.25*t/%g], 0, motive_cal);\n', Tsw, Tsw);
wantD = {'t', 'Input'; 'pose7', 'Output'; 'q0_9dof', 'Parameter'; 'motive_cal', 'Parameter'};   % data name | scope
for d = reshape(fc.find('-isa', 'Stateflow.Data'), 1, [])
    if ~any(strcmp(d.Name, wantD(:,1))), delete(d); end
end
for k = 1:size(wantD, 1)
    d = fc.find('-isa', 'Stateflow.Data', 'Name', wantD{k,1});
    if isempty(d), d = Stateflow.Data(fc);  d.Name = wantD{k,1}; end
    d.Scope = wantD{k,2};
end
add_line(mdl, 'F7SweepClock/1', 'F7Sweep/1');
add_line(mdl, 'F7Sweep/1', [ssN '/2']);
set_param(mdl, 'SimulationMode', 'normal');   % set once: Fast Restart rejects SimulationMode as a sim() argument (first run of this test)
% ROMECommand (Keanu's, untouched) errors on the SECOND Fast Restart run at its line 'if toc(obj.lastTelemetryTime_) > 0.5'
% (measured 19 Nov 2026, make-it-fail run mf_const of this test, EnableHardware 0); it is not part of the camera path and
% with EnableHardware 0 its outputs are unused by the controller (JointSource takes SensorModel's joints), so it is
% commented out IN MEMORY for the reset runs only, to let the Fast Restart pair reach the unwrap state.
set_param([mdl '/ROMECommand'], 'Commented', 'on');
QT = cell(1, 4);  rErr = repmat({''}, 1, 4);  % q_track of r1 r2 fr1 fr2 (N x 3 each), error text per run
for k = 1:4
    try
        if k == 3, set_param(mdl, 'FastRestart', 'on'); end
        evalin('base', 'clear ukd_current');
        evalc('o = sim(mdl, ''StopTime'', num2str(Tsw), ''ReturnWorkspaceOutputs'', ''on'');');
        QT{k} = rows(o.q_track.Data, numel(o.q_track.Time));
    catch er
        rErr{k} = [er.identifier ': ' er.message];
        for c1 = 1:numel(er.cause)
            rErr{k} = [rErr{k} ' | ' er.cause{c1}.identifier ': ' er.cause{c1}.message];
            for c2 = 1:numel(er.cause{c1}.cause), rErr{k} = [rErr{k} ' | ' er.cause{c1}.cause{c2}.identifier ': ' er.cause{c1}.cause{c2}.message]; end
        end
    end
end
try, set_param(mdl, 'FastRestart', 'off'); catch, end
q0r = evalin('base', 'q0_9dof');
close_system(mdl, 0);
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dcBase));  fclose(fid);
sw = NaN;  d0 = NaN(1, 3);  dAll = NaN(1, 3);  % r1 sweep (rad); per r2 fr1 fr2: |first heading - q0(3)|, max |run - r1|
if ~isempty(QT{1}), sw = max(QT{1}(:, 3)) - q0r(3); end
for k = 2:4
    if ~isempty(QT{k}) && ~isempty(QT{1})
        d0(k-1) = abs(QT{k}(1, 3) - q0r(3));
        if isequal(size(QT{k}), size(QT{1})), dAll(k-1) = max(abs(QT{k} - QT{1}), [], 'all'); else, dAll(k-1) = Inf; end
    end
end
okRn = sw >= 2*pi && d0(1) <= tolQ && dAll(1) <= tolQ;            % normal restart (r2 vs r1)
% Fast Restart (controller ruling, F7 fix round 1): refused whenever EnableMotive ~= 0 (apply_pacing:fastrestartcam,
% InitFcn and StartFcn), so both fr runs must stop with that refusal and return nothing.
frRef = cellfun(@(s) contains(s, 'apply_pacing:fastrestartcam'), rErr(3:4));   % fr1, fr2 refused for that reason (logical)
okRf = all(frRef) && isempty(QT{3}) && isempty(QT{4});
fprintf('\n(r)   r1 heading sweep %.2f deg past q0(3) (need >= 360)\n', rad2deg(sw));
fprintf('(r)   normal restart: r2 first heading - q0(3) %.3e rad, max |r2 - r1| %.3e -> %s\n', d0(1), dAll(1), mat2str(okRn));
fprintf('(r)   Fast Restart (EnableMotive 1): fr1 refused with apply_pacing:fastrestartcam %d, fr2 %d (need both, nothing returned: %d %d) -> %s\n', ...
        frRef(1), frRef(2), isempty(QT{3}), isempty(QT{4}), mat2str(okRf));
for k = find(~cellfun(@isempty, rErr)), fprintf('      run %d stopped: %s\n', k, rErr{k}); end
if ~(okRn && okRf), fails{end+1} = sprintf('(r) unwrap state not reset between runs (normal %d, Fast Restart %d; start errors %s rad)', okRn, okRf, mat2str(d0, 3)); end

evalin('base', 'clear all');
if isappdata(0, 'f7_carry'), rmappdata(0, 'f7_carry'); end
if ~isempty(fails)
    tag = '';  if ~isempty(ib), tag = sprintf(' [break %s]', B{ib,1}); end
    error('test_motive_emulator:fail', '%d check(s) failed%s:\n  %s', numel(fails), tag, strjoin(fails, sprintf('\n  ')));
end
fprintf('\ntest_motive_emulator PASS: (i) (ii) (iii) (r)\n');
end
