function varargout = test_camera_guard_wiring(brk)
%TEST_CAMERA_GUARD_WIRING  F6 wiring + wire3 fix round 1 (20 Nov 2026): camera guard, arm-ready wait, telemetry loss,
%   wheel AND joint gate, safe stop, end to end against the recording fake robot. TEMPORARY COPY of ROME_merged;
%   ROMECommand.m in the copy = Keanu's file with only tcpclient replaced by RobotLinkEcho (tests/stubs_hw: records every
%   line, answers each ROME line with an ARM telemetry line, or the lines of robot_link_replay.txt and then nothing) or
%   RobotLinkRecorder (records, never answers). NO NETWORK. ROME_merged is never written.
%   Robot runs: mode 3 (robot, arm moving, fake camera), T_stop shortened to 4 s, paced (apply_pacing).
%   PARK = the parked posture in firmware degrees, q0_9dof(4:9) mapped and clamped (what calibrateROMEArm parks at).
%   Joint fields are compared within half a unit of the %.3f print (0.0005 deg).
%     (a) the real model is wired: CameraGuard feeds Reshape; StopAny <- RunGate; CmdGate <- RunGate; ROMECommand in 1
%         <- CmdGate, in 2 <- JointGate; wheel_sent <- CmdGate
%     (b) hold: an emulator dropout of 5 samples ([1.975, 2.225) s) is bridged: T_stop reached, 5 hold samples, longest
%         dropout 0.25 s, no stop; the joint fields of the 5 hold lines equal the line before the dropout (held)
%     (c) stop: 6 samples ([1.975, 2.275) s) stop the run at t = 2.25 s, reason 2; STOP_ALL last, once; the stop line's
%         wheels are 0; the joint fields of the 6 dropout lines equal the line before the dropout (held)
%     (d) arm-ready wait: RobotLinkRecorder (no telemetry): the arm wait refuses at t = 2.00 s; every ROME line has zero
%         wheels and the joints at PARK (never the arm_home seed)
%     (e) camera wait: an emulator dropout from t = 0 ([0, 3) s, no frame ever): the guard stops the run at t = 2.00 s,
%         reason 1; every ROME line has zero wheels and the joints at PARK
%     (f) telemetry loss after go: robot_link_replay.txt answers the first 20 ROME lines with ARM lines at PARK, then
%         nothing: go is given; ROMECommand drops armReady 0.5 s after the last line; after 5 lost samples the run stops
%         (refuse, lost seen) before T_stop; the stop line's wheels are 0; the joints of the lost samples are held
%   MAKE-IT-FAIL: test_camera_guard_wiring('<id>') edits one chart of the COPY's model in memory (every run); the named
%   check must FAIL:
%     mf_nostop       RunGate's stop forced to 0                                   -> (c)
%     mf_nogate       RunGate's wheel gate forced to 1                             -> (c)
%     mf_nojointgate  JointGate always passes the controller's joint command       -> (b)
%     mf_camwait      CameraGuard reports no wait (cam(1) = 0)                     -> (e)
%   test_camera_guard_wiring('list') returns the table. Flat function without local functions (CLAUDE.md 5).
B = {'mf_nostop',      'c', 'RunGate',   'stop  = double(cam(3) ~= 0 || arm(3) ~= 0);', 'stop  = 0;', 'RunGate never stops the run';
     'mf_nogate',      'c', 'RunGate',   'gate  = double(cam(1) == 0 && cam(3) == 0 && goNow && arm(3) == 0);', 'gate  = 1;', 'wheels never gated';
     'mf_nojointgate', 'b', 'JointGate', 'if running', 'if true', 'joints never gated';
     'mf_camwait',     'e', ['Enabled' newline 'Subsystem/CameraGuard'], 'cam = [w; h; st;', 'cam = [0; h; st;', 'camera wait not reported'};
if nargin >= 1 && strcmp(brk, 'list'), varargout{1} = B; return; end
ib = [];
if nargin >= 1 && ~isempty(brk)
    ib = find(strcmp(B(:,1), brk));
    assert(numel(ib) == 1, 'test_camera_guard_wiring:brk', 'unknown break %s', brk);
    fprintf('MAKE-IT-FAIL break %s (check (%s) must FAIL): %s\n', brk, B{ib,2}, B{ib,6});
end
here = fileparts(mfilename('fullpath'));  root = fileparts(here);  mdl = 'ROME_9DOF';  lf = char(10);
ssN = ['Enabled' newline 'Subsystem'];
fails = {};
% ---- (a) the real model -------------------------------------------------------------------------------------------
if bdIsLoaded(mdl), close_system(mdl, 0); end
evalc('load_system(fullfile(root, [mdl ''.slx'']))');
nm = @(h) strrep(get_param(h, 'Name'), newline, ' ');
W = {[mdl '/' ssN '/Reshape'], 1, 'CameraGuard'; [mdl '/Stop Simulation'], 1, 'StopAny'; [mdl '/StopAny'], 2, 'RunGate'; ...
     [mdl '/CmdGate'], 2, 'RunGate'; [mdl '/ROMECommand'], 1, 'CmdGate'; [mdl '/ROMECommand'], 2, 'JointGate'; ...
     [mdl '/To Workspace wheel_sent'], 1, 'CmdGate'};
okA = true;  ta = {};
for k = 1:size(W, 1)
    g = '<none>';
    if getSimulinkBlockHandle(W{k,1}) ~= -1
        p = get_param(W{k,1}, 'PortConnectivity');  s = p(W{k,2});
        if s.SrcBlock ~= -1, g = nm(s.SrcBlock); end
    end
    ta{end+1} = sprintf('%s<-%s', nm(W{k,1}), g); %#ok<AGROW>
    okA = okA && strcmp(g, W{k,3});
end
close_system(mdl, 0);
fprintf('(a) %s -> %s\n', strjoin(ta, ', '), mat2str(okA));
if ~okA, fails{end+1} = '(a) wiring'; end
% ---- copy with the fake robot ----------------------------------------------------------------------------------------
tmp = tempname;  startdir = pwd;  path0 = path;
mkdir(tmp);  copyfile(root, tmp);
prerun_fixture('signoff', tmp);
mdlcell = {mdl};
cleanup = onCleanup(@() cellfun(@(c) c(), {@() cd(startdir), @() path(path0), ...
          @() cellfun(@(m) close_system(m, 0), mdlcell(bdIsLoaded(mdl))), @() evalin('base', 'clear all'), @() rmdir(tmp, 's')})); %#ok<NASGU>
realtxt = fileread(fullfile(root, 'ROMECommand.m'));
old = 'tcpclient(obj.ESP32_IP, obj.ESP32_Port, "ConnectTimeout", 5)';
assert(numel(strfind(realtxt, old)) == 1, 'test_camera_guard_wiring:stub', 'tcpclient constructor found %d times', numel(strfind(realtxt, old)));
for f = {'RobotLinkRecorder.m', 'RobotLinkEcho.m', 'MotiveDataCounting.m'}
    copyfile(fullfile(here, 'stubs_hw', f{1}), fullfile(tmp, f{1}), 'f');
end
addpath(tmp, fullfile(tmp, 'Ground Vehicle'), fullfile(tmp, 'Optitrack'));
addpath(here, '-end');
cd(tmp);  rehash;
dcf = fullfile(tmp, 'define_constants.m');  slx = fullfile(tmp, [mdl '.slx']);  recf = fullfile(tmp, 'robot_link_record.txt');
cmdf = fullfile(tmp, 'ROMECommand.m');  repf = fullfile(tmp, 'robot_link_replay.txt');
dcBase = fileread(dcf);
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc_set_mode(dcBase, 3)));  fclose(fid);
Tshort = 4;                                        % shortened run (s)
runs = {'hold', 'RobotLinkEcho', [1.975 2.225], 0; 'stop', 'RobotLinkEcho', [1.975 2.275], 0; 'arm', 'RobotLinkRecorder', [-1 -1], 0; ...
        'camwait', 'RobotLinkEcho', [0 3], 0; 'armlost', 'RobotLinkEcho', [-1 -1], 20};   % name | link | dropout window | replay lines
Res = struct();  park = [];
for r = 1:size(runs, 1)
    [rn, link, win, nrep] = runs{r,:};
    fid = fopen(cmdf, 'w');  fwrite(fid, uint8(strrep(realtxt, old, [link '(obj.ESP32_IP, obj.ESP32_Port, "ConnectTimeout", 5)'])));  fclose(fid);
    if isfile(recf), delete(recf); end
    if isfile(repf), delete(repf); end
    evalin('base', 'clear all');  evalin('base', 'clear classes');
    clear define_constants lab_settings ROMECommand RobotLinkEcho RobotLinkRecorder;  rehash;
    evalc('load_system(slx)');
    set_param(mdl, 'SimulationMode', 'normal');
    q0 = evalin('base', 'q0_9dof');
    park = evalin('base', 'arm_sign(:).''') .* rad2deg(q0(4:9)).' + evalin('base', 'arm_offset_deg(:).''');
    park = arm_clamp(park, evalin('base', 'arm_fw_switch'), evalin('base', 'arm_fw_other'), evalin('base', 'arm_fw_margin_deg'));
    if nrep > 0
        fid = fopen(repf, 'w');  fprintf(fid, ['ARM' repmat(',%.17g', 1, 6) ',READY\n'], repmat(park(:), 1, nrep));  fclose(fid);
    end
    if ~isempty(ib)
        ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/' B{ib,3}]);
        assert(numel(ch) == 1 && numel(strfind(ch.Script, B{ib,4})) == 1, 'test_camera_guard_wiring:brk', '%s: break text not found once', B{ib,1});
        ch.Script = strrep(ch.Script, B{ib,4}, B{ib,5});
    end
    assignin('base', 'T_stop', Tshort);
    assignin('base', 'motive_emulate_dropout', win);
    prerun_fixture('park', tmp);
    err = '';  o = [];
    try
        evalc('o = sim(mdl, ''ReturnWorkspaceOutputs'', ''on'');');
    catch ex
        err = ex.message;
    end
    close_system(mdl, 0);
    S = struct('err', err, 'tend', NaN, 't', [], 'cam', zeros(0, 6), 'arm', zeros(0, 4), 'L', read_link_record(recf));
    try
        S.t = o.cam_guard.Time(:);  S.tend = S.t(end);
        S.cam = squeeze(o.cam_guard.Data);  if size(S.cam, 1) == 6 && size(S.cam, 2) ~= 6, S.cam = S.cam.'; end
        S.arm = squeeze(o.arm_gate.Data);   if size(S.arm, 1) == 4 && size(S.arm, 2) ~= 4, S.arm = S.arm.'; end
    catch ex2
        S.err = [S.err ' | output: ' ex2.message];
    end
    isR = startsWith(S.L.line, 'ROME,');
    S.ts = S.L.ts(isR);                            % simulation time of each ROME line (s)
    S.F = cell2mat(cellfun(@(s) sscanf(s(6:end), '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f', [1 10]), S.L.line(isR), 'UniformOutput', false));   % wheels 1:4, joints 5:10
    Res.(rn) = S;
    fprintf('[%s] link %s, window %s, replay %d: sim error ''%s'', t_end %.3f s, %d ROME lines\n', rn, link, mat2str(win), nrep, err, S.tend, size(S.F, 1));
end
if isfile(repf), delete(repf); end
tolJ = 0.0005 + 1e-9;                              % half a unit of the %.3f print (deg)
atT = @(S, t) find(abs(S.ts - t) < 1e-6, 1);       % ROME line sent at simulation time t (index)
heldOk = @(S, t0, tw) ~isempty(atT(S, t0)) && all(arrayfun(@(t) ~isempty(atT(S, t)) && max(abs(S.F(atT(S, t), 5:10) - S.F(atT(S, t0), 5:10))) <= tolJ, tw));
% ---- (b) hold --------------------------------------------------------------------------------------------------------
H = Res.hold;  okB = false;
if ~isempty(H.cam) && ~isempty(H.F)
    hj = heldOk(H, 1.95, 2.00:0.05:2.20);
    okB = isempty(H.err) && abs(H.tend - Tshort) < 1e-9 && sum(H.cam(:, 2)) == 5 && ~any(H.cam(:, 3)) && abs(max(H.cam(:, 5)) - 0.25) < 1e-9 && hj;
    fprintf('(b) hold: t_end %.3f s, hold samples %d, stop %d, longest dropout %.4f s; joints held over the dropout %d -> %s\n', H.tend, sum(H.cam(:, 2)), any(H.cam(:, 3)), max(H.cam(:, 5)), hj, mat2str(okB));
else
    fprintf('(b) hold: no output (%s) -> false\n', H.err);
end
if ~okB, fails{end+1} = '(b) 5-sample dropout bridged, joints held'; end
% ---- (c) stop --------------------------------------------------------------------------------------------------------
P = Res.stop;  okC = false;  tc = 'no output';
if ~isempty(P.cam) && ~isempty(P.F)
    ln = P.L.line;
    nStop = sum(strcmp(ln, ['STOP_ALL' lf]));  lastStop = ~isempty(ln) && strcmp(ln{end}, ['STOP_ALL' lf]);
    hj = heldOk(P, 1.95, 2.00:0.05:2.25);
    okC = abs(P.tend - 2.25) < 1e-9 && P.cam(end, 3) == 1 && P.cam(end, 4) == 2 && nStop == 1 && lastStop && all(P.F(end, 1:4) == 0) && hj;
    tc = sprintf('t_end %.3f s (want 2.25), stop %d reason %d, STOP_ALL x%d last %d, last ROME wheels %s, joints held %d', P.tend, P.cam(end, 3), P.cam(end, 4), nStop, lastStop, mat2str(P.F(end, 1:4)), hj);
end
fprintf('(c) stop: %s -> %s\n', tc, mat2str(okC));
if ~okC, fails{end+1} = '(c) 6-sample dropout stops the run, wheels zero, joints held, STOP_ALL last'; end
% ---- (d) arm-ready wait refuses --------------------------------------------------------------------------------------
A = Res.arm;  okD = false;  td = 'no output';
if ~isempty(A.arm) && ~isempty(A.F)
    dj = max(abs(A.F(:, 5:10) - park), [], 'all');
    okD = abs(A.tend - 2.00) < 1e-9 && A.arm(end, 3) == 1 && ~any(A.arm(:, 1)) && all(A.F(:, 1:4) == 0, 'all') && dj <= tolJ;
    td = sprintf('t_end %.3f s (want 2.00), refuse %d, go ever %d, %d ROME lines, max |wheel| %g, max |joint - PARK| %.4f deg', A.tend, A.arm(end, 3), any(A.arm(:, 1)), size(A.F, 1), max(abs(A.F(:, 1:4)), [], 'all'), dj);
end
fprintf('(d) arm wait: %s -> %s\n', td, mat2str(okD));
if ~okD, fails{end+1} = '(d) no telemetry: arm wait refuses, wheels zero, joints parked'; end
% ---- (e) camera wait -------------------------------------------------------------------------------------------------
E = Res.camwait;  okE = false;  te = 'no output';
if ~isempty(E.cam) && ~isempty(E.F)
    dj = max(abs(E.F(:, 5:10) - park), [], 'all');
    okE = abs(E.tend - 2.00) < 1e-9 && E.cam(end, 3) == 1 && E.cam(end, 4) == 1 && all(E.F(:, 1:4) == 0, 'all') && dj <= tolJ;
    te = sprintf('t_end %.3f s (want 2.00), stop %d reason %d, %d ROME lines, max |wheel| %g, max |joint - PARK| %.4f deg', E.tend, E.cam(end, 3), E.cam(end, 4), size(E.F, 1), max(abs(E.F(:, 1:4)), [], 'all'), dj);
end
fprintf('(e) camera wait: %s -> %s\n', te, mat2str(okE));
if ~okE, fails{end+1} = '(e) no camera frame: guard stops, wheels zero, joints parked'; end
% ---- (f) telemetry loss after go -------------------------------------------------------------------------------------
G = Res.armlost;  okF = false;  tf = 'no output';
if ~isempty(G.arm) && ~isempty(G.F)
    kl = find(G.arm(:, 4) ~= 0);                   % lost samples (index into the logged steps)
    hj = false;
    if ~isempty(kl), hj = heldOk(G, G.t(kl(1)) - 0.05, G.t(kl).'); end
    okF = any(G.arm(:, 1)) && G.arm(end, 3) == 1 && ~isempty(kl) && G.tend < Tshort - 1e-9 && all(G.F(end, 1:4) == 0) && hj;
    tf = sprintf('go %d, lost samples %d (first at %.2f s), refuse %d, t_end %.3f s, last wheels %s, joints held %d', any(G.arm(:, 1)), numel(kl), ...
                 G.t(max([kl; 1])), G.arm(end, 3), G.tend, mat2str(G.F(end, 1:4)), hj);
end
fprintf('(f) telemetry loss: %s -> %s\n', tf, mat2str(okF));
if ~okF, fails{end+1} = '(f) telemetry lost after go: held, then stopped'; end
if ~isempty(fails)
    error('test_camera_guard_wiring:fail', 'test_camera_guard_wiring FAILED: %s', strjoin(fails, '; '));
end
fprintf('test_camera_guard_wiring PASS\n');
end
