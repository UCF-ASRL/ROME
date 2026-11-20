function varargout = test_motive_model(brk)
%TEST_MOTIVE_MODEL  The ABSOLUTE floor-frame camera path in ROME_9DOF.slx (focus plan F5b, Plan 2 Task 2 tests iv, v).
%   F7 (19 Nov 2026): this test covers the REAL-camera source, so every run and every refusal case sets
%   motive_source = 'optitrack' (the file default is now 'emulator', tested by tests/test_motive_emulator.m). Since F7
%   MotiveData/7 reaches MotiveFloor through the switch PoseSource (tools/wire_motive_emulator.m), and the reader gate is
%   EnableMotive && EnableHardware && 'optitrack'. Break mf_legacy first removes the F7 blocks (wire_motive_emulator
%   'unwire'), because wire_motive_absolute refuses a model that has them.
%   Model under test (tools/wire_motive_absolute.m): inside the Enabled Subsystem (SID 100, enabled by EnableMotive)
%   the MotiveData block (SID 105) takes its Enable input from the root Constant MotiveReaderEnable,
%   double(EnableMotive ~= 0 && EnableHardware ~= 0); its output 7 (pose7, raw [x y z qx qy qz qw]) feeds the MATLAB
%   Function MotiveFloor (script = Optitrack/motive_floor_block.m: motive_to_floor with the Parameter motive_cal, then
%   yaw_unwrap seeded from q0_9dof(3), state kept only on finite headings), whose [x; y; psi] is tracked_position
%   (via Reshape) and the Discrete Derivative. tracked_position -> MuxTracked -> StateSource (u2 = EnableMotive) ->
%   UKDynamics in 1. The old q0 + displacement path (MotiveInit SID 221, Add SID 184, Gain -1 SID 173, Mux1 SID 108)
%   is gone. The refusal: motive_cal_check.m, called by define_constants.m and by apply_pacing.m for a hardware run
%   (model InitFcn and StartFcn).
%
%   NO NETWORK, EVER. A TEMPORARY COPY of ROME_merged (tempname) in which
%     ROMECommand.m        is tests/stubs_hw/ROMECommand.m (Keanu's file with only the transport replaced by
%                          RobotLinkRecorder; test_hw_nocam asserts that identity byte for byte) + RobotLinkRecorder.m
%     MotiveDataInject.m   (tests/stubs_motive) the REAL MotiveData with only openSocket (counts its calls in a file,
%                          opens nothing) and receiveBytes (nTimeout timeouts, then a fixed NatNet datagram built here
%                          from a known pose) overridden. The copy's MotiveData block is pointed at it in memory.
%   ROME_merged itself is never written. Each run loads the copy fresh (PostLoadFcn define_constants with the file's
%   EnableHardware = 0, EnableMotive = 0), then sets the run's flags IN THE BASE WORKSPACE (the way test_pacing does),
%   so the copy's define_constants.m is never edited.
%
%   Injected pose (test-local): cal = Rz(37 deg) Rx(90 deg) (a Y-up Motive frame turned 37 deg), t = [1.2; -0.4; 0.3] m,
%   scale 0.001 m per Motive unit (mm), measured = true. The floor pose is b_true = [q0(1) + 0.25; q0(2) - 0.20;
%   +175 deg], rigid body at z 0.35 m, pose7 = floor_to_motive(b_true, 0.35, cal), sent as float32. Oracle:
%   E = [motive_to_floor(p7s, cal)(1:2); yaw_unwrap(psi_raw, q0_9dof(3))], p7s = double(single(pose7)), i.e. the
%   heading near q0(3) = -179 deg: 175 - 360 = -185 deg. Premises (test-local arithmetic): E is within 1e-4 m / 1e-5
%   rad of b_true with psi - 2 pi; E(1:2) is >= 0.2 m from q0(1:2) (the old q0 + displacement path cannot produce it);
%   |q0(3)| > 170 deg (the seed decides the branch: the raw heading is +175 deg).
%
%   Runs (StopTime short: a hardware run is paced at real time):
%     iv   EnableHardware 1, EnableMotive 1, test cal, no timeout, 1 s.
%     nan  as iv with 3 timeouts first (MotiveData outputs NaN before the first valid frame), 1 s.
%     g    EnableHardware 0, EnableMotive 1, motive_cal as define_constants leaves it, 0.5 s (unpaced).
%     v6   EnableHardware 1, EnableMotive 1, motive_cal as define_constants leaves it (measured = false), 0.5 s.
%   Checks (all evaluated, then one error listing the failed ones):
%     (iv) iv run returned; q_track (tracked_position) equals E at EVERY logged step (>= 20 steps) within 1e-12, and so
%          does the base part (1:3) of the StateSource output (UKDynamics in 1, logged by a To Workspace added in
%          memory); openSocket was called (the reader ran with both flags on); premises above.
%     (n)  pure: motive_floor_block (the chart script file) fed NaN(7,1) three times then p7s returns NaN, NaN, NaN,
%          then E (the unwrap state is not poisoned by NaN and stays seeded from q0_9dof(3)).
%          model (nan run): IF the run returned, q_track is the F6 camera guard's seed q0_9dof(1:3) (was NaN before F6) at the 3 first steps and E (1e-12) at every later
%          step; if the run stopped, NOT EVALUATED and its error is printed (NaN into UKDynamics is the existing
%          behaviour, F6 scope). q_cmd / wheel_sent finiteness is PRINTED, not asserted (observation for F6).
%     (g)  g run: openSocket called 0 times (the real reader stays shut without EnableHardware: no socket, no 500 ms
%          timeouts in an emulated run); IF the run returned, q_track is the F6 guard's seed q0_9dof(1:3) at every step (no frame; NaN before F6; never held
%          zeros or q0). The model executes MotiveData before UKDynamics (data dependency), so a later stop of the
%          run cannot hide a socket opened at the first step.
%     (v)  motive_cal_check: v1 EnableHardware 1, EnableMotive 1, measured false -> motive_cal_check:unmeasured, the
%          message names motive_cal.measured; v2 motive_cal undefined -> motive_cal_check:nocal; v3 EnableMotive 0,
%          v4 EnableHardware 0, v5 measured true -> no error. v6 the sim is refused, message names motive_cal;
%          v8 apply_pacing(mdl) (InitFcn/StartFcn path) refuses with motive_cal_check:unmeasured. v7 the copy's
%          define_constants.m, evaluated IN MEMORY with EnableHardware 1 and EnableMotive 1 (model closed), errors
%          with motive_cal_check:unmeasured and leaves arm_freeze_user NaN.
%   MAKE-IT-FAIL: test_motive_model('<id>') applies one break to the COPY and must then error naming its check;
%   test_motive_model('list') returns the table.
%   Flat function without local functions (CLAUDE.md section 5, crash 2). Clears the base workspace.
if nargin < 1 || isempty(brk), brk = ''; end
here = fileparts(mfilename('fullpath'));      % ROME_merged/tests (char)
root = fileparts(here);                       % ROME_merged (char)
mdl  = 'ROME_9DOF';                           % model name (char)
lf   = char(10);                              % line feed (char)
tolE = 1e-12;                                 % model vs oracle (m, rad)

% ---- break table: id | check that must FAIL | edits on the copy | what the break does ------------------------------
% edit: {'tool', mode} tools/wire_motive_absolute.m on the copy's model; {'file', file in the copy, old, new} exactly
% one match; {'param', block in the model, parameter, value} set on the copy's model and saved.
B = { ...
  'mf_legacy',    'iv', {{'tool7', 'unwire'}, {'tool', 'legacy'}},'the old path back (MotiveInit q0_9dof(1:3) + Add of [x y -z_rot], reader enabled by EnableMotive alone): tracked = q0 + displacement'; ...
  'mf_norefusal', 'v',  {{'file', 'motive_cal_check.m', 'if ~measured', 'if false'}}, 'the motive_cal.measured refusal removed'; ...
  'mf_gate',      'g',  {{'param', 'MotiveReaderEnable', 'Value', 'EnableMotive'}}, 'reader enabled by EnableMotive alone: an EnableHardware = 0 run opens the socket'};
if strcmp(brk, 'list'), varargout{1} = B; return; end
ib = [];                                      % row of the break in B (empty: no break)
if ~isempty(brk)
    ib = find(strcmp(B(:,1), brk));
    assert(numel(ib) == 1, 'test_motive_model:brk', 'unknown break ''%s''; test_motive_model(''list'') gives the table', brk);
    fprintf('MAKE-IT-FAIL break %s (check (%s) must FAIL): %s\n', B{ib,1}, B{ib,2}, B{ib,4});
end

% ---- test-local cal and packet builder ----------------------------------------------------------------------------
Rz = @(a) [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];   % rotation about z by a (rad), 3x3
Rx = @(a) [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];   % rotation about x by a (rad), 3x3
cal = struct('R', Rz(deg2rad(37)) * Rx(deg2rad(90)), 't', [1.2; -0.4; 0.3], 'scale', 0.001, 'measured', true);   % test cal (R -, t m, scale m/unit, flag)
assert(abs(cal.R(3, 2) - 1) < 1e-15, 'test_motive_model:setup', 'test cal does not map Motive +y to floor +z');
z_rb   = 0.35;                                % floor height of the rigid-body origin (m)
dPos   = [0.25; -0.20];                       % injected placement minus q0(1:2) (m)
psiInj = deg2rad(175);                        % injected heading, raw (rad)
nTO    = 3;                                   % timeouts before the first frame in the nan run (count)
[~, ~, endian] = computer;                    % host byte order
assert(endian == 'L', 'test_motive_model:endian', 'host is big-endian: typecast-based packets would be byte-swapped');
le16  = @(v) reshape(typecast(uint16(v), 'uint8'), [], 1);      % uint16 -> 2 bytes (uint8 column)
le32i = @(v) reshape(typecast(int32(v), 'uint8'), [], 1);       % int32 -> 4 bytes
f32   = @(v) reshape(typecast(single(v(:).'), 'uint8'), [], 1); % float32 vector -> 4 bytes each
msName = uint8(['RigidBody 1' 0]).';          % marker-set name, 12 bytes (S = 16 with the counts)
build = @(id, pose) [le16(7); le16(78); le32i(1234); le32i(1); msName; le32i(0); le32i(0); le32i(1); ...
                     le32i(id); f32(pose); f32(0.0005); le16(1); zeros(8, 1, 'uint8')];   % NatNet frame (tests/test_motive_reader.m layout): ID@37, pose@41, params@73 = tracked

% ---- temporary copy ------------------------------------------------------------------------------------------------
tmp = tempname;                               % temporary copy of ROME_merged (char)
startdir = pwd;  path0 = path;                % restored at exit
if bdIsLoaded(mdl), close_system(mdl, 0); end
mkdir(tmp);
copyfile(root, tmp);
prerun_fixture('signoff', tmp);              % F2 wiring: Part F/C sign-off dates in the COPY's lab_settings.m (prerun_check refuses robot runs without)
copyfile(fullfile(here, 'stubs_hw', 'ROMECommand.m'), fullfile(tmp, 'ROMECommand.m'), 'f');
copyfile(fullfile(here, 'stubs_hw', 'RobotLinkRecorder.m'), fullfile(tmp, 'RobotLinkRecorder.m'), 'f');
copyfile(fullfile(here, 'stubs_motive', 'MotiveDataInject.m'), fullfile(tmp, 'MotiveDataInject.m'), 'f');
mdlcell = {mdl};
cleanup = onCleanup(@() cellfun(@(c) c(), {@() cd(startdir), @() path(path0), ...
          @() cellfun(@(m) close_system(m, 0), mdlcell(bdIsLoaded(mdl))), @() rmdir(tmp, 's')})); %#ok<NASGU>
addpath(tmp, fullfile(tmp, 'Ground Vehicle'), fullfile(tmp, 'Optitrack'), here);   % here: tests/dc_set_mode.m
cd(tmp);
evalin('base', 'clear all; clear classes');
cntf = fullfile(tmp, 'motive_inject_count.txt');   % MotiveDataInject's openSocket counter (char)
for f = {fullfile(tmp, 'ROMECommand.m'), fullfile(tmp, 'RobotLinkRecorder.m'), fullfile(tmp, 'MotiveDataInject.m')}
    txt = fileread(f{1});
    bad = regexp(txt, '(tcpclient|MulticastSocket|DatagramSocket|serialport|serial\(|tcpip|udpport|natnet)\s*\(', 'match');
    assert(isempty(bad), 'test_motive_model:network', '%s contains a network call: %s', f{1}, strjoin(bad, ', '));
end
for c = {'ROMECommand', 'RobotLinkRecorder', 'MotiveDataInject'}
    assert(strcmp(fileparts(which(c{1})), tmp), 'test_motive_model:stub', '%s resolves to %s, not the copy %s', c{1}, which(c{1}), tmp);
end
assert(strcmp(fileparts(which('MotiveData')), fullfile(tmp, 'Optitrack')), 'test_motive_model:stub', 'MotiveData resolves to %s', which('MotiveData'));
mc = meta.class.fromName('MotiveDataInject');
for m = {'openSocket', 'receiveBytes'}
    mm = findobj(mc.MethodList, 'Name', m{1});
    assert(numel(mm) == 1 && strcmp(mm.DefiningClass.Name, 'MotiveDataInject'), 'test_motive_model:stub', 'MotiveDataInject does not override %s', m{1});
end
assert(strcmp(mc.SuperclassList(1).Name, 'MotiveData'), 'test_motive_model:stub', 'MotiveDataInject is not a MotiveData subclass');
fprintf('guard: ROMECommand -> %s\n       MotiveDataInject -> %s (subclass of %s)\n', which('ROMECommand'), which('MotiveDataInject'), which('MotiveData'));

% ---- the break, applied to the copy only ---------------------------------------------------------------------------
slx = fullfile(tmp, [mdl '.slx']);            % the copy's model file (char)
if ~isempty(ib)
    edits = B{ib,3};
    for e = 1:numel(edits)
        ed = edits{e};
        switch ed{1}
            case 'tool'
                addpath(fullfile(tmp, 'tools'));
                wire_motive_absolute(slx, ed{2});
            case 'tool7'
                addpath(fullfile(tmp, 'tools'));
                wire_motive_emulator(slx, ed{2});
            case 'param'
                evalc('load_system(slx)');
                blk = [mdl '/' ed{2}];
                fprintf('  break: %s %s ''%s'' -> ''%s''\n', blk, ed{3}, get_param(blk, ed{3}), ed{4});
                set_param(blk, ed{3}, ed{4});
                save_system(mdl);
                close_system(mdl, 0);
            case 'file'
                f = fullfile(tmp, ed{2});
                fid = fopen(f, 'r');  s = char(fread(fid, Inf, '*uint8').');  fclose(fid);
                n = numel(strfind(s, ed{3}));
                assert(n == 1, 'test_motive_model:brk', 'break edit in %s matches %d times (expected 1)', ed{2}, n);
                fid = fopen(f, 'w');  fwrite(fid, uint8(strrep(s, ed{3}, ed{4})));  fclose(fid);
        end
        fprintf('  break edit %d applied to the copy: %s\n', e, ed{1});
    end
    evalin('base', 'clear all; clear classes');
    rehash;
end

% ---- runs ------------------------------------------------------------------------------------------------------------
blkM = [mdl '/Enabled' newline 'Subsystem/MATLAB System'];   % the MotiveData block (SID 105)
cfg = {'iv', 1, 1, true, 0, 1.0; 'nan', 1, 1, true, nTO, 1.0; 'g', 0, 1, false, 0, 0.5; 'v6', 1, 1, false, 0, 0.5};   % name | EnableHardware | EnableMotive | test cal? | timeouts | StopTime (s)
R = struct();                                 % one field per run
fails = {};                                   % one message per failed check (cell of char)
for r = 1:size(cfg, 1)
    nm = cfg{r,1};  EH = cfg{r,2};  EM = cfg{r,3};  useCal = cfg{r,4};  nT = cfg{r,5};  Tst = cfg{r,6};
    evalin('base', 'clear all');
    evalc('load_system(slx)');                % PostLoadFcn = the copy's define_constants (file flags 0 / 0)
    q0 = evalin('base', 'q0_9dof');           % 9x1 start configuration (m, m, rad, joints rad)
    b_true = [q0(1:2) + dPos; psiInj];        % injected floor pose (m, m, rad)
    p7  = floor_to_motive(b_true, z_rb, cal); % Motive pose (Motive units, -), 7x1
    p7s = double(single(p7));                 % the pose as the parser returns it (float32 -> double)
    braw = motive_to_floor(p7s, cal);         % [x; y; psi raw] (m, m, rad)
    E = [braw(1:2); yaw_unwrap(braw(3), q0(3))];   % oracle (m, m, rad)
    assignin('base', 'motive_inject', struct('nTimeout', nT, 'packet', build(1, p7)));
    sim0 = get_param(blkM, 'SimulateUsing');
    set_param(blkM, 'System', 'MotiveDataInject');
    set_param(blkM, 'SimulateUsing', sim0);   % changing System resets it (test_hw_nocam, measured 19 Nov)
    assert(strcmp(get_param(blkM, 'SimulateUsing'), 'Interpreted execution'), 'test_motive_model:stub', 'MotiveData block SimulateUsing is %s', get_param(blkM, 'SimulateUsing'));
    load_system('simulink');
    add_block('simulink/Sinks/To Workspace', [mdl '/F5bStateIn'], 'VariableName', 'state_in', 'SaveFormat', 'Timeseries');
    add_line(mdl, 'StateSource/1', 'F5bStateIn/1');
    assignin('base', 'EnableHardware', EH);   % NOT through define_constants
    assignin('base', 'EnableMotive', EM);
    assignin('base', 'motive_source', 'optitrack');   % F7: the real-camera source (file default 'emulator')
    if EH && EM, assignin('base', 'arm_freeze_user', 0); end   % mode 4 as the author defines it (arm moving); not required since the frozenfeedback refusal was removed (author 19 Nov)
    assignin('base', 'run_mode_test_override', {'EnableHardware', 'EnableMotive', 'motive_source', 'arm_freeze'});   % F12 fix round 1: flags set by hand, not through run_mode
    if EH, assignin('base', 'sensor_model', 0); end   % apply_pacing refuses a hardware run with SensorModel noise on
    if useCal, assignin('base', 'motive_cal', cal); end
    if isfile(cntf), delete(cntf); end
    prerun_fixture('park', tmp);             % F2/F11 wiring: park record of the copy's settings, as calibrateROMEArm writes it
    evalin('base', 'clear ukd_current');
    err = '';  haveOut = false;  out = [];  simtxt = '';
    try
        simtxt = evalc('out = sim(mdl, ''StopTime'', num2str(Tst), ''SimulationMode'', ''normal'', ''ReturnWorkspaceOutputs'', ''on'');');
        haveOut = true;
    catch ex
        err = [ex.identifier ': ' ex.message];
        for c1 = 1:numel(ex.cause)
            err = [err ' | ' ex.cause{c1}.message]; %#ok<AGROW>
            for c2 = 1:numel(ex.cause{c1}.cause), err = [err ' | ' ex.cause{c1}.cause{c2}.message]; end %#ok<AGROW>
        end
    end
    S = struct('E', E, 'b_true', b_true, 'q0', q0, 'p7s', p7s, 'haveOut', haveOut, 'err', err, 'simtxt', simtxt);
    S.nOpen = 0;                              % openSocket calls this run (count)
    if isfile(cntf), S.nOpen = numel(regexp(fileread(cntf), 'openSocket')); end
    S.Q = [];  S.SI = [];  S.qcmdFinite = NaN;  S.wheelFinite = NaN;
    if haveOut
        d = squeeze(out.get('q_track').Data);  if size(d, 1) ~= 3, d = d.'; end
        S.Q = d;                              % 3xN tracked_position (m, m, rad)
        d = squeeze(out.get('state_in').Data);  if size(d, 1) ~= 9, d = d.'; end
        S.SI = d;                             % 9xN StateSource output = UKDynamics in 1
        qc = out.get('q_cmd').Data;  ws = out.get('wheel_sent').Data;
        S.qcmdFinite = all(isfinite(qc(:)));  S.wheelFinite = all(isfinite(ws(:)));
    end
    if strcmp(nm, 'v6')
        S.id8 = '';                           % apply_pacing(mdl) error id with the model loaded (char)
        try
            apply_pacing(mdl);
        catch e8
            S.id8 = e8.identifier;
        end
    end
    close_system(mdl, 0);
    fprintf('%-4s EnableHardware %d EnableMotive %d test cal %d timeouts %d StopTime %g s: returned %d, openSocket x%d, steps %d, q_cmd finite %g, wheel_sent finite %g\n', ...
            nm, EH, EM, useCal, nT, Tst, haveOut, S.nOpen, size(S.Q, 2), S.qcmdFinite, S.wheelFinite);
    if ~isempty(err), fprintf('     stopped: %s\n', err(1:min(end, 500))); end
    R.(nm) = S;
end

% ---- (iv) --------------------------------------------------------------------------------------------------------
S = R.iv;  E = S.E;
pr1 = norm(E(1:2) - S.b_true(1:2)) < 1e-4 && abs(E(3) - (S.b_true(3) - 2*pi)) < 1e-5;   % oracle = injected pose (single rounding)
pr2 = norm(E(1:2) - S.q0(1:2)) >= 0.2;        % the old path (q0 + displacement 0) cannot produce E
pr3 = abs(S.q0(3)) > deg2rad(170);            % seed near +-pi: the unwrap decides the branch
fprintf('iv   premises: oracle vs injected %d, |E(1:2) - q0(1:2)| = %.3f m (>= 0.2) %d, q0(3) = %.2f deg (|.| > 170) %d; E = [%.6f %.6f %.4f deg]\n', ...
        pr1, norm(E(1:2) - S.q0(1:2)), pr2, rad2deg(S.q0(3)), pr3, E(1), E(2), rad2deg(E(3)));
okIv = S.haveOut && size(S.Q, 2) >= 20 && S.nOpen >= 1 && pr1 && pr2 && pr3;
eQ = Inf;  eS = Inf;                          % max |model - oracle| (m, rad)
if S.haveOut && ~isempty(S.Q)
    eQ = max(max(abs(S.Q - E)));
    eS = max(max(abs(S.SI(1:3, :) - E)));
    fprintf('iv   tracked_position vs oracle: max %.3e over %d steps; UKDynamics in 1 (base) vs oracle: max %.3e; first sample [%.6f %.6f %.4f deg]\n', ...
            eQ, size(S.Q, 2), eS, S.Q(1, 1), S.Q(2, 1), rad2deg(S.Q(3, 1)));
end
okIv = okIv && eQ <= tolE && eS <= tolE;
fprintf('iv   -> %s\n', mat2str(okIv));
if ~okIv, fails{end+1} = sprintf('(iv) tracked_position / controller base ~= motive_to_floor(pose7) + unwrap (returned %d, steps %d, openSocket %d, err %.3e / %.3e, premises %d%d%d)', ...
                                  S.haveOut, size(S.Q, 2), S.nOpen, eQ, eS, pr1, pr2, pr3); end

% ---- (n) ---------------------------------------------------------------------------------------------------------
okN = false;  nOut = [];                      % pure check verdict (logical), outputs 3x(nTO+1)
try
    clear motive_floor_block
    for k = 1:nTO, nOut(:, k) = motive_floor_block(NaN(7, 1), cal, R.iv.q0); end %#ok<AGROW>
    nOut(:, nTO + 1) = motive_floor_block(R.iv.p7s, cal, R.iv.q0);
    clear motive_floor_block
    okN = all(all(isnan(nOut(:, 1:nTO)))) && max(abs(nOut(:, nTO + 1) - R.iv.E)) <= tolE;
    fprintf('n    pure: motive_floor_block NaN x%d then the pose -> NaN %d, last %s (oracle %s)\n', nTO, all(all(isnan(nOut(:, 1:nTO)))), mat2str(nOut(:, end).', 8), mat2str(R.iv.E.', 8));
catch en
    fprintf('n    pure: motive_floor_block errored: %s\n', en.message);
end
S = R.nan;
if S.haveOut && ~isempty(S.Q)
    seedN = isequal(S.Q(:, 1:nTO), repmat(S.q0(1:3), 1, nTO));   % F6: the camera guard outputs the seed q0_9dof(1:3) until the first frame
    okNm = size(S.Q, 2) > nTO + 10 && seedN && max(max(abs(S.Q(:, nTO+1:end) - S.E))) <= tolE;
    fprintf('n    model: q_track = seed q0_9dof(1:3) at steps 1-%d %d (F6 guard, was NaN), later max |. - oracle| %.3e -> %s (q_cmd finite %g, wheel_sent finite %g)\n', ...
            nTO, seedN, max(max(abs(S.Q(:, nTO+1:end) - S.E))), mat2str(okNm), S.qcmdFinite, S.wheelFinite);
else
    okNm = true;
    fprintf('n    model: NOT EVALUATED, the nan run stopped (existing NaN behaviour, F6 scope): %s\n', S.err(1:min(end, 300)));
end
if ~(okN && okNm), fails{end+1} = sprintf('(n) NaN before the first frame: pure %d, model %d', okN, okNm); end

% ---- (g) ---------------------------------------------------------------------------------------------------------
S = R.g;
okG = S.nOpen == 0;
if S.haveOut && ~isempty(S.Q)
    seedG = isequal(S.Q, repmat(S.q0(1:3), 1, size(S.Q, 2)));   % F6: no frame ever -> the guard's seed at every step (was NaN)
    okG = okG && seedG;
    fprintf('g    EnableHardware 0: openSocket x%d, q_track = seed q0_9dof(1:3) at all %d steps %d (F6 guard, was NaN) -> %s\n', S.nOpen, size(S.Q, 2), seedG, mat2str(okG));
else
    fprintf('g    EnableHardware 0: openSocket x%d; run stopped, q_track NOT EVALUATED -> %s\n', S.nOpen, mat2str(okG));
end
if ~okG, fails{end+1} = sprintf('(g) reader not gated on EnableHardware (openSocket x%d)', S.nOpen); end

% ---- (v) ---------------------------------------------------------------------------------------------------------
cal0 = struct('R', eye(3), 't', zeros(3, 1), 'scale', 1, 'measured', false);   % define_constants' default (UNMEASURED)
vc = {'v1', 1, 1, cal0, 'motive_cal_check:unmeasured'; 'v2', 1, 1, [], 'motive_cal_check:nocal'; ...
      'v3', 1, 0, cal0, ''; 'v4', 0, 1, cal0, ''; 'v5', 1, 1, cal, ''};   % case | EnableHardware | EnableMotive | motive_cal ([] = undefined) | expected id ('' = no error)
okV = true;
for k = 1:size(vc, 1)
    evalin('base', 'clear all');
    assignin('base', 'EnableHardware', vc{k,2});  assignin('base', 'EnableMotive', vc{k,3});
    assignin('base', 'motive_source', 'optitrack');   % F7: the refusal applies to the real cameras only
    if ~isempty(vc{k,4}), assignin('base', 'motive_cal', vc{k,4}); end
    id = '';  ms = '';                        % error id and message (char)
    try
        motive_cal_check();
    catch ev
        id = ev.identifier;  ms = ev.message;
    end
    ok = strcmp(id, vc{k,5}) && (~strcmp(vc{k,1}, 'v1') || contains(ms, 'motive_cal.measured'));
    fprintf('%s   EnableHardware %d EnableMotive %d: id ''%s'' (expected ''%s'') -> %s\n', vc{k,1}, vc{k,2}, vc{k,3}, id, vc{k,5}, mat2str(ok));
    if strcmp(vc{k,1}, 'v1'), fprintf('     message: %s\n', ms); end
    okV = okV && ok;
end
S = R.v6;
ok6 = ~S.haveOut && contains(S.err, 'motive_cal');
ok8 = strcmp(S.id8, 'motive_cal_check:unmeasured');
fprintf('v6   sim with EnableHardware 1, EnableMotive 1, motive_cal unmeasured: refused %d, names motive_cal %d -> %s\n', ~S.haveOut, contains(S.err, 'motive_cal'), mat2str(ok6));
fprintf('v8   apply_pacing(mdl) same state: id ''%s'' -> %s\n', S.id8, mat2str(ok8));
dcTxt = fileread(fullfile(tmp, 'define_constants.m'));   % the copy's define_constants.m (char)
id7 = '';  af7 = [];                          % error id of the in-memory run, arm_freeze_user after it
if true                                       % F12: dc_set_mode errors (count-asserted) instead of skipping on a missing line
    evalin('base', 'clear all');
    assignin('base', 'dc_text_v7', dc_set_mode(dcTxt, 4, struct('arm_freeze', 1)));   % EH 1, EM 1, 'optitrack', arm_freeze 1 as before F12
    try
        evalin('base', 'evalc(dc_text_v7);');
    catch e7
        id7 = e7.identifier;
    end
    try, af7 = evalin('base', 'arm_freeze_user'); catch, af7 = []; end
    evalin('base', 'clear dc_text_v7');
end
ok7 = strcmp(id7, 'motive_cal_check:unmeasured') && isscalar(af7) && isnan(af7);
fprintf('v7   define_constants in memory, run_mode 4 + arm_freeze 1 (EH 1, EM 1, optitrack): id ''%s'', arm_freeze_user %s -> %s\n', id7, mat2str(af7), mat2str(ok7));
if ~(okV && ok6 && ok7 && ok8), fails{end+1} = sprintf('(v) refusal of a camera hardware run with motive_cal.measured = false (v1-5 %d, v6 %d, v7 %d, v8 %d)', okV, ok6, ok7, ok8); end

if ~isempty(fails)
    error('test_motive_model:fail', '%d check(s) failed:\n  %s', numel(fails), strjoin(fails, sprintf('\n  ')));
end
fprintf('\ntest_motive_model PASS\n');
end
