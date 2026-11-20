function wire_motive_emulator(slxFile, mode)
%WIRE_MOTIVE_EMULATOR  Camera emulator and camera-source switch in ROME_9DOF.slx (focus plan F7, mode 2). Re-runnable.
%   wire_motive_emulator()                   wires ROME_merged/ROME_9DOF.slx (this file's ../ROME_9DOF.slx)
%   wire_motive_emulator(slxFile)            wires the given model file (e.g. a temp copy)
%   wire_motive_emulator(slxFile, 'unwire')  removes every F7 block and restores the F5b model exactly
%                                            (tools/wire_motive_absolute.m 'wire' state); wire_motive_absolute refuses
%                                            to run on a model that still has the F7 blocks
%
%   'wire' builds, after tearing down any earlier F7 blocks (so running it twice gives the same model):
%   inside 'Enabled Subsystem' (SID 100, still enabled by Constant1 = EnableMotive)
%     emu_pose7   Inport 2: the emulated camera pose from the root block MotiveEmulator
%     PoseSelect  Constant 'motive_source_check(motive_source)': 1 'emulator', 0 'optitrack', compile error otherwise
%     PoseSource  Switch (u2 > 0.5): in 1 emu_pose7, in 2 PoseSelect, in 3 MotiveData output 7 (pose7) -> MotiveFloor
%                 input 1. Downstream (MotiveFloor = motive_to_floor + yaw_unwrap, Reshape, tracked_position,
%                 MuxTracked, StateSource, UKDynamics) is the F5b chain, unchanged: ONE camera pipeline, two sources.
%   at the root
%     MotiveEmulator      MATLAB Function, script = Optitrack/motive_emulator_block.m verbatim: floor_to_motive of the
%                         StateDelay base (the model's predicted pose; MocapNoise added only when motive_emulate_noise
%                         and sensor_model are both on, simulation only). Inputs StateDelay/1, MocapNoise/1; Parameters
%                         motive_cal, mocap_sigma, sensor_model, motive_emulate_noise (non-tunable). Output -> subsystem
%                         input 2.
%     MotiveReaderEnable  Value 'double(EnableMotive ~= 0 && EnableHardware ~= 0 && ~motive_source_check(motive_source))':
%                         the real MotiveData reader (socket) runs only on the robot, with cameras, with 'optitrack'.
%     JointSource         Switch (u2 > 0.5) -> MuxTracked input 2 (the joints the controller reads with EnableMotive = 1):
%                         in 1 TelemetryDelay (real arm telemetry, ROMECommand output 2 in rad), in 2 JointFromTelemetry
%                         = Constant 'double(EnableHardware ~= 0 && arm_freeze_user == 0)', in 3 SimJoints = Selector 4:9
%                         of SensorModel output. Telemetry only on the robot with the arm moving (mode 3/4); in simulation
%                         ROMECommand has no telemetry, and with the arm frozen (mode 2) the arm never follows the
%                         controller, which read the parked telemetry and wound up (1657 deg, F7 run 1). Author 19 Nov:
%                         "Arm from model if frozen". arm_freeze_user, because ROMECommand zeroes arm_freeze at setup.
%     TelemetryDelay      InitialCondition 'q0_9dof(4:9)' (was 'arm_home'): the arm is parked at the start posture
%                         (calibrateROMEArm), and UKDynamics seeds its integrator from its input at the first step, so
%                         arm_home there made the first joint command start from arm_home, not from where the arm is.
%   'unwire' deletes those blocks and restores: MotiveData/7 -> MotiveFloor/1, TelemetryDelay/1 -> MuxTracked/2,
%   MotiveReaderEnable 'double(EnableMotive ~= 0 && EnableHardware ~= 0)', TelemetryDelay IC 'arm_home'.
%
%   The model is compiled once, saved, closed and RE-READ from disk; then asserted and printed with SIDs: every
%   connection of the mode, the block set, block parameters, the MotiveEmulator script = the file and its data scopes,
%   MotiveFloor script = Optitrack/motive_floor_block.m (F5b), the other existing MATLAB Function charts unchanged
%   (SHA-256 before / after, CLAUDE.md section 27), the InitFcn unchanged, and the F1 arm-freeze wiring intact
%   (arm_freeze_banner(mdl), which refuses any other wiring). Errors on any mismatch.
%   Flat function without local functions (CLAUDE.md section 5, crash 2).
if nargin < 1 || isempty(slxFile)
    slxFile = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'ROME_9DOF.slx');   % ROME_merged/ROME_9DOF.slx (char)
end
if nargin < 2 || isempty(mode), mode = 'wire'; end
assert(any(strcmp(mode, {'wire', 'unwire'})), 'wire_motive_emulator:mode', 'mode must be wire or unwire, not %s', mode);
assert(isfile(slxFile), 'wire_motive_emulator:nofile', 'model file %s does not exist', slxFile);
root = fileparts(slxFile);                     % folder of the model, the ROME_merged layout (char)
[~, mdl] = fileparts(slxFile);                 % model name (char)
path0 = path;                                  % MATLAB path at entry, restored at exit (char)
addpath(root, fullfile(root, 'Ground Vehicle'), fullfile(root, 'Optitrack'));
cleanup = onCleanup(@() path(path0)); %#ok<NASGU>
emuFile = fullfile(root, 'Optitrack', 'motive_emulator_block.m');   % the MotiveEmulator script (char)
flrFile = fullfile(root, 'Optitrack', 'motive_floor_block.m');      % the MotiveFloor script, F5b (char)
assert(isfile(emuFile) && isfile(flrFile), 'wire_motive_emulator:nochart', 'chart script %s or %s missing', emuFile, flrFile);
fid = fopen(emuFile, 'r');  emuTxt = char(fread(fid, Inf, '*uint8').');  fclose(fid);
emuTxt = strrep(emuTxt, char([13 10]), char(10));   % LF line endings (char)
fid = fopen(flrFile, 'r');  flrTxt = char(fread(fid, Inf, '*uint8').');  fclose(fid);
flrTxt = strrep(flrTxt, char([13 10]), char(10));
ssN = ['Enabled' newline 'Subsystem'];         % subsystem name (char)
SS = [mdl '/' ssN];                            % subsystem path (char)
MD = [SS '/MATLAB System'];  FL = [SS '/MotiveFloor'];  PS = [SS '/PoseSource'];  PE = [SS '/PoseSelect'];  EI = [SS '/emu_pose7'];
EM = [mdl '/MotiveEmulator'];  JS = [mdl '/JointSource'];  JT = [mdl '/JointFromTelemetry'];  SJ = [mdl '/SimJoints'];
TD = [mdl '/TelemetryDelay'];  SD = [mdl '/StateDelay'];  MN = [mdl '/MocapNoise'];  SM = [mdl '/SensorModel'];
MT = [mdl '/MuxTracked'];  RE = [mdl '/MotiveReaderEnable'];  ST = [mdl '/StateSource'];  UK = [mdl '/UKDynamics'];  UM = [mdl '/UseMotive'];
reF5b = 'double(EnableMotive ~= 0 && EnableHardware ~= 0)';   % F5b reader gate (char)
reF7  = 'double(EnableMotive ~= 0 && EnableHardware ~= 0 && ~motive_source_check(motive_source))';   % F7 reader gate (char)
psVal = 'motive_source_check(motive_source)';  % PoseSelect value (char)
jtVal = 'double(EnableHardware ~= 0 && arm_freeze_user == 0)';   % JointFromTelemetry value (char): telemetry only on the robot with the arm moving
pacBan = 'apply_pacing(bdroot); arm_freeze_banner(bdroot)';   % InitFcn set by F1 (char), must stay
sha = @(s) lower(reshape(dec2hex(typecast(java.security.MessageDigest.getInstance('SHA-256').digest(uint8(unicode2native(s, 'UTF-8'))), 'uint8'), 2).', 1, []));   % SHA-256 of a char (hex char)
exists = @(b) getSimulinkBlockHandle(b) ~= -1;  % block present in the loaded model (logical)
srcOf = @(b) get_param(b, 'PortConnectivity');   % port connectivity of a block (struct array)
fprintf('wire_motive_emulator: mode %s on %s\n', mode, slxFile);

% ---- load (PostLoadFcn = define_constants; its card is captured, not printed) -------------------------------------
if bdIsLoaded(mdl), close_system(mdl, 0); end
evalc('load_system(slxFile)');
assert(strcmp(get_param(mdl, 'FileName'), slxFile), 'wire_motive_emulator:file', 'loaded %s, expected %s', get_param(mdl, 'FileName'), slxFile);
assert(exists(FL) && exists(MD) && exists(RE), 'wire_motive_emulator:f5b', 'the F5b camera path (MotiveFloor, MotiveData, MotiveReaderEnable) is not in the model: run tools/wire_motive_absolute.m first');
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart');
ch = ch(strncmp({ch.Path}, [mdl '/'], numel(mdl) + 1) & ~strcmp({ch.Path}, EM));   % existing charts, MotiveEmulator excluded
chName = sort({ch.Path});                      % chart paths (cell of char)
h0 = cellfun(@(p) sha(rt.find('-isa', 'Stateflow.EMChart', 'Path', p).Script), chName, 'UniformOutput', false);
fprintf('charts before: %s\n', strjoin(cellfun(@(p, h) sprintf('%s %s', strrep(p, newline, ' '), h(1:16)), chName, h0, 'UniformOutput', false), '; '));
initFcn0 = get_param(mdl, 'InitFcn');          % InitFcn before (char), must not change
assert(strcmp(initFcn0, pacBan), 'wire_motive_emulator:initfcn', 'InitFcn is ''%s'', expected the F1 ''%s''', initFcn0, pacBan);

% ---- tear down: every F7 connection (and the two F5b ones the build replaces), then the F7 blocks ------------------
% {system, dst block, dst input, src block, src output}: deleted when the dst input is fed by exactly that source
conn = {mdl, EM, 1, SD, 1;  mdl, EM, 2, MN, 1;  mdl, SS, 2, EM, 1;  mdl, JS, 1, TD, 1;  mdl, JS, 2, JT, 1; ...
        mdl, SJ, 1, SM, 1;  mdl, JS, 3, SJ, 1;  mdl, MT, 2, JS, 1;  mdl, MT, 2, TD, 1; ...
        SS, PS, 1, EI, 1;  SS, PS, 2, PE, 1;  SS, PS, 3, MD, 7;  SS, FL, 1, PS, 1;  SS, FL, 1, MD, 7};
nDel = 0;                                      % lines deleted (count)
for k = 1:size(conn, 1)
    [sy, db, dp, sb, sp] = conn{k,:};
    if ~(exists(db) && exists(sb)), continue; end
    p = srcOf(db);  p = p(arrayfun(@(q) ~isempty(str2double(q.Type)) && ~isnan(str2double(q.Type)), p));   % numbered inputs only
    if numel(p) < dp, continue; end
    s = p(dp);
    if ~isempty(s.SrcBlock) && s.SrcBlock ~= -1 && s.SrcBlock == get_param(sb, 'Handle') && s.SrcPort + 1 == sp
        delete_line(sy, sprintf('%s/%d', get_param(sb, 'Name'), sp), sprintf('%s/%d', get_param(db, 'Name'), dp));
        nDel = nDel + 1;
    end
end
for b = {EM, JS, JT, SJ, PS, PE, EI}
    if exists(b{1}), delete_block(b{1}); end
end
fprintf('teardown: %d lines deleted\n', nDel);
% nothing else may feed the inputs the build connects (unknown model state otherwise: stop before saving)
for b = {FL, MT}
    p = srcOf(b{1});
    if strcmp(b{1}, MT), s = p(2); else, s = p(1); end
    if ~(isempty(s.SrcBlock) || s.SrcBlock == -1)
        error('wire_motive_emulator:state', '%s is still fed by %s after teardown: unknown model state, NOT saved', ...
              strrep(b{1}, newline, ' '), getfullname(s.SrcBlock));
    end
end
nIn = numel(find_system(SS, 'SearchDepth', 1, 'BlockType', 'Inport'));   % subsystem Inports after teardown (count)
assert(nIn == 1, 'wire_motive_emulator:state', 'Enabled Subsystem has %d Inports after teardown (expected 1, reader_enable)', nIn);

% ---- build ---------------------------------------------------------------------------------------------------------
load_system('simulink');
if strcmp(mode, 'wire')
    add_block('simulink/Sources/In1', EI, 'Port', '2', 'Position', [130 -60 160 -46]);
    add_block('simulink/Sources/Constant', PE, 'Value', psVal, 'Position', [100 -20 220 0], ...
              'Description', 'F7: 1 motive_source ''emulator'', 0 ''optitrack'', error otherwise (motive_source_check.m)');
    add_block('simulink/Signal Routing/Switch', PS, 'Criteria', 'u2 > Threshold', 'Threshold', '0.5', 'Position', [300 -60 330 30], ...
              'Description', 'F7: camera source. u2 = PoseSelect > 0.5: the emulated pose (MotiveEmulator), else MotiveData pose7 (tools/wire_motive_emulator.m)');
    add_line(SS, 'emu_pose7/1', 'PoseSource/1', 'autorouting', 'smart');
    add_line(SS, 'PoseSelect/1', 'PoseSource/2', 'autorouting', 'smart');
    add_line(SS, 'MATLAB System/7', 'PoseSource/3', 'autorouting', 'smart');
    add_line(SS, 'PoseSource/1', 'MotiveFloor/1', 'autorouting', 'smart');
    add_block('simulink/User-Defined Functions/MATLAB Function', EM, 'Position', [1250 640 1400 700]);
    rt = sfroot;
    fc = rt.find('-isa', 'Stateflow.EMChart', 'Path', EM);   % the new chart object
    assert(numel(fc) == 1, 'wire_motive_emulator:chart', 'MotiveEmulator chart found %d times', numel(fc));
    fc.Script = emuTxt;
    want = {'q_pred', 'Input'; 'n', 'Input'; 'pose7', 'Output'; 'motive_cal', 'Parameter'; 'mocap_sigma', 'Parameter'; ...
            'sensor_model', 'Parameter'; 'motive_emulate_noise', 'Parameter'};   % data name | scope
    dl = fc.find('-isa', 'Stateflow.Data');
    for d = dl(:).'
        if ~any(strcmp(d.Name, want(:,1))), fprintf('  chart data %s (library default) deleted\n', d.Name); delete(d); end
    end
    for k = 1:size(want, 1)
        d = fc.find('-isa', 'Stateflow.Data', 'Name', want{k,1});
        if isempty(d)
            d = Stateflow.Data(fc);  d.Name = want{k,1};
            fprintf('  chart data %s created\n', want{k,1});
        end
        d.Scope = want{k,2};
        if strcmp(want{k,2}, 'Parameter')
            try
                d.Tunable = false;             % read once at compile: fixed during a run
            catch et
                fprintf('  chart data %s: Tunable not set (%s)\n', want{k,1}, et.message);
            end
        end
    end
    add_line(mdl, 'StateDelay/1', 'MotiveEmulator/1', 'autorouting', 'smart');
    add_line(mdl, 'MocapNoise/1', 'MotiveEmulator/2', 'autorouting', 'smart');
    add_line(mdl, 'MotiveEmulator/1', [ssN '/2'], 'autorouting', 'smart');
    add_block('simulink/Signal Routing/Selector', SJ, 'NumberOfDimensions', '1', 'IndexOptions', 'Index vector (dialog)', ...
              'Indices', '4:9', 'InputPortWidth', '9', 'Position', [1440 400 1480 430]);
    add_block('simulink/Sources/Constant', JT, 'Value', jtVal, 'Position', [1300 375 1420 395], ...
              'Description', 'F7: 1 on the robot with the arm moving (joints from arm telemetry), 0 in simulation or with the arm frozen (joints from the model, SensorModel)');
    add_block('simulink/Signal Routing/Switch', JS, 'Criteria', 'u2 > Threshold', 'Threshold', '0.5', 'Position', [1460 330 1490 420], ...
              'Description', 'F7: joints read with EnableMotive = 1: arm telemetry (TelemetryDelay) only when the robot is on and the arm moves (JointFromTelemetry); otherwise the model''s joints (SensorModel) (tools/wire_motive_emulator.m)');
    add_line(mdl, 'SensorModel/1', 'SimJoints/1', 'autorouting', 'smart');
    add_line(mdl, 'TelemetryDelay/1', 'JointSource/1', 'autorouting', 'smart');
    add_line(mdl, 'JointFromTelemetry/1', 'JointSource/2', 'autorouting', 'smart');
    add_line(mdl, 'SimJoints/1', 'JointSource/3', 'autorouting', 'smart');
    add_line(mdl, 'JointSource/1', 'MuxTracked/2', 'autorouting', 'smart');
    set_param(RE, 'Value', reF7, 'Description', 'F5b/F7: the real MotiveData reader runs only on the robot, with cameras, with motive_source ''optitrack'' (tools/wire_motive_emulator.m)');
    set_param(TD, 'InitialCondition', 'q0_9dof(4:9)');
else
    add_line(SS, 'MATLAB System/7', 'MotiveFloor/1', 'autorouting', 'smart');
    add_line(mdl, 'TelemetryDelay/1', 'MuxTracked/2', 'autorouting', 'smart');
    set_param(RE, 'Value', reF5b, 'Description', 'F5b: the real MotiveData reader runs only on the robot with cameras (tools/wire_motive_absolute.m). F7 hook (motive_source).');
    set_param(TD, 'InitialCondition', 'arm_home');
end

% ---- compile once (base workspace from the PostLoadFcn define_constants) ------------------------------------------
feval(mdl, [], [], [], 'compile');
try
    if exists(EM)
        d = get_param(EM, 'CompiledPortDimensions');
        fprintf('compiled: MotiveEmulator input dims %s, output dims %s\n', mat2str(d.Inport), mat2str(d.Outport));
        d = get_param(PS, 'CompiledPortDimensions');
        fprintf('compiled: PoseSource input dims %s, output dims %s\n', mat2str(d.Inport), mat2str(d.Outport));
        d = get_param(JS, 'CompiledPortDimensions');
        fprintf('compiled: JointSource input dims %s, output dims %s\n', mat2str(d.Inport), mat2str(d.Outport));
    end
    d = get_param(MT, 'CompiledPortDimensions');
    fprintf('compiled: MuxTracked input dims %s, output dims %s\n', mat2str(d.Inport), mat2str(d.Outport));
catch err
    feval(mdl, [], [], [], 'term');
    rethrow(err);
end
feval(mdl, [], [], [], 'term');
h1 = cellfun(@(p) sha(rt.find('-isa', 'Stateflow.EMChart', 'Path', p).Script), chName, 'UniformOutput', false);
assert(isequal(h0, h1), 'wire_motive_emulator:chart', 'an existing chart script changed while wiring; model NOT saved');
save_system(mdl);
close_system(mdl, 0);

% ---- re-read from disk and assert ----------------------------------------------------------------------------------
evalc('load_system(slxFile)');
rt = sfroot;
fprintf('re-read %s (%s)\n', slxFile, get_param(mdl, 'LastModifiedDate'));
bad = {};                                      % failed expectations (cell of char)
h2 = cellfun(@(p) sha(rt.find('-isa', 'Stateflow.EMChart', 'Path', p).Script), chName, 'UniformOutput', false);
fprintf('existing charts after re-read: %s -> unchanged %d\n', strjoin(cellfun(@(p, h) sprintf('%s %s', strrep(p, newline, ' '), h(1:16)), chName, h2, 'UniformOutput', false), '; '), isequal(h0, h2));
if ~isequal(h0, h2), bad{end+1} = 'existing chart scripts differ after re-read'; end
common = {MT, 1, SS, 1;  ST, 1, MT, 1;  UK, 1, ST, 1;  TD, 1, [mdl '/ArmDeg2Rad'], 1;  SS, 1, RE, 1;  MD, 1, [SS '/reader_enable'], 1;  ST, 3, SM, 1;  SM, 1, SD, 1;  ST, 2, UM, 1};   % dst, input, src, output (ST in 2 = UseMotive, the camera/model switch control)
switch mode
    case 'wire',   want = [{PS, 1, EI, 1;  PS, 2, PE, 1;  PS, 3, MD, 7;  FL, 1, PS, 1;  EM, 1, SD, 1;  EM, 2, MN, 1;  SS, 2, EM, 1; ...
                            JS, 1, TD, 1;  JS, 2, JT, 1;  JS, 3, SJ, 1;  SJ, 1, SM, 1;  MT, 2, JS, 1}; common];
    case 'unwire', want = [{FL, 1, MD, 7;  MT, 2, TD, 1}; common];
end
tags = {'WRONG', 'OK'};
nm = @(b) strrep(strrep(b, [mdl '/'], ''), newline, ' ');   % printable short path (char)
for k = 1:size(want, 1)
    got = '<none>';
    if exists(want{k,1})
        p = srcOf(want{k,1});  p = p(arrayfun(@(q) ~isnan(str2double(q.Type)), p));   % numbered inputs
        if numel(p) >= want{k,2}
            s = p(want{k,2});
            if ~isempty(s.SrcBlock) && s.SrcBlock ~= -1, got = sprintf('%s/%d', getfullname(s.SrcBlock), s.SrcPort + 1); end
        end
    end
    exp_ = sprintf('%s/%d', want{k,3}, want{k,4});
    sidD = '-';  sidS = '-';
    if exists(want{k,1}), sidD = get_param(want{k,1}, 'SID'); end
    if exists(want{k,3}), sidS = get_param(want{k,3}, 'SID'); end
    fprintf('  %-34s in %d <- %-34s (SID %s <- SID %s) %s\n', nm(want{k,1}), want{k,2}, nm(got), sidD, sidS, tags{1 + strcmp(got, exp_)});
    if ~strcmp(got, exp_), bad{end+1} = sprintf('%s in %d fed by %s, expected %s', nm(want{k,1}), want{k,2}, nm(got), nm(exp_)); end %#ok<AGROW>
end
blk = {EM, JS, JT, SJ, PS, PE, EI};            % blocks whose presence depends on the mode
present = cellfun(exists, blk);
wantPresent = repmat(strcmp(mode, 'wire'), 1, numel(blk));
fprintf('  present [MotiveEmulator JointSource JointFromTelemetry SimJoints PoseSource PoseSelect emu_pose7] = %s (expected %s)\n', mat2str(present), mat2str(wantPresent));
if ~isequal(present, wantPresent), bad{end+1} = 'block set differs from the mode'; end
nIn = numel(find_system(SS, 'SearchDepth', 1, 'BlockType', 'Inport'));   % subsystem Inports (count)
fprintf('  Enabled Subsystem Inports: %d (expected %d)\n', nIn, 1 + strcmp(mode, 'wire'));
if nIn ~= 1 + strcmp(mode, 'wire'), bad{end+1} = 'subsystem Inport count'; end
vals = {get_param(RE, 'Value'), get_param(TD, 'InitialCondition')};
switch mode
    case 'wire',   wv = {reF7, 'q0_9dof(4:9)'};
    case 'unwire', wv = {reF5b, 'arm_home'};
end
fprintf('  MotiveReaderEnable Value %s\n  TelemetryDelay InitialCondition %s\n', vals{:});
if ~isequal(vals, wv), bad{end+1} = 'MotiveReaderEnable Value or TelemetryDelay InitialCondition differ'; end
fc = rt.find('-isa', 'Stateflow.EMChart', 'Path', FL);
sF = strcmp(strrep(fc.Script, char([13 10]), char(10)), flrTxt);
fprintf('  MotiveFloor script = %s: %d (F5b, kept)\n', flrFile, sF);
if ~sF, bad{end+1} = 'MotiveFloor script differs from Optitrack/motive_floor_block.m'; end
if strcmp(mode, 'wire')
    pv = {get_param(PE, 'Value'), get_param(PS, 'Criteria'), get_param(PS, 'Threshold'), get_param(JT, 'Value'), ...
          get_param(JS, 'Criteria'), get_param(JS, 'Threshold'), get_param(SJ, 'Indices'), get_param(SJ, 'InputPortWidth')};
    pw = {psVal, 'u2 > Threshold', '0.5', jtVal, 'u2 > Threshold', '0.5', '4:9', '9'};
    fprintf('  PoseSelect %s | PoseSource %s %s | JointFromTelemetry %s | JointSource %s %s | SimJoints %s of %s\n', pv{:});
    if ~isequal(pv, pw), bad{end+1} = 'F7 block parameters differ'; end
    fc = rt.find('-isa', 'Stateflow.EMChart', 'Path', EM);
    scr = strrep(fc.Script, char([13 10]), char(10));   % chart script, LF (char)
    sameScr = strcmp(scr, emuTxt);
    fprintf('  MotiveEmulator script = %s: %d (%d / %d chars, SHA-256 %s)\n', emuFile, sameScr, numel(scr), numel(emuTxt), sha(scr));
    if ~sameScr, bad{end+1} = 'MotiveEmulator script differs from Optitrack/motive_emulator_block.m'; end
    dl = fc.find('-isa', 'Stateflow.Data');
    ds = sort(arrayfun(@(d) sprintf('%s:%s', d.Name, d.Scope), dl, 'UniformOutput', false));
    fprintf('  MotiveEmulator data: %s\n', strjoin(ds.', ', '));
    if ~isequal(ds(:).', sort({'motive_cal:Parameter', 'mocap_sigma:Parameter', 'motive_emulate_noise:Parameter', 'n:Input', ...
                                'pose7:Output', 'q_pred:Input', 'sensor_model:Parameter'})), bad{end+1} = 'MotiveEmulator data differ'; end
end
fprintf('  InitFcn: %s\n', get_param(mdl, 'InitFcn'));
if ~strcmp(get_param(mdl, 'InitFcn'), initFcn0), bad{end+1} = 'InitFcn changed'; end
try
    txt = evalc('arm_freeze_banner(mdl)');     % refuses (error) unless the F1 wiring is tools/wire_arm_freeze.m's
    fprintf('  F1 arm freeze wiring (arm_freeze_banner): %s', txt);
catch eb
    bad{end+1} = ['F1 arm-freeze wiring: ' eb.message];
end
close_system(mdl, 0);
if ~isempty(bad)
    error('wire_motive_emulator:verify', 'after re-read: %s', strjoin(bad, '; '));
end
fprintf('wire_motive_emulator: %s done, re-read and verified\n', mode);
end
