function wire_camera_guard(slxFile)
%WIRE_CAMERA_GUARD  NaN-safe camera path, arm-ready wait, command gate and safe stop in ROME_9DOF.slx (focus plan F6).
%   wire_camera_guard(slxFile)   wires the model file (char). NOT re-runnable: refuses a model that already has
%                                CameraGuard (wire a fresh copy of the F7 model instead).
%   Design: f6core-report.md wiring note, items 1-6. The logic is in the pure cores (camera_guard_core.m,
%   arm_ready_core.m, tested offline); the charts only keep state. Built:
%   inside 'Enabled Subsystem' (SID 100, enabled by EnableMotive)
%     emu_valid    Inport 3: EmuDropout's valid flag (emulator path)
%     ValidSource  Switch (u2 > 0.5 on PoseSelect, the same criterion as PoseSource): in 1 emu_valid, in 3 MotiveData
%                  output 8 (valid). With 'optitrack' the reader's own flag, with 'emulator' the emulator's
%     CameraGuard  MATLAB Function, script = Optitrack/camera_guard_block.m: MotiveFloor/1 -> CameraGuard -> Reshape
%                  and Discrete Derivative (replaces the direct line). Inputs: ValidSource, GuardClock (Clock),
%                  GuardLimits (rome_limit('camera_guard') = [dt_9dof; cam_wait_s; cam_dropout_max_s]), GuardSeed
%                  (q0_9dof(1:3))
%     cam_guard    Outport 3: CameraGuard output 2, [wait hold stop reason longest_dropout_s n_dropouts]; 0 while the
%                  subsystem is disabled (EnableMotive = 0)
%   at the root
%     EmuDropout   MATLAB Function, script = Optitrack/emulator_dropout_block.m, between MotiveEmulator and subsystem
%                  input 2; its valid -> subsystem input 3. Window from EmuDropoutWin (rome_limit('emulate_dropout'):
%                  [-1 -1] = none unless a test sets motive_emulate_dropout)
%     ArmReady     MATLAB Function, script = arm_ready_block.m. Inputs: ArmReadyDelay (UnitDelay of ROMECommand out 3,
%                  IC 0), ArmDegDelay (UnitDelay of ROMECommand out 2, IC rad2deg(arm_home(:))), Clock9, ArmWaitOn
%                  (double(EnableHardware ~= 0 && arm_freeze_user == 0 && EnableMotive ~= 0): the controller reads
%                  telemetry; the EnableMotive = 0 regression path does not), ArmLimits (rome_limit('arm_wait')),
%                  ArmHome (rad2deg(arm_home(:)))
%     RunGate      MATLAB Function, script = run_gate_block.m: inputs cam_guard, ArmReady; outputs gate, stop, reset
%     CmdGate      Product (element-wise): CommandGuard out 1 .* gate -> ROMECommand in 1 AND To Workspace wheel_sent
%                  (the logged command is the one sent)
%     Stop Simulation  input from StopAny = Constant3 (the panel's manual stop button, kept) OR RunGate stop
%     UKDynamics in 8  from RunGate reset (was Constant Reset = 0, deleted)
%     To Workspace cam_guard (cam_guard), To Workspace arm_gate (ArmReady output)
%   Not gated: the joint command (ROMECommand input 2 stays ArmFreezeSwitch, which arm_freeze_banner checks).
%   Checks after compile, save, close and reload: every new connection, chart scripts = files (LF), every other chart
%   unchanged (SHA-256 before / after), InitFcn / StopFcn unchanged, arm_freeze_banner(mdl) passes (F1 wiring intact).
%   Flat function without local functions (CLAUDE.md section 5, crash 2).
[root, mdl] = fileparts(slxFile);
assert(isfile(slxFile), 'wire_camera_guard:nofile', 'model file %s does not exist', slxFile);
path0 = path;  addpath(root, fullfile(root, 'Ground Vehicle'), fullfile(root, 'Optitrack'));
cleanup = onCleanup(@() path(path0)); %#ok<NASGU>
rd = @(f) strrep(fileread(f), char([13 10]), char(10));   % file text, LF (char)
SCR = {'CameraGuard', fullfile(root, 'Optitrack', 'camera_guard_block.m'); 'EmuDropout', fullfile(root, 'Optitrack', 'emulator_dropout_block.m'); ...
       'ArmReady', fullfile(root, 'arm_ready_block.m'); 'RunGate', fullfile(root, 'run_gate_block.m')};
txt = cell(size(SCR, 1), 1);
for k = 1:size(SCR, 1), txt{k} = rd(SCR{k,2}); end
sha = @(s) lower(reshape(dec2hex(typecast(java.security.MessageDigest.getInstance('SHA-256').digest(uint8(unicode2native(s, 'UTF-8'))), 'uint8'), 2).', 1, []));
ssN = ['Enabled' newline 'Subsystem'];  SS = [mdl '/' ssN];
if bdIsLoaded(mdl), close_system(mdl, 0); end
evalc('load_system(slxFile)');
assert(getSimulinkBlockHandle([SS '/CameraGuard']) == -1, 'wire_camera_guard:wired', 'CameraGuard already in %s: wire a fresh F7 model', slxFile);
for b = {[SS '/PoseSource'], [SS '/PoseSelect'], [SS '/MotiveFloor'], [mdl '/MotiveEmulator'], [mdl '/Reset'], [mdl '/Constant3']}
    assert(getSimulinkBlockHandle(b{1}) ~= -1, 'wire_camera_guard:f7', 'block %s missing: not the F7 model', strrep(b{1}, newline, ' '));
end
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart');  ch = ch(strncmp({ch.Path}, [mdl '/'], numel(mdl) + 1));
chName = sort({ch.Path});
h0 = cellfun(@(p) sha(rt.find('-isa', 'Stateflow.EMChart', 'Path', p).Script), chName, 'UniformOutput', false);
ini0 = get_param(mdl, 'InitFcn');  stp0 = get_param(mdl, 'StopFcn');
load_system('simulink');
% ---- inside the subsystem -------------------------------------------------------------------------------------------
delete_line(SS, 'MotiveFloor/1', 'Reshape/1');
delete_line(SS, 'MotiveFloor/1', 'Discrete Derivative/1');
add_block('simulink/Sources/In1', [SS '/emu_valid'], 'Port', '3', 'Position', [130 -110 160 -96]);
add_block('simulink/Signal Routing/Switch', [SS '/ValidSource'], 'Criteria', 'u2 > Threshold', 'Threshold', '0.5', ...
          'InputSameDT', 'off', 'OutDataTypeStr', 'double', 'Position', [300 -150 330 -80], ...
          'Description', 'F6: valid flag of the camera source, switched like PoseSource (tools/wire_camera_guard.m)');
add_block('simulink/Sources/Clock', [SS '/GuardClock'], 'Position', [380 -200 400 -180]);
add_block('simulink/Sources/Constant', [SS '/GuardLimits'], 'Value', 'rome_limit(''camera_guard'')', 'Position', [340 -240 460 -220]);
add_block('simulink/Sources/Constant', [SS '/GuardSeed'], 'Value', 'q0_9dof(1:3)', 'Position', [340 -280 460 -260]);
add_block('simulink/User-Defined Functions/MATLAB Function', [SS '/CameraGuard'], 'Position', [520 -200 660 -100]);
add_block('simulink/Sinks/Out1', [SS '/cam_guard'], 'Port', '3', 'Position', [720 -160 750 -146]);
rt = sfroot;
c = rt.find('-isa', 'Stateflow.EMChart', 'Path', [SS '/CameraGuard']);  c.Script = txt{1};
add_line(SS, 'emu_valid/1', 'ValidSource/1', 'autorouting', 'smart');
add_line(SS, 'PoseSelect/1', 'ValidSource/2', 'autorouting', 'smart');
add_line(SS, 'MATLAB System/8', 'ValidSource/3', 'autorouting', 'smart');
add_line(SS, 'MotiveFloor/1', 'CameraGuard/1', 'autorouting', 'smart');
add_line(SS, 'ValidSource/1', 'CameraGuard/2', 'autorouting', 'smart');
add_line(SS, 'GuardClock/1', 'CameraGuard/3', 'autorouting', 'smart');
add_line(SS, 'GuardLimits/1', 'CameraGuard/4', 'autorouting', 'smart');
add_line(SS, 'GuardSeed/1', 'CameraGuard/5', 'autorouting', 'smart');
add_line(SS, 'CameraGuard/1', 'Reshape/1', 'autorouting', 'smart');
add_line(SS, 'CameraGuard/1', 'Discrete Derivative/1', 'autorouting', 'smart');
add_line(SS, 'CameraGuard/2', 'cam_guard/1', 'autorouting', 'smart');
% ---- root: emulator dropout injection ------------------------------------------------------------------------------
delete_line(mdl, 'MotiveEmulator/1', [ssN '/2']);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/EmuDropout'], 'Position', [1450 640 1560 700]);
add_block('simulink/Sources/Constant', [mdl '/EmuDropoutWin'], 'Value', 'rome_limit(''emulate_dropout'')', 'VectorParams1D', 'off', 'Position', [1300 720 1420 740]);
c = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/EmuDropout']);  c.Script = txt{2};
add_line(mdl, 'MotiveEmulator/1', 'EmuDropout/1', 'autorouting', 'smart');
add_line(mdl, 'Clock9/1', 'EmuDropout/2', 'autorouting', 'smart');
add_line(mdl, 'EmuDropoutWin/1', 'EmuDropout/3', 'autorouting', 'smart');
add_line(mdl, 'EmuDropout/1', [ssN '/2'], 'autorouting', 'smart');
add_line(mdl, 'EmuDropout/2', [ssN '/3'], 'autorouting', 'smart');
% ---- root: arm-ready wait ------------------------------------------------------------------------------------------
add_block('simulink/Discrete/Unit Delay', [mdl '/ArmReadyDelay'], 'InitialCondition', '0', 'SampleTime', '-1', 'Position', [1260 200 1290 230]);
add_block('simulink/Discrete/Unit Delay', [mdl '/ArmDegDelay'], 'InitialCondition', 'rad2deg(arm_home(:))', 'SampleTime', '-1', 'Position', [1260 150 1290 180]);
add_block('simulink/Sources/Constant', [mdl '/ArmWaitOn'], 'Value', 'double(EnableHardware ~= 0 && arm_freeze_user == 0 && EnableMotive ~= 0)', 'Position', [1200 100 1320 120]);
add_block('simulink/Sources/Constant', [mdl '/ArmLimits'], 'Value', 'rome_limit(''arm_wait'')', 'Position', [1200 60 1320 80]);
add_block('simulink/Sources/Constant', [mdl '/ArmHome'], 'Value', 'rad2deg(arm_home(:))', 'Position', [1200 20 1320 40]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/ArmReady'], 'Position', [1360 40 1480 200]);
c = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/ArmReady']);  c.Script = txt{3};
add_line(mdl, 'ROMECommand/3', 'ArmReadyDelay/1', 'autorouting', 'smart');
add_line(mdl, 'ROMECommand/2', 'ArmDegDelay/1', 'autorouting', 'smart');
add_line(mdl, 'ArmReadyDelay/1', 'ArmReady/1', 'autorouting', 'smart');
add_line(mdl, 'ArmDegDelay/1', 'ArmReady/2', 'autorouting', 'smart');
add_line(mdl, 'Clock9/1', 'ArmReady/3', 'autorouting', 'smart');
add_line(mdl, 'ArmWaitOn/1', 'ArmReady/4', 'autorouting', 'smart');
add_line(mdl, 'ArmLimits/1', 'ArmReady/5', 'autorouting', 'smart');
add_line(mdl, 'ArmHome/1', 'ArmReady/6', 'autorouting', 'smart');
% ---- root: gate, stop, reset, logging ------------------------------------------------------------------------------
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/RunGate'], 'Position', [1560 40 1680 140]);
c = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/RunGate']);  c.Script = txt{4};
add_line(mdl, [ssN '/3'], 'RunGate/1', 'autorouting', 'smart');
add_line(mdl, 'ArmReady/1', 'RunGate/2', 'autorouting', 'smart');
delete_line(mdl, 'CommandGuard/1', 'ROMECommand/1');
delete_line(mdl, 'CommandGuard/1', 'To Workspace wheel_sent/1');
add_block('simulink/Math Operations/Product', [mdl '/CmdGate'], 'Inputs', '**', 'Multiplication', 'Element-wise(.*)', 'Position', [960 290 990 330], ...
          'Description', 'F6: wheel command x gate (0 while the camera waits or stopped, or the arm wait has not passed); tools/wire_camera_guard.m');
add_line(mdl, 'CommandGuard/1', 'CmdGate/1', 'autorouting', 'smart');
add_line(mdl, 'RunGate/1', 'CmdGate/2', 'autorouting', 'smart');
add_line(mdl, 'CmdGate/1', 'ROMECommand/1', 'autorouting', 'smart');
add_line(mdl, 'CmdGate/1', 'To Workspace wheel_sent/1', 'autorouting', 'smart');
delete_line(mdl, 'Constant3/1', 'Stop Simulation/1');   % Constant3 stays: the panel's Push Button is bound to its Value (manual stop)
add_block('simulink/Logic and Bit Operations/Logical Operator', [mdl '/StopAny'], 'Operator', 'OR', 'Inputs', '2', 'Position', [-5 230 10 270], ...
          'Description', 'F6: stop on the panel button (Constant3) OR the camera guard / arm wait (RunGate stop); tools/wire_camera_guard.m');
add_line(mdl, 'Constant3/1', 'StopAny/1', 'autorouting', 'smart');
add_line(mdl, 'RunGate/2', 'StopAny/2', 'autorouting', 'smart');
add_line(mdl, 'StopAny/1', 'Stop Simulation/1', 'autorouting', 'smart');
delete_line(mdl, 'Reset/1', 'UKDynamics/8');  delete_block([mdl '/Reset']);
add_line(mdl, 'RunGate/3', 'UKDynamics/8', 'autorouting', 'smart');
add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace cam_guard'], 'VariableName', 'cam_guard', 'SaveFormat', 'Timeseries', 'Position', [1700 160 1760 190]);
add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace arm_gate'], 'VariableName', 'arm_gate', 'SaveFormat', 'Timeseries', 'Position', [1700 200 1760 230]);
add_line(mdl, [ssN '/3'], 'To Workspace cam_guard/1', 'autorouting', 'smart');
add_line(mdl, 'ArmReady/1', 'To Workspace arm_gate/1', 'autorouting', 'smart');
% ---- compile once ---------------------------------------------------------------------------------------------------
feval(mdl, [], [], [], 'compile');
try
    d = get_param([SS '/CameraGuard'], 'CompiledPortDimensions');  fprintf('compiled: CameraGuard in %s out %s\n', mat2str(d.Inport), mat2str(d.Outport));
    d = get_param([mdl '/ArmReady'], 'CompiledPortDimensions');    fprintf('compiled: ArmReady in %s out %s\n', mat2str(d.Inport), mat2str(d.Outport));
    d = get_param([mdl '/EmuDropout'], 'CompiledPortDimensions');  fprintf('compiled: EmuDropout in %s out %s\n', mat2str(d.Inport), mat2str(d.Outport));
    d = get_param([mdl '/CmdGate'], 'CompiledPortDimensions');     fprintf('compiled: CmdGate in %s out %s\n', mat2str(d.Inport), mat2str(d.Outport));
catch err
    feval(mdl, [], [], [], 'term');  rethrow(err);
end
feval(mdl, [], [], [], 'term');
save_system(mdl);  close_system(mdl, 0);
% ---- re-read and verify ---------------------------------------------------------------------------------------------
evalc('load_system(slxFile)');
rt = sfroot;  bad = {};
src = @(b, p) get_param(b, 'PortConnectivity');
want = {[SS '/CameraGuard'], 1, 'MotiveFloor/1'; [SS '/CameraGuard'], 2, 'ValidSource/1'; [SS '/Reshape'], 1, 'CameraGuard/1'; ...
        [SS '/Discrete Derivative'], 1, 'CameraGuard/1'; [SS '/ValidSource'], 3, 'MATLAB System/8'; [SS '/ValidSource'], 1, 'emu_valid/1'; ...
        [mdl '/EmuDropout'], 1, 'MotiveEmulator/1'; [mdl '/ROMECommand'], 1, 'CmdGate/1'; [mdl '/To Workspace wheel_sent'], 1, 'CmdGate/1'; ...
        [mdl '/CmdGate'], 1, 'CommandGuard/1'; [mdl '/CmdGate'], 2, 'RunGate/1'; [mdl '/Stop Simulation'], 1, 'StopAny/1'; ...
        [mdl '/UKDynamics'], 8, 'RunGate/3'; [mdl '/StopAny'], 1, 'Constant3/1'; [mdl '/StopAny'], 2, 'RunGate/2'; [mdl '/ArmReady'], 1, 'ArmReadyDelay/1'; [mdl '/ArmReadyDelay'], 1, 'ROMECommand/3'; ...
        [mdl '/ArmDegDelay'], 1, 'ROMECommand/2'; [mdl '/ROMECommand'], 2, 'ArmFreezeSwitch/1'};
for k = 1:size(want, 1)
    p = src(want{k,1});  p = p(arrayfun(@(q) ~isnan(str2double(q.Type)), p));  s = p(want{k,2});
    got = '<none>';  if ~isempty(s.SrcBlock) && s.SrcBlock ~= -1, got = sprintf('%s/%d', get_param(s.SrcBlock, 'Name'), s.SrcPort + 1); end
    got = strrep(got, newline, ' ');
    fprintf('  %s in %d <- %s\n', strrep(want{k,1}, newline, ' '), want{k,2}, got);
    if ~strcmp(got, want{k,3}), bad{end+1} = sprintf('%s in %d fed by %s (want %s)', strrep(want{k,1}, newline, ' '), want{k,2}, got, want{k,3}); end %#ok<AGROW>
end
pth = {[SS '/CameraGuard'], [mdl '/EmuDropout'], [mdl '/ArmReady'], [mdl '/RunGate']};
for k = 1:4
    c = rt.find('-isa', 'Stateflow.EMChart', 'Path', pth{k});
    if ~strcmp(strrep(c.Script, char([13 10]), char(10)), txt{k}), bad{end+1} = sprintf('%s script differs from %s', pth{k}, SCR{k,2}); end %#ok<AGROW>
end
h1 = cellfun(@(p) sha(rt.find('-isa', 'Stateflow.EMChart', 'Path', p).Script), chName, 'UniformOutput', false);
if ~isequal(h0, h1), bad{end+1} = 'an existing chart script changed'; end
if ~strcmp(get_param(mdl, 'InitFcn'), ini0) || ~strcmp(get_param(mdl, 'StopFcn'), stp0), bad{end+1} = 'InitFcn or StopFcn changed'; end
try, evalc('arm_freeze_banner(mdl)'); catch eb, bad{end+1} = ['arm_freeze_banner: ' eb.identifier]; end
close_system(mdl, 0);
if ~isempty(bad), error('wire_camera_guard:verify', 'wire_camera_guard: %s', strjoin(bad, '; ')); end
fprintf('wire_camera_guard: done, re-read and verified (%s)\n', slxFile);
end
