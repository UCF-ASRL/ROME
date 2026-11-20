function wire_camera_guard_fix1(slxFile)
%WIRE_CAMERA_GUARD_FIX1  wire3 fix round 1 (review C2, I1, I2, I4) on an F6-wired ROME_9DOF.slx (tools/wire_camera_guard.m).
%   wire_camera_guard_fix1(slxFile)   NOT re-runnable: refuses a model that already has JointGate, or has no CameraGuard.
%   Built (model rad unless stated):
%     ParkPosture  Constant q0_9dof(4:9): the posture calibrateROMEArm parks the arm at (the park record holds it, F11)
%     JointGate    MATLAB Function, script = joint_gate_block.m: ArmFreezeSwitch/1 -> JointGate -> ROMECommand in 2 AND
%                  To Workspace joint_sent_rad. Parked posture until the controller first runs on real data; the last
%                  command held during a camera hold, a telemetry loss or a stop (review C2)
%     TelOrPark    MATLAB Function, script = tel_or_park_block.m: TelemetryDelay/1 -> TelOrPark -> JointSource in 1. The
%                  controller reads the parked posture until the arm-ready go, never ROMECommand's arm_home seed (C2)
%     RefClock     MATLAB Function, script = ref_clock_block.m: Clock9 -> RefClock -> EndEffectorTrajectory in 1. The
%                  reference is frozen at 0 while the run waits and starts at the release (I2)
%     RunGate      script replaced by run_gate_block.m: also re-seeds at the step a wait ends (I1); arm input 4x1
%     ArmReady     script replaced by arm_ready_block.m: output [go waiting refuse lost], loss after go held then stopped
%                  (I4); ArmLimits = rome_limit('arm_wait') now 3x1 (adds arm_lost_max_s)
%   Not changed: CommandGuard's soft-start ramp still counts from the simulation start (its chart is CommandGuard.m,
%   gated by check_guard; changing it is outside this round). arm_freeze_banner's wiring check now expects
%   ROMECommand in 2 <- JointGate/1 <- ArmFreezeSwitch/1.
%   Checks after compile, save, close and reload: the new connections, the five chart scripts = files (LF), every other
%   chart unchanged (SHA-256), InitFcn / StopFcn unchanged, arm_freeze_banner(mdl) passes. Flat function (CLAUDE.md 5).
[root, mdl] = fileparts(slxFile);
assert(isfile(slxFile), 'wire_camera_guard_fix1:nofile', 'model file %s does not exist', slxFile);
path0 = path;  addpath(root, fullfile(root, 'Ground Vehicle'), fullfile(root, 'Optitrack'));
cleanup = onCleanup(@() path(path0)); %#ok<NASGU>
rd = @(f) strrep(fileread(f), char([13 10]), char(10));
ssN = ['Enabled' newline 'Subsystem'];  SS = [mdl '/' ssN];
NEW = {'JointGate', 'joint_gate_block.m'; 'TelOrPark', 'tel_or_park_block.m'; 'RefClock', 'ref_clock_block.m'; ...
       'RunGate', 'run_gate_block.m'; 'ArmReady', 'arm_ready_block.m'};
txt = cellfun(@(f) rd(fullfile(root, f)), NEW(:,2), 'UniformOutput', false);
sha = @(s) lower(reshape(dec2hex(typecast(java.security.MessageDigest.getInstance('SHA-256').digest(uint8(unicode2native(s, 'UTF-8'))), 'uint8'), 2).', 1, []));
if bdIsLoaded(mdl), close_system(mdl, 0); end
evalc('load_system(slxFile)');
assert(getSimulinkBlockHandle([SS '/CameraGuard']) ~= -1, 'wire_camera_guard_fix1:f6', 'no CameraGuard: run tools/wire_camera_guard.m first');
assert(getSimulinkBlockHandle([mdl '/JointGate']) == -1, 'wire_camera_guard_fix1:wired', 'JointGate already in %s', slxFile);
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart');  ch = ch(strncmp({ch.Path}, [mdl '/'], numel(mdl) + 1));
other = setdiff(sort({ch.Path}), {[mdl '/RunGate'], [mdl '/ArmReady']});
h0 = cellfun(@(p) sha(rt.find('-isa', 'Stateflow.EMChart', 'Path', p).Script), other, 'UniformOutput', false);
ini0 = get_param(mdl, 'InitFcn');  stp0 = get_param(mdl, 'StopFcn');
load_system('simulink');
% ---- replaced scripts -----------------------------------------------------------------------------------------------
c = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/RunGate']);   c.Script = txt{4};
c = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/ArmReady']);  c.Script = txt{5};
% ---- new blocks -----------------------------------------------------------------------------------------------------
add_block('simulink/Sources/Constant', [mdl '/ParkPosture'], 'Value', 'q0_9dof(4:9)', 'Position', [1100 560 1220 580], ...
          'Description', 'wire3 fix 1: the parked posture (calibrateROMEArm parks the arm here; F11 park record)');
for k = 1:3
    add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/' NEW{k,1}], 'Position', [1300 500 + 80 * k 1400 560 + 80 * k]);
    c = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/' NEW{k,1}]);  c.Script = txt{k};
end
% JointGate: ArmFreezeSwitch/1 -> JointGate -> ROMECommand/2, joint_sent_rad
delete_line(mdl, 'ArmFreezeSwitch/1', 'ROMECommand/2');
delete_line(mdl, 'ArmFreezeSwitch/1', 'To Workspace joint_sent_rad/1');
add_line(mdl, 'ArmFreezeSwitch/1', 'JointGate/1', 'autorouting', 'smart');
add_line(mdl, [ssN '/3'], 'JointGate/2', 'autorouting', 'smart');
add_line(mdl, 'ArmReady/1', 'JointGate/3', 'autorouting', 'smart');
add_line(mdl, 'ParkPosture/1', 'JointGate/4', 'autorouting', 'smart');
add_line(mdl, 'JointGate/1', 'ROMECommand/2', 'autorouting', 'smart');
add_line(mdl, 'JointGate/1', 'To Workspace joint_sent_rad/1', 'autorouting', 'smart');
% TelOrPark: TelemetryDelay/1 -> TelOrPark -> JointSource/1
delete_line(mdl, 'TelemetryDelay/1', 'JointSource/1');
add_line(mdl, 'TelemetryDelay/1', 'TelOrPark/1', 'autorouting', 'smart');
add_line(mdl, 'ArmReady/1', 'TelOrPark/2', 'autorouting', 'smart');
add_line(mdl, 'ParkPosture/1', 'TelOrPark/3', 'autorouting', 'smart');
add_line(mdl, 'TelOrPark/1', 'JointSource/1', 'autorouting', 'smart');
% RefClock: Clock9/1 -> RefClock -> EndEffectorTrajectory/1
delete_line(mdl, 'Clock9/1', 'EndEffectorTrajectory/1');
add_line(mdl, 'Clock9/1', 'RefClock/1', 'autorouting', 'smart');
add_line(mdl, [ssN '/3'], 'RefClock/2', 'autorouting', 'smart');
add_line(mdl, 'ArmReady/1', 'RefClock/3', 'autorouting', 'smart');
add_line(mdl, 'RefClock/1', 'EndEffectorTrajectory/1', 'autorouting', 'smart');
% ---- compile once ---------------------------------------------------------------------------------------------------
feval(mdl, [], [], [], 'compile');
try
    for b = {'JointGate', 'TelOrPark', 'RefClock', 'RunGate', 'ArmReady'}
        d = get_param([mdl '/' b{1}], 'CompiledPortDimensions');
        fprintf('compiled: %s in %s out %s\n', b{1}, mat2str(d.Inport), mat2str(d.Outport));
    end
catch err
    feval(mdl, [], [], [], 'term');  rethrow(err);
end
feval(mdl, [], [], [], 'term');
save_system(mdl);  close_system(mdl, 0);
% ---- re-read and verify ---------------------------------------------------------------------------------------------
evalc('load_system(slxFile)');
rt = sfroot;  bad = {};
want = {'ROMECommand', 2, 'JointGate/1'; 'To Workspace joint_sent_rad', 1, 'JointGate/1'; 'JointGate', 1, 'ArmFreezeSwitch/1'; ...
        'JointGate', 3, 'ArmReady/1'; 'JointGate', 4, 'ParkPosture/1'; 'JointSource', 1, 'TelOrPark/1'; 'TelOrPark', 1, 'TelemetryDelay/1'; ...
        'TelOrPark', 2, 'ArmReady/1'; 'EndEffectorTrajectory', 1, 'RefClock/1'; 'RefClock', 1, 'Clock9/1'; 'RefClock', 3, 'ArmReady/1'; ...
        'RunGate', 2, 'ArmReady/1'; 'ArmFreezeSwitch', 3, 'CommandGuard/2'};
for k = 1:size(want, 1)
    p = get_param([mdl '/' want{k,1}], 'PortConnectivity');  p = p(arrayfun(@(q) ~isnan(str2double(q.Type)), p));  s = p(want{k,2});
    got = '<none>';  if ~isempty(s.SrcBlock) && s.SrcBlock ~= -1, got = sprintf('%s/%d', get_param(s.SrcBlock, 'Name'), s.SrcPort + 1); end
    fprintf('  %s in %d <- %s\n', want{k,1}, want{k,2}, got);
    if ~strcmp(got, want{k,3}), bad{end+1} = sprintf('%s in %d fed by %s (want %s)', want{k,1}, want{k,2}, got, want{k,3}); end %#ok<AGROW>
end
for k = 1:size(NEW, 1)
    c = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/' NEW{k,1}]);
    if ~strcmp(strrep(c.Script, char([13 10]), char(10)), txt{k}), bad{end+1} = sprintf('%s script differs from %s', NEW{k,1}, NEW{k,2}); end %#ok<AGROW>
end
h1 = cellfun(@(p) sha(rt.find('-isa', 'Stateflow.EMChart', 'Path', p).Script), other, 'UniformOutput', false);
if ~isequal(h0, h1), bad{end+1} = 'an existing chart script changed'; end
if ~strcmp(get_param(mdl, 'InitFcn'), ini0) || ~strcmp(get_param(mdl, 'StopFcn'), stp0), bad{end+1} = 'InitFcn or StopFcn changed'; end
try, evalc('arm_freeze_banner(mdl)'); catch eb, bad{end+1} = ['arm_freeze_banner: ' eb.identifier ' ' eb.message]; end
close_system(mdl, 0);
if ~isempty(bad), error('wire_camera_guard_fix1:verify', 'wire_camera_guard_fix1: %s', strjoin(bad, '; ')); end
fprintf('wire_camera_guard_fix1: done, re-read and verified (%s)\n', slxFile);
end
