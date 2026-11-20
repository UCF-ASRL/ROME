function wire_camera_guard_fix4(slxFile)
%WIRE_CAMERA_GUARD_FIX4  wire3 fix 4 on a fix-1 model (tools/wire_camera_guard_fix1.m): the emulator plant takes the
%   GATED command. NOT re-runnable: refuses a model that already has PlantGate, or has no JointGate.
%   Block by block (the only change):
%     line UKDynamics/1 -> StateDelay/1   deleted (UKDynamics/1 still feeds ArmJoints and To Workspace q_cmd)
%     PlantSeed  Constant q0_9dof (the StateDelay initial condition)
%     PlantGate  MATLAB Function, script = plant_gate_block.m; inputs UKDynamics/1, RunGate/1 (wheel gate),
%                Enabled Subsystem/3 (cam_guard), ArmReady/1, PlantSeed/1; output -> StateDelay/1
%   So StateDelay (and through it SensorModel and MotiveEmulator, the fake camera of modes 1-3) holds the base while
%   CmdGate is closed and the joints while JointGate holds them. Checks after compile, save, close and reload: the new
%   connections, PlantGate script = file (LF), every other chart unchanged (SHA-256), InitFcn / StopFcn unchanged,
%   arm_freeze_banner(mdl) passes. Flat function (CLAUDE.md 5).
[root, mdl] = fileparts(slxFile);
assert(isfile(slxFile), 'wire_camera_guard_fix4:nofile', 'model file %s does not exist', slxFile);
path0 = path;  addpath(root, fullfile(root, 'Ground Vehicle'), fullfile(root, 'Optitrack'));
cleanup = onCleanup(@() path(path0)); %#ok<NASGU>
txt = strrep(fileread(fullfile(root, 'plant_gate_block.m')), char([13 10]), char(10));
sha = @(s) lower(reshape(dec2hex(typecast(java.security.MessageDigest.getInstance('SHA-256').digest(uint8(unicode2native(s, 'UTF-8'))), 'uint8'), 2).', 1, []));
ssN = ['Enabled' newline 'Subsystem'];
if bdIsLoaded(mdl), close_system(mdl, 0); end
evalc('load_system(slxFile)');
assert(getSimulinkBlockHandle([mdl '/JointGate']) ~= -1, 'wire_camera_guard_fix4:fix1', 'no JointGate: run tools/wire_camera_guard_fix1.m first');
assert(getSimulinkBlockHandle([mdl '/PlantGate']) == -1, 'wire_camera_guard_fix4:wired', 'PlantGate already in %s', slxFile);
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart');  ch = ch(strncmp({ch.Path}, [mdl '/'], numel(mdl) + 1));
other = sort({ch.Path});
h0 = cellfun(@(p) sha(rt.find('-isa', 'Stateflow.EMChart', 'Path', p).Script), other, 'UniformOutput', false);
ini0 = get_param(mdl, 'InitFcn');  stp0 = get_param(mdl, 'StopFcn');
load_system('simulink');
delete_line(mdl, 'UKDynamics/1', 'StateDelay/1');
add_block('simulink/Sources/Constant', [mdl '/PlantSeed'], 'Value', 'q0_9dof', 'Position', [1500 760 1600 780]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/PlantGate'], 'Position', [1640 740 1740 820]);
c = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/PlantGate']);  c.Script = txt;
add_line(mdl, 'UKDynamics/1', 'PlantGate/1', 'autorouting', 'smart');
add_line(mdl, 'RunGate/1', 'PlantGate/2', 'autorouting', 'smart');
add_line(mdl, [ssN '/3'], 'PlantGate/3', 'autorouting', 'smart');
add_line(mdl, 'ArmReady/1', 'PlantGate/4', 'autorouting', 'smart');
add_line(mdl, 'PlantSeed/1', 'PlantGate/5', 'autorouting', 'smart');
add_line(mdl, 'PlantGate/1', 'StateDelay/1', 'autorouting', 'smart');
feval(mdl, [], [], [], 'compile');
try
    d = get_param([mdl '/PlantGate'], 'CompiledPortDimensions');
    fprintf('compiled: PlantGate in %s out %s\n', mat2str(d.Inport), mat2str(d.Outport));
catch err
    feval(mdl, [], [], [], 'term');  rethrow(err);
end
feval(mdl, [], [], [], 'term');
save_system(mdl);  close_system(mdl, 0);
evalc('load_system(slxFile)');
rt = sfroot;  bad = {};
want = {'StateDelay', 1, 'PlantGate/1'; 'PlantGate', 1, 'UKDynamics/1'; 'PlantGate', 2, 'RunGate/1'; 'PlantGate', 4, 'ArmReady/1'; ...
        'PlantGate', 5, 'PlantSeed/1'; 'ArmJoints', 1, 'UKDynamics/1'};
for k = 1:size(want, 1)
    p = get_param([mdl '/' want{k,1}], 'PortConnectivity');  p = p(arrayfun(@(q) ~isnan(str2double(q.Type)), p));  s = p(want{k,2});
    got = '<none>';  if ~isempty(s.SrcBlock) && s.SrcBlock ~= -1, got = sprintf('%s/%d', get_param(s.SrcBlock, 'Name'), s.SrcPort + 1); end
    fprintf('  %s in %d <- %s\n', want{k,1}, want{k,2}, got);
    if ~strcmp(got, want{k,3}), bad{end+1} = sprintf('%s in %d fed by %s (want %s)', want{k,1}, want{k,2}, got, want{k,3}); end %#ok<AGROW>
end
c = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/PlantGate']);
if ~strcmp(strrep(c.Script, char([13 10]), char(10)), txt), bad{end+1} = 'PlantGate script differs from plant_gate_block.m'; end
h1 = cellfun(@(p) sha(rt.find('-isa', 'Stateflow.EMChart', 'Path', p).Script), other, 'UniformOutput', false);
if ~isequal(h0, h1), bad{end+1} = 'an existing chart script changed'; end
if ~strcmp(get_param(mdl, 'InitFcn'), ini0) || ~strcmp(get_param(mdl, 'StopFcn'), stp0), bad{end+1} = 'InitFcn or StopFcn changed'; end
try, evalc('arm_freeze_banner(mdl)'); catch eb, bad{end+1} = ['arm_freeze_banner: ' eb.identifier]; end
close_system(mdl, 0);
if ~isempty(bad), error('wire_camera_guard_fix4:verify', 'wire_camera_guard_fix4: %s', strjoin(bad, '; ')); end
fprintf('wire_camera_guard_fix4: done, re-read and verified (%s)\n', slxFile);
end
