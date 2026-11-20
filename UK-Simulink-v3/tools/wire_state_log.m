function wire_state_log(slxFile)
%WIRE_STATE_LOG  Log the controller's input: To Workspace 'q_ctrl_in' on StateSource output 1 (focus plan F3 wiring).
%   wire_state_log(slxFile)   slxFile = path of ROME_9DOF.slx (char). Re-runnable (an existing block is replaced).
%   StateSource (Switch, SID 218) output 1 is the 9-vector UKDynamics reads (input 1): with EnableMotive = 1 the camera
%   pipeline's base (MuxTracked) and the JointSource joints; with EnableMotive = 0 SensorModel. Until now only
%   SensorModel's output was logged (q_meas, SID 226), which is NOT what the controller reads with EnableMotive = 1
%   (F3 core report, concern 1). Adds 'To Workspace q_ctrl_in' (VariableName q_ctrl_in, Timeseries, SampleTime -1,
%   MaxDataPoints inf, as q_meas) fed by StateSource/1. Nothing else is changed: before saving it records every
%   Stateflow chart script (SHA-256), the model parameters below and the top-level block names; after save, close and
%   reload it asserts the scripts and parameters unchanged, the block set = before + the new block, and the new block's
%   source = StateSource/1. Errors wire_state_log:* on any mismatch. Flat function, no local functions (CLAUDE.md 5).
[~, mdl] = fileparts(slxFile);
BN = 'To Workspace q_ctrl_in';  BP = [mdl '/' BN];
PAR = {'StopTime', 'Solver', 'FixedStep', 'EnablePacing', 'PacingRate', 'InitFcn', 'StartFcn', 'StopFcn', 'PostLoadFcn'};
sha = @(s) lower(reshape(dec2hex(typecast(java.security.MessageDigest.getInstance('SHA-256').digest(uint8(unicode2native(s, 'UTF-8'))), 'uint8'), 2).', 1, []));
if bdIsLoaded(mdl), close_system(mdl, 0); end
evalc('load_system(slxFile)');
if getSimulinkBlockHandle(BP) ~= -1
    l = get_param(BP, 'LineHandles');  if l.Inport(1) ~= -1, delete_line(l.Inport(1)); end
    delete_block(BP);
end
p0 = cellfun(@(q) get_param(mdl, q), PAR, 'UniformOutput', false);
ch = sfroot().find('-isa', 'Stateflow.EMChart');  ch = ch(arrayfun(@(c) startsWith(c.Path, [mdl '/']), ch));
[~, ix] = sort(arrayfun(@(c) c.Path, ch, 'UniformOutput', false));  ch = ch(ix);
s0 = arrayfun(@(c) sha(c.Script), ch, 'UniformOutput', false);
b0 = sort(get_param(find_system(mdl, 'SearchDepth', 1, 'Type', 'block'), 'Name'));
pos = get_param([mdl '/To Workspace q_meas'], 'Position');
add_block('simulink/Sinks/To Workspace', BP, 'VariableName', 'q_ctrl_in', 'SaveFormat', 'Timeseries', 'SampleTime', '-1', ...
          'MaxDataPoints', 'inf', 'Position', pos + [0 60 0 60], ...
          'Description', 'F3: the 9-vector the controller reads (StateSource output 1 -> UKDynamics input 1); tools/wire_state_log.m');
add_line(mdl, 'StateSource/1', [BN '/1'], 'autorouting', 'smart');
save_system(mdl);  close_system(mdl, 0);
evalc('load_system(slxFile)');
p1 = cellfun(@(q) get_param(mdl, q), PAR, 'UniformOutput', false);
ch = sfroot().find('-isa', 'Stateflow.EMChart');  ch = ch(arrayfun(@(c) startsWith(c.Path, [mdl '/']), ch));
[~, ix] = sort(arrayfun(@(c) c.Path, ch, 'UniformOutput', false));  ch = ch(ix);
s1 = arrayfun(@(c) sha(c.Script), ch, 'UniformOutput', false);
b1 = sort(get_param(find_system(mdl, 'SearchDepth', 1, 'Type', 'block'), 'Name'));
pc = get_param(BP, 'PortConnectivity');
src = '<none>';  if pc(1).SrcBlock ~= -1, src = sprintf('%s/%d', get_param(pc(1).SrcBlock, 'Name'), pc(1).SrcPort + 1); end
vn = get_param(BP, 'VariableName');  sf = get_param(BP, 'SaveFormat');
close_system(mdl, 0);
fprintf('wire_state_log: %s\n  %s fed by %s, VariableName %s, SaveFormat %s; %d chart scripts; %d -> %d top-level blocks\n', ...
        slxFile, BN, src, vn, sf, numel(s1), numel(b0), numel(b1));
for k = 1:numel(PAR)
    assert(isequal(p0{k}, p1{k}), 'wire_state_log:param', '%s changed', PAR{k});
end
assert(isequal(s0, s1), 'wire_state_log:chart', 'a chart script changed on save');
assert(isequal(sort([b0; {BN}]), b1), 'wire_state_log:blocks', 'block set is not the old set plus %s', BN);
assert(strcmp(src, 'StateSource/1') && strcmp(vn, 'q_ctrl_in') && strcmp(sf, 'Timeseries'), 'wire_state_log:block', 'new block: source %s, variable %s, format %s', src, vn, sf);
fprintf('wire_state_log: done, re-read and verified\n');
end
