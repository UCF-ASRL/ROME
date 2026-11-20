function wire_arm_freeze(slxFile, mode)
%WIRE_ARM_FREEZE  Upstream arm freeze in ROME_9DOF.slx (Plan 2 Task 5 / focus plan F1, 19 Nov 2026). Re-runnable.
%   Author, 19 Nov 2026: ROMECommand.m stays byte-identical to Keanu's version; the student's arm_freeze
%   (default 1) is applied UPSTREAM in the model, with a banner ARM FROZEN / ARM WILL MOVE.
%
%   wire_arm_freeze()                     wires ROME_merged/ROME_9DOF.slx (this file's ../ROME_9DOF.slx)
%   wire_arm_freeze(slxFile)              wires the given model file (e.g. a temp copy)
%   wire_arm_freeze(slxFile, 'remove')    restores the wiring before F1 (no switch, no joint_guard_rad, InitFcn
%                                         apply_pacing only)
%   wire_arm_freeze(slxFile, 'bypass')    MAKE-IT-FAIL only: deletes the switch and its two Constants and joins
%                                         CommandGuard straight to ROMECommand again, keeping joint_guard_rad
%                                         and the banner (so the banner then claims a freeze that is not there)
%
%   'wire' builds, after removing any earlier F1 blocks (so running it twice gives the same model):
%     ArmFreezePosture  Constant, Value q0_9dof(4:9) (6x1, model rad, VectorParams1D off, SampleTime inf)
%     ArmFreezeUser     Constant, Value arm_freeze_user (flag, SampleTime inf). NOT arm_freeze: ROMECommand.m L56
%                       writes arm_freeze = 0 into the base workspace at every run start; a Constant is read
%                       at compile, before that write, so a FIRST run would hold but every later Run of the
%                       same session would move the arm (docs/verification/2026-11-19_f1_eval_order_probe.log).
%     ArmFreezeSwitch   Switch, u1 = ArmFreezePosture, u2 = ArmFreezeUser, u3 = CommandGuard out 2 (joint_out,
%                       model rad), criterion u2 > 0.5 (as StateSource). Output -> ROMECommand in 2 (jointRad)
%                       and 'To Workspace joint_sent_rad' (what is sent, as its name says).
%     'To Workspace joint_guard_rad'  copy of joint_sent_rad's To Workspace on CommandGuard out 2 (the
%                       controller's joint command before the freeze), so tests can see both.
%     InitFcn           'apply_pacing(bdroot); arm_freeze_banner(bdroot)' (pacing refusals keep precedence).
%   ROMECommand maps and clamps jointRad itself (arm_sign, arm_offset_deg, arm_clamp), so a frozen run sends
%   arm_clamp(arm_sign .* rad2deg(q0_9dof(4:9))' + arm_offset_deg) = define_constants' arm_freeze_deg.
%
%   The model is compiled (port sizes printed; not in bypass mode), saved, closed and RE-READ from disk; the connectivity of every
%   touched block is then asserted and printed with its SID. The four MATLAB Function chart scripts are hashed
%   (SHA-256) before and after and must not change (CLAUDE.md section 27). Errors on any mismatch.
%   Flat function without local functions (CLAUDE.md section 5, crash 2).
if nargin < 1 || isempty(slxFile)
    slxFile = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'ROME_9DOF.slx');   % ROME_merged/ROME_9DOF.slx (char)
end
if nargin < 2 || isempty(mode), mode = 'wire'; end
assert(any(strcmp(mode, {'wire', 'remove', 'bypass'})), 'wire_arm_freeze:mode', 'mode must be wire, remove or bypass, not %s', mode);
assert(isfile(slxFile), 'wire_arm_freeze:nofile', 'model file %s does not exist', slxFile);
root = fileparts(slxFile);                     % folder of the model, the ROME_merged layout (char)
[~, mdl] = fileparts(slxFile);                 % model name (char)
path0 = path;                                  % MATLAB path at entry, restored at exit (char)
addpath(root, fullfile(root, 'Ground Vehicle'), fullfile(root, 'Optitrack'));
cleanup = onCleanup(@() path(path0)); %#ok<NASGU>
pacOnly = 'apply_pacing(bdroot)';                              % InitFcn before F1 (char)
pacBan  = 'apply_pacing(bdroot); arm_freeze_banner(bdroot)';   % InitFcn with the banner (char)
SW = [mdl '/ArmFreezeSwitch'];                 % block paths (char)
CU = [mdl '/ArmFreezeUser'];
CP = [mdl '/ArmFreezePosture'];
TG = [mdl '/To Workspace joint_guard_rad'];
TS = [mdl '/To Workspace joint_sent_rad'];
RC = [mdl '/ROMECommand'];
CG = [mdl '/CommandGuard'];
sha = @(s) lower(reshape(dec2hex(typecast(java.security.MessageDigest.getInstance('SHA-256').digest(uint8(unicode2native(s, 'UTF-8'))), 'uint8'), 2).', 1, []));   % SHA-256 of a char (hex char)
exists = @(b) getSimulinkBlockHandle(b) ~= -1;  % block present in the loaded model (logical)
fprintf('wire_arm_freeze: mode %s on %s\n', mode, slxFile);

% ---- load (PostLoadFcn = define_constants; its card is captured, not printed) -------------------------------------
if bdIsLoaded(mdl), close_system(mdl, 0); end
evalc('load_system(slxFile)');
assert(strcmp(get_param(mdl, 'FileName'), slxFile), 'wire_arm_freeze:file', 'loaded %s, expected %s', get_param(mdl, 'FileName'), slxFile);
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart');     % MATLAB Function charts of every loaded model
ch = ch(strncmp({ch.Path}, [mdl '/'], numel(mdl) + 1));
chName = {ch.Path};                            % chart paths (cell of char)
h0 = cellfun(@(p) sha(rt.find('-isa', 'Stateflow.EMChart', 'Path', p).Script), chName, 'UniformOutput', false);   % hashes before
fprintf('charts before: %s\n', strjoin(cellfun(@(p, h) sprintf('%s %s', p, h(1:16)), chName, h0, 'UniformOutput', false), '; '));

% ---- tear down earlier F1 blocks (all modes; bypass keeps joint_guard_rad) -----------------------------------------
if exists(SW)
    lh = get_param(SW, 'LineHandles');
    if lh.Outport(1) ~= -1, delete_line(lh.Outport(1)); end   % the tree Switch -> ROMECommand / joint_sent_rad
    if lh.Inport(3) ~= -1, delete_line(mdl, 'CommandGuard/2', 'ArmFreezeSwitch/3'); end   % one branch of the guard line
    delete_block(SW);
end
for b = {CU, CP}
    if exists(b{1})
        lh = get_param(b{1}, 'LineHandles');
        if lh.Outport(1) ~= -1, delete_line(lh.Outport(1)); end
        delete_block(b{1});
    end
end
if ~strcmp(mode, 'bypass') && exists(TG)
    lh = get_param(TG, 'LineHandles');
    if lh.Inport(1) ~= -1, delete_line(mdl, 'CommandGuard/2', 'To Workspace joint_guard_rad/1'); end
    delete_block(TG);
end

% ---- build --------------------------------------------------------------------------------------------------------
pc = get_param(RC, 'PortConnectivity');        % ROMECommand ports: 1-3 inputs, 4-6 outputs
pt = get_param(TS, 'PortConnectivity');        % joint_sent_rad's To Workspace: 1 input
for e = {{RC, pc(2), 'ROMECommand/2'}, {TS, pt(1), 'To Workspace joint_sent_rad/1'}}
    s = e{1}{2};                               % its input port's connectivity
    if ~isempty(s.SrcBlock) && s.SrcBlock ~= -1
        assert(strcmp(getfullname(s.SrcBlock), CG) && s.SrcPort == 1, 'wire_arm_freeze:state', ...
               '%s is fed by %s port %d, expected CommandGuard port 2: unknown model state, nothing changed', e{1}{3}, getfullname(s.SrcBlock), s.SrcPort + 1);
        if strcmp(mode, 'wire'), delete_line(mdl, 'CommandGuard/2', e{1}{3}); end
    elseif ~strcmp(mode, 'wire')
        add_line(mdl, 'CommandGuard/2', e{1}{3}, 'autorouting', 'smart');
    end
end
if strcmp(mode, 'wire')
    add_block('simulink/Sources/Constant', CP, 'Value', 'q0_9dof(4:9)', 'VectorParams1D', 'off', 'SampleTime', 'inf', ...
              'Position', [900 160 980 190], 'Description', 'F1 arm freeze: start posture q0_9dof(4:9), 6x1 model rad (tools/wire_arm_freeze.m)');
    add_block('simulink/Sources/Constant', CU, 'Value', 'arm_freeze_user', 'SampleTime', 'inf', ...
              'Position', [900 210 980 240], 'Description', 'F1 arm freeze flag: arm_freeze_user, set at the END of define_constants (not arm_freeze, which ROMECommand.m L56 resets to 0 at every run start)');
    add_block('simulink/Signal Routing/Switch', SW, 'Criteria', 'u2 > Threshold', 'Threshold', '0.5', ...
              'Position', [1005 160 1030 260], 'Description', 'F1 arm freeze: u2 = arm_freeze_user > 0.5 sends q0_9dof(4:9), else the CommandGuard joint command (tools/wire_arm_freeze.m)');
    add_block(TS, TG, 'VariableName', 'joint_guard_rad', 'Position', [1260 540 1320 570]);
    add_line(mdl, 'ArmFreezePosture/1', 'ArmFreezeSwitch/1', 'autorouting', 'smart');
    add_line(mdl, 'ArmFreezeUser/1',    'ArmFreezeSwitch/2', 'autorouting', 'smart');
    add_line(mdl, 'CommandGuard/2',     'ArmFreezeSwitch/3', 'autorouting', 'smart');
    add_line(mdl, 'ArmFreezeSwitch/1',  'ROMECommand/2', 'autorouting', 'smart');
    add_line(mdl, 'ArmFreezeSwitch/1',  'To Workspace joint_sent_rad/1', 'autorouting', 'smart');
    add_line(mdl, 'CommandGuard/2',     'To Workspace joint_guard_rad/1', 'autorouting', 'smart');
end
fi = get_param(mdl, 'InitFcn');               % InitFcn now (char)
assert(any(strcmp(fi, {pacOnly, pacBan})), 'wire_arm_freeze:initfcn', 'InitFcn is ''%s'', expected ''%s'' or ''%s''', fi, pacOnly, pacBan);
if strcmp(mode, 'wire'),   set_param(mdl, 'InitFcn', pacBan);  end
if strcmp(mode, 'remove'), set_param(mdl, 'InitFcn', pacOnly); end

% ---- compile once: port sizes into ROMECommand (InitFcn runs, so the banner prints here) --------------------------
% Not in bypass mode: there the banner (arm_freeze_banner) refuses to compile a model whose switch is gone while
% arm_freeze_user = 1, which is the point of that break.
if ~strcmp(mode, 'bypass')
    feval(mdl, [], [], [], 'compile');
    try
        d = get_param(RC, 'CompiledPortDimensions');   % Inport: [nd d1 d2] per port
        fprintf('compiled: ROMECommand input dims %s\n', mat2str(d.Inport));
        if exists(SW)
            d2 = get_param(SW, 'CompiledPortDimensions');
            fprintf('compiled: ArmFreezeSwitch input dims %s, output %s\n', mat2str(d2.Inport), mat2str(d2.Outport));
        end
    catch err
        feval(mdl, [], [], [], 'term');
        rethrow(err);
    end
    feval(mdl, [], [], [], 'term');
end
h1 = cellfun(@(p) sha(rt.find('-isa', 'Stateflow.EMChart', 'Path', p).Script), chName, 'UniformOutput', false);
assert(isequal(h0, h1), 'wire_arm_freeze:chart', 'a chart script changed while wiring; model NOT saved');
save_system(mdl);
close_system(mdl, 0);

% ---- re-read from disk and assert --------------------------------------------------------------------------------
evalc('load_system(slxFile)');
rt = sfroot;
h2 = cellfun(@(p) sha(rt.find('-isa', 'Stateflow.EMChart', 'Path', p).Script), chName, 'UniformOutput', false);
fprintf('re-read %s (%s)\n', slxFile, get_param(mdl, 'LastModifiedDate'));
fprintf('charts after re-read: %s -> unchanged %d\n', strjoin(cellfun(@(p, h) sprintf('%s %s', p, h(1:16)), chName, h2, 'UniformOutput', false), '; '), isequal(h0, h2));
switch mode
    case 'wire',   want = {RC, 2, SW, 1; TS, 1, SW, 1; SW, 1, CP, 1; SW, 2, CU, 1; SW, 3, CG, 2; TG, 1, CG, 2};   % dst, input, src, output
    case 'remove', want = {RC, 2, CG, 2; TS, 1, CG, 2};
    case 'bypass', want = {RC, 2, CG, 2; TS, 1, CG, 2};
end
bad = {};                                      % failed expectations (cell of char)
tags = {'WRONG', 'OK'};                        % verdict text, indexed by 1 + match
for k = 1:size(want, 1)
    p = get_param(want{k,1}, 'PortConnectivity');
    s = p(want{k,2});
    got = '<none>';
    if ~isempty(s.SrcBlock) && s.SrcBlock ~= -1, got = sprintf('%s/%d', getfullname(s.SrcBlock), s.SrcPort + 1); end
    exp_ = sprintf('%s/%d', want{k,3}, want{k,4});
    fprintf('  %-34s in %d <- %-34s (SID %s <- SID %s) %s\n', strrep(want{k,1}, [mdl '/'], ''), want{k,2}, strrep(got, [mdl '/'], ''), ...
            get_param(want{k,1}, 'SID'), get_param(want{k,3}, 'SID'), tags{1 + strcmp(got, exp_)});
    if ~strcmp(got, exp_), bad{end+1} = sprintf('%s in %d fed by %s, expected %s', want{k,1}, want{k,2}, got, exp_); end %#ok<AGROW>
end
present = cellfun(exists, {SW, CU, CP, TG});   % F1 blocks present (logical 1x4)
switch mode
    case 'wire',   wantPresent = [true true true true];   wantInit = pacBan;
    case 'remove', wantPresent = [false false false false]; wantInit = pacOnly;
    case 'bypass', wantPresent = [false false false true];  wantInit = get_param(mdl, 'InitFcn');   % left as it was
end
fprintf('  F1 blocks present [Switch User Posture joint_guard_rad] = %s (expected %s)\n', mat2str(present), mat2str(wantPresent));
if ~isequal(present, wantPresent), bad{end+1} = 'F1 block set differs from the mode'; end
if strcmp(mode, 'wire')
    vals = {get_param(CU, 'Value'), get_param(CP, 'Value'), get_param(CP, 'VectorParams1D'), get_param(SW, 'Criteria'), get_param(SW, 'Threshold'), get_param(TG, 'VariableName')};
    wantVals = {'arm_freeze_user', 'q0_9dof(4:9)', 'off', 'u2 > Threshold', '0.5', 'joint_guard_rad'};
    fprintf('  ArmFreezeUser Value %s | ArmFreezePosture Value %s VectorParams1D %s | Switch %s, Threshold %s | joint_guard_rad VariableName %s\n', vals{:});
    if ~isequal(vals, wantVals), bad{end+1} = 'block parameters differ'; end
end
fprintf('  InitFcn: %s\n', get_param(mdl, 'InitFcn'));
if ~strcmp(get_param(mdl, 'InitFcn'), wantInit), bad{end+1} = 'InitFcn differs'; end
if ~isequal(h0, h2), bad{end+1} = 'chart scripts differ after re-read'; end
close_system(mdl, 0);
if ~isempty(bad)
    error('wire_arm_freeze:verify', 'after re-read: %s', strjoin(bad, '; '));
end
fprintf('wire_arm_freeze: %s done, re-read and verified\n', mode);
end
