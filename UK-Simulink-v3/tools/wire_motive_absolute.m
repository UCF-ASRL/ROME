function wire_motive_absolute(slxFile, mode)
%WIRE_MOTIVE_ABSOLUTE  Absolute floor-frame camera path in ROME_9DOF.slx (focus plan F5b, Plan 2 Task 2). Re-runnable.
%   wire_motive_absolute()                  wires ROME_merged/ROME_9DOF.slx (this file's ../ROME_9DOF.slx)
%   wire_motive_absolute(slxFile)           wires the given model file (e.g. a temp copy)
%   wire_motive_absolute(slxFile, 'legacy') restores the chain before F5b (make-it-fail of tests/test_motive_model.m
%                                           check iv): root Constant MotiveInit (q0_9dof(1:3)) -> subsystem Inport q0
%                                           -> Add, with Mux1 [MotiveData 1, MotiveData 2, Gain -1 (Multiply2) of
%                                           MotiveData 6]; MotiveData Enable from the Enable port (EnableMotive alone)
%
%   'wire' builds, after tearing down both the legacy chain and any earlier F5b blocks (so running it twice gives the
%   same model), inside 'Enabled Subsystem' (SID 100, still enabled by Constant1 = EnableMotive):
%     MotiveFloor        MATLAB Function, script = Optitrack/motive_floor_block.m verbatim (motive_to_floor with the
%                        chart Parameter motive_cal, then yaw_unwrap seeded from the Parameter q0_9dof(3)). Input 1 =
%                        MotiveData output 7 (pose7); output -> Reshape (-> tracked_position) and Discrete Derivative
%                        (kept, -> Reshape1 -> tracked_velocity).
%     reader_enable      Inport 1, -> MotiveData input 1 (Enable). The Enable port's output is hidden.
%   and at the root
%     MotiveReaderEnable Constant 'double(EnableMotive ~= 0 && EnableHardware ~= 0)' -> subsystem input 1: the real
%                        reader (socket, 500 ms receive timeout) runs only on the robot with cameras. F7 hook: the
%                        camera emulator (motive_source) changes this expression and puts its pose7 source in front
%                        of MotiveFloor input 1.
%   Removed: MotiveInit (SID 221), Add (SID 184), Multiply2 = Gain -1 (SID 173), Mux1 (SID 108), Inport q0 (SID 183).
%   Kept: MotiveData (SID 105, name 'MATLAB System'; outputs 1-6 still drive the 'Motive Data1' scope through Mux),
%   Discrete Derivative, both Reshapes, both Outports; MuxTracked, StateSource, UKDynamics unchanged.
%
%   The model is compiled once (port sizes printed), saved, closed and RE-READ from disk; then asserted and printed
%   with SIDs: the connectivity of the chain MotiveData/7 -> MotiveFloor -> Reshape -> tracked_position -> MuxTracked
%   -> StateSource -> UKDynamics (or the legacy chain), which blocks exist and which do not (also by SID), the block
%   parameters, the MotiveFloor script = the file (line endings normalised), its data scopes, the four existing
%   MATLAB Function charts unchanged (SHA-256 before / after, CLAUDE.md section 27), the InitFcn unchanged, and the
%   F1 arm-freeze wiring intact (arm_freeze_banner(mdl), which refuses any other wiring). Errors on any mismatch.
%   Flat function without local functions (CLAUDE.md section 5, crash 2).
if nargin < 1 || isempty(slxFile)
    slxFile = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'ROME_9DOF.slx');   % ROME_merged/ROME_9DOF.slx (char)
end
if nargin < 2 || isempty(mode), mode = 'wire'; end
assert(any(strcmp(mode, {'wire', 'legacy'})), 'wire_motive_absolute:mode', 'mode must be wire or legacy, not %s', mode);
assert(isfile(slxFile), 'wire_motive_absolute:nofile', 'model file %s does not exist', slxFile);
root = fileparts(slxFile);                     % folder of the model, the ROME_merged layout (char)
[~, mdl] = fileparts(slxFile);                 % model name (char)
path0 = path;                                  % MATLAB path at entry, restored at exit (char)
addpath(root, fullfile(root, 'Ground Vehicle'), fullfile(root, 'Optitrack'));
cleanup = onCleanup(@() path(path0)); %#ok<NASGU>
chartFile = fullfile(root, 'Optitrack', 'motive_floor_block.m');   % the MotiveFloor script (char)
assert(isfile(chartFile), 'wire_motive_absolute:nochart', 'chart script %s does not exist', chartFile);
fid = fopen(chartFile, 'r');  chartTxt = char(fread(fid, Inf, '*uint8').');  fclose(fid);   % file text (char)
chartTxt = strrep(chartTxt, char([13 10]), char(10));   % LF line endings (char)
ssN = ['Enabled' newline 'Subsystem'];         % subsystem name (char)
SS = [mdl '/' ssN];                            % subsystem path (char)
MD = [SS '/MATLAB System'];  EN = [SS '/Enable'];  RS = [SS '/Reshape'];  DD = [SS '/Discrete Derivative'];
TP = [SS '/tracked_position'];  FL = [SS '/MotiveFloor'];  RI = [SS '/reader_enable'];
Q0 = [SS '/q0'];  AD = [SS '/Add'];  M1 = [SS '/Mux1'];  G1 = [SS '/Multiply2'];
RE = [mdl '/MotiveReaderEnable'];  MI = [mdl '/MotiveInit'];
MT = [mdl '/MuxTracked'];  ST = [mdl '/StateSource'];  UK = [mdl '/UKDynamics'];  UM = [mdl '/UseMotive'];  C1 = [mdl '/Constant1'];
reVal = 'double(EnableMotive ~= 0 && EnableHardware ~= 0)';   % reader enable expression (char)
pacBan = 'apply_pacing(bdroot); arm_freeze_banner(bdroot)';   % InitFcn set by F1 (char), must stay
sha = @(s) lower(reshape(dec2hex(typecast(java.security.MessageDigest.getInstance('SHA-256').digest(uint8(unicode2native(s, 'UTF-8'))), 'uint8'), 2).', 1, []));   % SHA-256 of a char (hex char)
exists = @(b) getSimulinkBlockHandle(b) ~= -1;  % block present in the loaded model (logical)
srcOf = @(b) get_param(b, 'PortConnectivity');   % port connectivity of a block (struct array)
fprintf('wire_motive_absolute: mode %s on %s\n', mode, slxFile);

% ---- load (PostLoadFcn = define_constants; its card is captured, not printed) -------------------------------------
if bdIsLoaded(mdl), close_system(mdl, 0); end
evalc('load_system(slxFile)');
assert(strcmp(get_param(mdl, 'FileName'), slxFile), 'wire_motive_absolute:file', 'loaded %s, expected %s', get_param(mdl, 'FileName'), slxFile);
if exists([mdl '/MotiveEmulator']) || exists([SS '/PoseSource'])   % F7 camera emulator present: this tool does not know its blocks
    close_system(mdl, 0);
    error('wire_motive_absolute:f7', 'the F7 camera emulator is wired in %s: run tools/wire_motive_emulator(slxFile, ''unwire'') first. Nothing changed.', slxFile);
end
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart');
ch = ch(strncmp({ch.Path}, [mdl '/'], numel(mdl) + 1) & ~strcmp({ch.Path}, FL));   % existing charts, MotiveFloor excluded
chName = sort({ch.Path});                      % chart paths (cell of char)
h0 = cellfun(@(p) sha(rt.find('-isa', 'Stateflow.EMChart', 'Path', p).Script), chName, 'UniformOutput', false);
fprintf('charts before: %s\n', strjoin(cellfun(@(p, h) sprintf('%s %s', strrep(p, newline, ' '), h(1:16)), chName, h0, 'UniformOutput', false), '; '));
initFcn0 = get_param(mdl, 'InitFcn');          % InitFcn before (char), must not change
assert(strcmp(initFcn0, pacBan), 'wire_motive_absolute:initfcn', 'InitFcn is ''%s'', expected the F1 ''%s''', initFcn0, pacBan);

% ---- tear down: every legacy and F5b connection, then the blocks ---------------------------------------------------
% {system, dst block, dst input, src block, src output}: deleted when the dst input is fed by exactly that source
conn = {mdl, SS, 1, MI, 1;  mdl, SS, 1, RE, 1; ...
        SS, AD, 1, Q0, 1;  SS, AD, 2, M1, 1;  SS, RS, 1, AD, 1;  SS, DD, 1, AD, 1; ...
        SS, M1, 1, MD, 1;  SS, M1, 2, MD, 2;  SS, M1, 3, G1, 1;  SS, G1, 1, MD, 6; ...
        SS, MD, 1, EN, 1;  SS, MD, 1, RI, 1; ...
        SS, FL, 1, MD, 7;  SS, RS, 1, FL, 1;  SS, DD, 1, FL, 1};
nDel = 0;                                      % lines deleted (count)
for k = 1:size(conn, 1)
    [sy, db, dp, sb, sp] = conn{k,:};
    if ~(exists(db) && exists(sb)), continue; end
    p = srcOf(db);  s = p(dp);
    if ~isempty(s.SrcBlock) && s.SrcBlock ~= -1 && s.SrcBlock == get_param(sb, 'Handle') && s.SrcPort + 1 == sp
        delete_line(sy, sprintf('%s/%d', get_param(sb, 'Name'), sp), sprintf('%s/%d', get_param(db, 'Name'), dp));
        nDel = nDel + 1;
    end
end
for b = {AD, M1, G1, Q0, FL, RI, MI, RE}
    if exists(b{1}), delete_block(b{1}); end
end
fprintf('teardown: %d lines deleted\n', nDel);
% nothing else may feed the inputs the build connects (unknown model state otherwise: stop before saving)
for b = {RS, DD, MD}
    p = srcOf(b{1});  s = p(1);
    if ~(isempty(s.SrcBlock) || s.SrcBlock == -1)
        error('wire_motive_absolute:state', '%s input 1 is still fed by %s after teardown: unknown model state, NOT saved', ...
              strrep(b{1}, newline, ' '), getfullname(s.SrcBlock));
    end
end
pss = srcOf(SS);
assert(~any(arrayfun(@(q) strcmp(q.Type, '1') && ~isempty(q.SrcBlock) && q.SrcBlock ~= -1, pss)), 'wire_motive_absolute:state', 'subsystem input 1 still fed after teardown');

% ---- build ---------------------------------------------------------------------------------------------------------
load_system('simulink');
if strcmp(mode, 'wire')
    set_param(EN, 'ShowOutputPort', 'off');
    add_block('simulink/Sources/In1', RI, 'Port', '1', 'Position', [-60 175 -30 189]);
    add_block('simulink/User-Defined Functions/MATLAB Function', FL, 'Position', [400 10 500 70]);
    rt = sfroot;
    fc = rt.find('-isa', 'Stateflow.EMChart', 'Path', FL);   % the new chart object
    assert(numel(fc) == 1, 'wire_motive_absolute:chart', 'MotiveFloor chart found %d times', numel(fc));
    fc.Script = chartTxt;
    want = {'pose7', 'Input'; 'base', 'Output'; 'motive_cal', 'Parameter'; 'q0_9dof', 'Parameter'};   % data name | scope
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
                d.Tunable = false;             % read once at compile: motive_cal and q0_9dof are fixed during a run
            catch et
                fprintf('  chart data %s: Tunable not set (%s)\n', want{k,1}, et.message);
            end
        end
    end
    add_block('simulink/Sources/Constant', RE, 'Value', reVal, 'Position', [1225 150 1285 180], ...
              'Description', 'F5b: the real MotiveData reader runs only on the robot with cameras (tools/wire_motive_absolute.m). F7 hook (motive_source).');
    add_line(SS, 'reader_enable/1', 'MATLAB System/1', 'autorouting', 'smart');
    add_line(SS, 'MATLAB System/7', 'MotiveFloor/1', 'autorouting', 'smart');
    add_line(SS, 'MotiveFloor/1', 'Reshape/1', 'autorouting', 'smart');
    add_line(SS, 'MotiveFloor/1', 'Discrete Derivative/1', 'autorouting', 'smart');
    add_line(mdl, 'MotiveReaderEnable/1', [ssN '/1'], 'autorouting', 'smart');
else
    set_param(EN, 'ShowOutputPort', 'on');
    add_block('simulink/Sources/In1', Q0, 'Port', '1', 'Position', [325 -52 355 -38]);
    add_block('simulink/Math Operations/Sum', AD, 'Inputs', '++', 'IconShape', 'rectangular', 'Position', [430 22 460 53]);
    add_block('simulink/Signal Routing/Mux', M1, 'Inputs', '3', 'DisplayOption', 'bar', 'Position', [340 17 345 73]);
    add_block('simulink/Math Operations/Gain', G1, 'Gain', '-1', 'Position', [270 50 300 80]);
    add_block('simulink/Sources/Constant', MI, 'Value', 'q0_9dof(1:3)', 'Position', [1225 150 1285 180]);
    add_line(SS, 'Enable/1', 'MATLAB System/1', 'autorouting', 'smart');
    add_line(SS, 'MATLAB System/1', 'Mux1/1', 'autorouting', 'smart');
    add_line(SS, 'MATLAB System/2', 'Mux1/2', 'autorouting', 'smart');
    add_line(SS, 'MATLAB System/6', 'Multiply2/1', 'autorouting', 'smart');
    add_line(SS, 'Multiply2/1', 'Mux1/3', 'autorouting', 'smart');
    add_line(SS, 'q0/1', 'Add/1', 'autorouting', 'smart');
    add_line(SS, 'Mux1/1', 'Add/2', 'autorouting', 'smart');
    add_line(SS, 'Add/1', 'Reshape/1', 'autorouting', 'smart');
    add_line(SS, 'Add/1', 'Discrete Derivative/1', 'autorouting', 'smart');
    add_line(mdl, 'MotiveInit/1', [ssN '/1'], 'autorouting', 'smart');
end

% ---- compile once (base workspace from the PostLoadFcn define_constants) ------------------------------------------
feval(mdl, [], [], [], 'compile');
try
    d = get_param(MD, 'CompiledPortDimensions');
    fprintf('compiled: MotiveData input dims %s, output dims %s\n', mat2str(d.Inport), mat2str(d.Outport));
    d = get_param(TP, 'CompiledPortDimensions');
    fprintf('compiled: tracked_position input dims %s\n', mat2str(d.Inport));
    if exists(FL)
        d = get_param(FL, 'CompiledPortDimensions');
        fprintf('compiled: MotiveFloor input dims %s, output dims %s\n', mat2str(d.Inport), mat2str(d.Outport));
    end
catch err
    feval(mdl, [], [], [], 'term');
    rethrow(err);
end
feval(mdl, [], [], [], 'term');
h1 = cellfun(@(p) sha(rt.find('-isa', 'Stateflow.EMChart', 'Path', p).Script), chName, 'UniformOutput', false);
assert(isequal(h0, h1), 'wire_motive_absolute:chart', 'an existing chart script changed while wiring; model NOT saved');
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
common = {MT, 1, SS, 1;  ST, 1, MT, 1;  ST, 2, UM, 1;  UK, 1, ST, 1;  TP, 1, RS, 1};   % dst, input, src, output
switch mode
    case 'wire',   want = [{FL, 1, MD, 7;  RS, 1, FL, 1;  DD, 1, FL, 1;  MD, 1, RI, 1;  SS, 1, RE, 1}; common];
    case 'legacy', want = [{AD, 1, Q0, 1;  AD, 2, M1, 1;  M1, 1, MD, 1;  M1, 2, MD, 2;  M1, 3, G1, 1;  G1, 1, MD, 6; ...
                            RS, 1, AD, 1;  DD, 1, AD, 1;  MD, 1, EN, 1;  SS, 1, MI, 1}; common];
end
tags = {'WRONG', 'OK'};
nm = @(b) strrep(strrep(b, [mdl '/'], ''), newline, ' ');   % printable short path (char)
for k = 1:size(want, 1)
    got = '<none>';
    if exists(want{k,1})
        p = srcOf(want{k,1});  s = p(want{k,2});
        if ~isempty(s.SrcBlock) && s.SrcBlock ~= -1, got = sprintf('%s/%d', getfullname(s.SrcBlock), s.SrcPort + 1); end
    end
    exp_ = sprintf('%s/%d', want{k,3}, want{k,4});
    sidD = '-';  sidS = '-';
    if exists(want{k,1}), sidD = get_param(want{k,1}, 'SID'); end
    if exists(want{k,3}), sidS = get_param(want{k,3}, 'SID'); end
    fprintf('  %-34s in %d <- %-34s (SID %s <- SID %s) %s\n', nm(want{k,1}), want{k,2}, nm(got), sidD, sidS, tags{1 + strcmp(got, exp_)});
    if ~strcmp(got, exp_), bad{end+1} = sprintf('%s in %d fed by %s, expected %s', nm(want{k,1}), want{k,2}, nm(got), nm(exp_)); end %#ok<AGROW>
end
pss = srcOf(SS);  pe = pss(strcmp({pss.Type}, 'enable'));   % the subsystem's enable port connectivity
okEn = numel(pe) == 1 && pe.SrcBlock == get_param(C1, 'Handle') && strcmp(get_param(C1, 'Value'), 'EnableMotive');
fprintf('  subsystem enable <- Constant1 (Value %s) %s\n', get_param(C1, 'Value'), tags{1 + okEn});
if ~okEn, bad{end+1} = 'subsystem enable not fed by Constant1 = EnableMotive'; end
blk = {FL, RI, RE, AD, M1, G1, Q0, MI};        % blocks whose presence depends on the mode
present = cellfun(exists, blk);
switch mode
    case 'wire',   wantPresent = [true true true false false false false false];
    case 'legacy', wantPresent = [false false false true true true true true];
end
fprintf('  present [MotiveFloor reader_enable MotiveReaderEnable Add Mux1 Multiply2 q0 MotiveInit] = %s (expected %s)\n', mat2str(present), mat2str(wantPresent));
if ~isequal(present, wantPresent), bad{end+1} = 'block set differs from the mode'; end
sids = {'221', '184', '173', '108', '183', '105'};   % old SIDs (MotiveInit, Add, Gain -1, Mux1, Inport q0) and MotiveData (control)
nS = cellfun(@(s) numel(find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'SID', s)), sids);
fprintf('  blocks with SID 221 184 173 108 183 | 105 (MotiveData, control): %s\n', mat2str(nS));
if nS(6) ~= 1, bad{end+1} = 'SID search cannot find MotiveData SID 105: the SID check is blind'; end
if strcmp(mode, 'wire') && any(nS(1:5) ~= 0), bad{end+1} = 'an old SID (221 184 173 108 183) is still in the model'; end
vals = {get_param(MD, 'System'), get_param(MD, 'SimulateUsing')};
fprintf('  MotiveData System %s, SimulateUsing %s; Enable ShowOutputPort %s\n', vals{:}, get_param(EN, 'ShowOutputPort'));
if ~isequal(vals, {'MotiveData', 'Interpreted execution'}), bad{end+1} = 'MotiveData block System/SimulateUsing changed'; end
if strcmp(mode, 'wire')
    if ~strcmp(get_param(EN, 'ShowOutputPort'), 'off'), bad{end+1} = 'Enable ShowOutputPort not off'; end
    fprintf('  MotiveReaderEnable Value %s\n', get_param(RE, 'Value'));
    if ~strcmp(get_param(RE, 'Value'), reVal), bad{end+1} = 'MotiveReaderEnable Value differs'; end
    fc = rt.find('-isa', 'Stateflow.EMChart', 'Path', FL);
    scr = strrep(fc.Script, char([13 10]), char(10));   % chart script, LF (char)
    sameScr = strcmp(scr, chartTxt);
    fprintf('  MotiveFloor script = %s: %d (%d / %d chars, SHA-256 %s)\n', chartFile, sameScr, numel(scr), numel(chartTxt), sha(scr));
    if ~sameScr, bad{end+1} = 'MotiveFloor script differs from Optitrack/motive_floor_block.m'; end
    dl = fc.find('-isa', 'Stateflow.Data');
    ds = sort(arrayfun(@(d) sprintf('%s:%s', d.Name, d.Scope), dl, 'UniformOutput', false));
    fprintf('  MotiveFloor data: %s\n', strjoin(ds.', ', '));
    if ~isequal(ds(:).', sort({'base:Output', 'motive_cal:Parameter', 'pose7:Input', 'q0_9dof:Parameter'})), bad{end+1} = 'MotiveFloor data differ'; end
else
    if ~strcmp(get_param(EN, 'ShowOutputPort'), 'on'), bad{end+1} = 'Enable ShowOutputPort not on'; end
    if ~strcmp(get_param(MI, 'Value'), 'q0_9dof(1:3)') || ~strcmp(get_param(G1, 'Gain'), '-1'), bad{end+1} = 'legacy block parameters differ'; end
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
    error('wire_motive_absolute:verify', 'after re-read: %s', strjoin(bad, '; '));
end
fprintf('wire_motive_absolute: %s done, re-read and verified\n', mode);
end
