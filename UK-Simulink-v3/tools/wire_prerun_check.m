function wire_prerun_check(slxFile, initFcn)
%WIRE_PRERUN_CHECK  Set ROME_9DOF's InitFcn to apply_pacing, arm_freeze_banner, then prerun_check (focus plan F2/F11 wiring).
%   wire_prerun_check(slxFile)           slxFile = path of ROME_9DOF.slx (char). Re-runnable. InitFcn becomes
%                                        'apply_pacing(bdroot); arm_freeze_banner(bdroot); prerun_check(bdroot)'.
%   wire_prerun_check(slxFile, initFcn)  sets another InitFcn text (make-it-fail: the F1 text without prerun_check).
%   Nothing else is changed. Before saving it records every Stateflow chart script (SHA-256), the top-level block count
%   and the model parameters below; after save, close and reload it asserts that they are unchanged and that InitFcn
%   reads back exactly. Errors wire_prerun_check:* on any mismatch (the model is then left as saved; inspect it).
%   Flat function without local functions (CLAUDE.md section 5, crash 2).
if nargin < 2, initFcn = 'apply_pacing(bdroot); arm_freeze_banner(bdroot); prerun_check(bdroot)'; end
[~, mdl] = fileparts(slxFile);                            % model name (char)
PAR = {'StopTime', 'Solver', 'FixedStep', 'EnablePacing', 'PacingRate', 'StopFcn', 'StartFcn', 'PostLoadFcn', 'PreLoadFcn', ...
       'PreSaveFcn', 'ReturnWorkspaceOutputs', 'ReturnWorkspaceOutputsName'};   % model parameters that must not change
sha = @(s) lower(reshape(dec2hex(typecast(java.security.MessageDigest.getInstance('SHA-256').digest(uint8(unicode2native(s, 'UTF-8'))), 'uint8'), 2).', 1, []));   % SHA-256 of a char (hex char)
if bdIsLoaded(mdl), close_system(mdl, 0); end
evalc('load_system(slxFile)');                            % PostLoadFcn define_constants output captured
p0 = cellfun(@(q) get_param(mdl, q), PAR, 'UniformOutput', false);   % parameters before (cell)
ch = sfroot().find('-isa', 'Stateflow.EMChart');          % MATLAB Function charts
ch = ch(arrayfun(@(c) startsWith(c.Path, [mdl '/']), ch));
[~, ix] = sort(arrayfun(@(c) c.Path, ch, 'UniformOutput', false));  ch = ch(ix);
s0 = arrayfun(@(c) sha(c.Script), ch, 'UniformOutput', false);      % chart script hashes before (cellstr)
n0 = numel(find_system(mdl, 'SearchDepth', 1, 'Type', 'block'));   % top-level blocks before (count)
f0 = get_param(mdl, 'InitFcn');                           % InitFcn before (char)
set_param(mdl, 'InitFcn', initFcn);
save_system(mdl);
close_system(mdl, 0);
evalc('load_system(slxFile)');
p1 = cellfun(@(q) get_param(mdl, q), PAR, 'UniformOutput', false);
ch = sfroot().find('-isa', 'Stateflow.EMChart');
ch = ch(arrayfun(@(c) startsWith(c.Path, [mdl '/']), ch));
[~, ix] = sort(arrayfun(@(c) c.Path, ch, 'UniformOutput', false));  ch = ch(ix);
s1 = arrayfun(@(c) sha(c.Script), ch, 'UniformOutput', false);
n1 = numel(find_system(mdl, 'SearchDepth', 1, 'Type', 'block'));
f1 = get_param(mdl, 'InitFcn');
close_system(mdl, 0);
fprintf('wire_prerun_check: %s\n  InitFcn ''%s'' -> ''%s''\n  %d chart scripts, %d top-level blocks; parameters %s\n', ...
        slxFile, f0, f1, numel(s1), n1, strjoin(PAR, ', '));
for k = 1:numel(PAR)
    assert(isequal(p0{k}, p1{k}), 'wire_prerun_check:param', '%s changed: %s -> %s', PAR{k}, mat2str(p0{k}), mat2str(p1{k}));
end
assert(isequal(s0, s1), 'wire_prerun_check:chart', 'a chart script changed on save');
assert(n0 == n1, 'wire_prerun_check:count', 'top-level block count %d -> %d', n0, n1);
assert(strcmp(f1, initFcn), 'wire_prerun_check:initfcn', 'InitFcn reads back ''%s'', expected ''%s''', f1, initFcn);
fprintf('wire_prerun_check: done, re-read and verified\n');
end
