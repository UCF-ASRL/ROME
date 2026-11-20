function wire_run_mode_buttons(slxFile)
%WIRE_RUN_MODE_BUTTONS  Put the four run-mode buttons and the mode label on the top level of ROME_9DOF (focus plan F12).
%   wire_run_mode_buttons(slxFile)  slxFile = path of ROME_9DOF.slx (char). Re-runnable: existing 'Mode 1'..'Mode 4'
%   blocks and the label are deleted and added again.
%   Adds, above the left end of the existing top-level blocks:
%     'Mode 1' .. 'Mode 4'  Dashboard Callback Button (simulink_hmi_blocks/Callback Button; BlockType
%                           CustomCallbackButton), ButtonText 'Mode k', ClickFcn 'rome_mode(k)'. Created, saved and read
%                           back headless in R2026a (docs/verification/2026-11-19_f12_probe_buttons.log).
%     label                 annotation, Tag 'rome_run_mode_label', with neutral text: rome_mode writes the mode into
%                           it when a button is pressed; the saved text names no mode, so a freshly opened model never
%                           shows a stale one (define_constants prints the mode at every load).
%   Nothing else is changed. Before saving, it records every Stateflow chart script and the model parameters below;
%   after save, close and reload it asserts that the scripts and parameters are unchanged, that exactly the four
%   buttons and one label were added (top-level block count + 4), and that each button reads back exactly.
%   Errors: wire_run_mode_buttons:* on any mismatch (the model is then left as saved; inspect it).
mdl = 'ROME_9DOF';                                        % model name (char)
TAG = 'rome_run_mode_label';                              % label annotation Tag (char)
LABEL = run_mode_label();                                 % saved (neutral) label text, defined once in run_mode_label.m (char)
PSF = 'run_mode_label(bdroot)';                           % PreSaveFcn: the label is reset to LABEL at every save (fix round 1) (char)
PAR = {'StopTime', 'Solver', 'FixedStep', 'EnablePacing', 'PacingRate', 'InitFcn', 'StartFcn', 'StopFcn', 'PostLoadFcn', 'PreLoadFcn'};   % model parameters that must not change
if bdIsLoaded(mdl), close_system(mdl, 0); end
evalc('load_system(slxFile)');                            % PostLoadFcn define_constants output captured
[pBefore, sBefore, nBefore] = snapshot(mdl, PAR);
% ---- remove an earlier wiring ----
for k = 1:4
    b = [mdl '/Mode ' num2str(k)];
    if getSimulinkBlockHandle(b) ~= -1, delete_block(b); end
end
ha = find_system(mdl, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'annotation', 'Tag', TAG);   % earlier label(s) (handles)
for k = 1:numel(ha), delete(get_param(ha(k), 'Object')); end
nOwn = numel(find_system(mdl, 'SearchDepth', 1, 'Type', 'block'));   % top-level blocks without ours (count)
% ---- place: above the top-left of the existing blocks ----
pos = cell2mat(get_param(find_system(mdl, 'SearchDepth', 1, 'Type', 'block'), 'Position'));   % Nx4 [l t r b] (px)
x0 = min(pos(:,1));  y0 = min(pos(:,2));                  % top-left corner of the existing diagram (px)
for k = 1:4
    h = add_block('simulink_hmi_blocks/Callback Button', [mdl '/Mode ' num2str(k)], ...
                  'Position', [x0 + (k-1)*140, y0 - 90, x0 + (k-1)*140 + 120, y0 - 50]);
    set_param(h, 'ButtonText', ['Mode ' num2str(k)], 'ClickFcn', sprintf('rome_mode(%d)', k));
end
a = Simulink.Annotation([mdl '/' LABEL]);                 % canvas label (object)
a.Tag = TAG;
a.Position = [x0, y0 - 150, x0 + 900, y0 - 110];
set_param(mdl, 'PreSaveFcn', PSF);
save_system(mdl);
close_system(mdl, 0);
% ---- re-read from disk and assert ----
evalc('load_system(slxFile)');
[pAfter, sAfter, nAfter] = snapshot(mdl, PAR);
for k = 1:numel(PAR)
    if ~isequal(pBefore.(PAR{k}), pAfter.(PAR{k}))
        error('wire_run_mode_buttons:param', '%s changed: ''%s'' -> ''%s''', PAR{k}, pBefore.(PAR{k}), pAfter.(PAR{k}));
    end
end
assert(isequal(sBefore, sAfter), 'wire_run_mode_buttons:chart', 'a Stateflow chart script changed on save');
assert(nAfter == nOwn + 4, 'wire_run_mode_buttons:count', 'top-level blocks %d after, expected %d (+4 buttons)', nAfter, nOwn + 4);
for k = 1:4
    b = [mdl '/Mode ' num2str(k)];
    assert(getSimulinkBlockHandle(b) ~= -1, 'wire_run_mode_buttons:button', '%s missing after reload', b);
    bt = get_param(b, 'BlockType');  tx = get_param(b, 'ButtonText');  cf = get_param(b, 'ClickFcn');
    assert(strcmp(bt, 'CustomCallbackButton') && strcmp(tx, ['Mode ' num2str(k)]) && strcmp(cf, sprintf('rome_mode(%d)', k)), ...
           'wire_run_mode_buttons:button', '%s: BlockType %s, ButtonText ''%s'', ClickFcn ''%s''', b, bt, tx, cf);
    fprintf('  %s: %s, ButtonText ''%s'', ClickFcn ''%s''\n', b, bt, tx, cf);
end
hl = find_system(mdl, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'annotation', 'Tag', TAG);
assert(numel(hl) == 1 && strcmp(get_param(hl, 'Text'), LABEL), 'wire_run_mode_buttons:label', 'label: %d annotations with Tag %s', numel(hl), TAG);
fprintf('  label (Tag %s): ''%s''\n', TAG, get_param(hl, 'Text'));
assert(strcmp(get_param(mdl, 'PreSaveFcn'), PSF), 'wire_run_mode_buttons:presave', 'PreSaveFcn is ''%s'', expected ''%s''', get_param(mdl, 'PreSaveFcn'), PSF);
fprintf('  PreSaveFcn: ''%s''\n', PSF);
fprintf('  unchanged after save/reload: %s; %d chart scripts; top-level blocks %d -> %d\n', strjoin(PAR, ', '), numel(sAfter), nBefore, nAfter);
close_system(mdl, 0);
fprintf('wire_run_mode_buttons: %s wired and re-read OK\n', slxFile);
end

function [p, s, n] = snapshot(mdl, PAR)
%SNAPSHOT  Model parameters PAR (struct of char), every Stateflow chart script sorted by path (cell), top-level block count.
p = struct();
for k = 1:numel(PAR), p.(PAR{k}) = get_param(mdl, PAR{k}); end
ch = find(sfroot, '-isa', 'Stateflow.EMChart');           % MATLAB Function charts of loaded models (objects)
ch = ch(arrayfun(@(c) startsWith(c.Path, [mdl '/']), ch));
[~, o] = sort(arrayfun(@(c) c.Path, ch, 'UniformOutput', false));
s = arrayfun(@(c) {c.Path, c.Script}, ch(o), 'UniformOutput', false);
n = numel(find_system(mdl, 'SearchDepth', 1, 'Type', 'block'));
end
