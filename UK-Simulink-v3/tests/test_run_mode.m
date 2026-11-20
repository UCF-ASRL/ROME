function varargout = test_run_mode(brk)
%TEST_RUN_MODE  The one-number mode selector, wired (focus plan F12): run_mode in define_constants, rome_mode, the
%   model's four Mode buttons, and the gate's refusal of any mode but 1.
%   Runs on a TEMPORARY COPY of ROME_merged (tempname). ROME_merged is never written. The copy's ROMECommand.m and
%   Optitrack/MotiveData.m are the no-network stubs of tests/stubs (as test_pacing), so no run of this test can open
%   a serial port or a socket, whatever mode a break leaves in the file. Expected values are LITERALS below, not
%   run_mode_table, so a table bug cannot pass (a).
%   Checks (all evaluated, then one error listing the failed ones):
%     (e) buttons: the copy's model has blocks 'Mode 1'..'Mode 4' of BlockType CustomCallbackButton (Dashboard Callback
%         Button) with ButtonText 'Mode n' and ClickFcn exactly 'rome_mode(n)', and one label annotation (Tag
%         rome_run_mode_label) whose saved text names no mode; JointFromTelemetry's Value is exactly
%         'double(EnableHardware ~= 0 && arm_freeze_user == 0)'.
%     (a) for n = 1..4, model loaded, base workspace cleared, then the BUTTON'S OWN ClickFcn text evaluated: base
%         run_mode, EnableHardware, EnableMotive, motive_source, arm_freeze, arm_freeze_user equal the literal table;
%         the controller arm source (JointFromTelemetry's Value evaluated in base) is 0 0 1 1 (model, model,
%         telemetry, telemetry); dc_sha_at_run equals the SHA-256 of the copy's file; the banner literal is printed and
%         is in the canvas label. Mode 4 runs with motive_cal.measured set true in the copy (one count-asserted edit);
%         with the file's measured false, rome_mode(4) is refused with motive_cal_check:unmeasured, arm_freeze_user
%         is NaN and the label says REFUSED.
%     (c) every rome_mode call of (a) leaves every byte of the copy's define_constants.m unchanged except the run_mode
%         digit (oracle: the bytes before the call with that one byte replaced, found here by strfind).
%     (d) after rome_mode(1), set_run_mode(2) on the file WITHOUT re-running define_constants: the premise holds
%         (dc_sha_at_run matched the file before), then the Run is refused with cause arm_freeze_banner:stale.
%     (b) run_mode = 7 in the copy's file, model loaded fresh: the load warning carries run_mode_table:bad's text
%         ('run_mode must be one of' and all four banners), arm_freeze_user is NaN, EnableHardware is undefined, and
%         the Run is refused with cause apply_pacing:noflag (apply_pacing is first in InitFcn).
%     (f) run_mode = 2 in the copy's file: EVERY runner that simulates the real model (check_9dof, gate_all,
%         run_9dof_cases('sim'), plot_9dof, check_guard) errors assert_sim_mode:notsim naming itself, before any
%         simulation (gate_all once, not once per scenario); the file is byte-identical afterwards, no PRISTINE lock.
%     (g) the copy's define_constants.m (= the real one unless a break edits it) has exactly one run_mode line, one
%         dc_set_mode override anchor, exactly one assignment of each of EnableHardware, EnableMotive, motive_source,
%         arm_freeze, and no 'TEST OVERRIDE'; dc_set_mode refuses 0 or 2 run_mode lines (dc_set_mode:count) and a
%         missing anchor (dc_set_mode:anchor).
%     (e) also: define_constants' own MODE 1 banner is in the load_system output.
%     (j) after a press (label names a mode), save_system + reload: the label is the neutral text again (PreSaveFcn
%         run_mode_label(bdroot)); rome_mode(4) refused leaves the file and the workspace at the PREVIOUS mode.
%     (h) rome_mode(2) called while the model runs (from its StartFcn, try/catch) errors rome_mode:running and leaves
%         every byte of define_constants.m unchanged.
%     (i) arm_freeze_banner on a consistent mode-1 workspace prints the MODE 1 banner; with EnableHardware = 1 typed
%         in the base workspace it refuses arm_freeze_banner:mode naming EnableHardware; with that flag listed in
%         run_mode_test_override it passes and prints the TEST OVERRIDE line.
%   MAKE-IT-FAIL: test_run_mode('<id>') applies one break of the table below to the COPY and must then error with
%   test_run_mode:fail naming its check; test_run_mode('list') returns the table. tests/manual_makefail_f12.m runs them.
%   Flat function without local functions (CLAUDE.md section 5, crash 2). Clears the base workspace.
if nargin < 1 || isempty(brk), brk = ''; end
here = fileparts(mfilename('fullpath'));      % ROME_merged/tests (char)
root = fileparts(here);                       % ROME_merged (char)
mdl  = 'ROME_9DOF';                           % model name (char)
lf   = char(10);                              % line feed (char)
W = {  % n | EnableHardware | EnableMotive | motive_source | arm_freeze | arm from telemetry | banner (plan, THE FOUR MODES; author's words)
  1, 0, 1, 'emulator',  1, 0, 'MODE 1: Simulation only (end-effector dynamics sim, pose data from sim)';
  2, 1, 1, 'emulator',  1, 0, 'MODE 2: ground vehicle moving, frozen arm, camera off (fake camera)';
  3, 1, 1, 'emulator',  0, 1, 'MODE 3: ground vehicle moving, moving arm, camera off (fake camera)';
  4, 1, 1, 'optitrack', 0, 1, 'MODE 4: ground vehicle moving, moving arm, camera on (OptiTrack)'};
JTV  = 'double(EnableHardware ~= 0 && arm_freeze_user == 0)';   % JointFromTelemetry Value wanted (char)
TAG  = 'rome_run_mode_label';                 % canvas label Tag (char)
OVR  = 'arm_freeze     = run_mode_cfg.arm_freeze;';   % dc_set_mode's override anchor (char)
MEAS = '    ''measured'', false);';           % define_constants motive_cal line (char)
B = { ...
  'mf_a', 'a', {{'file', 'define_constants.m', OVR, 'arm_freeze     = run_mode_cfg.EnableHardware;'}}, 'define_constants derives arm_freeze from the wrong field (mode 3 and 4 freeze the arm)'; ...
  'mf_b', 'b', {{'file', 'define_constants.m', 'run_mode_cfg   = run_mode_table(run_mode);', 'run_mode_cfg   = run_mode_table(max(1, min(4, round(run_mode))));'}}, 'an invalid run_mode is clamped into 1-4 instead of refused'; ...
  'mf_c', 'c', {{'file', 'set_run_mode.m', 'uint8(sprintf(''%d'', M.n))', 'uint8(sprintf(''%d '', M.n))'}}, 'set_run_mode writes one extra byte (a space after the digit)'; ...
  'mf_d', 'd', {{'file', 'arm_freeze_banner.m', 'if ~shaOk', 'if false'}}, 'banner SHA check removed: a Run after set_run_mode without define_constants proceeds on the old mode'; ...
  'mf_e', 'e', {{'param', 'Mode 3', 'ClickFcn', 'rome_mode(2)'}}, 'button Mode 3 calls rome_mode(2)'; ...
  'mf_f1', 'f', {{'fileall', 'check_9dof.m', 'assert_sim_mode(', 'fprintf(''%s '', ', 2}}, 'check_9dof no longer asserts run mode 1 (it simulates the hardware-flagged mode; network stubs only here)'; ...
  'mf_f2', 'f', {{'fileall', 'run_9dof_cases.m', 'assert_sim_mode(', 'fprintf(''%s '', ', 3}}, 'run_9dof_cases no longer asserts run mode 1'; ...
  'mf_f3', 'f', {{'fileall', 'plot_9dof.m', 'assert_sim_mode(', 'fprintf(''%s '', ', 2}}, 'plot_9dof no longer asserts run mode 1'; ...
  'mf_f4', 'f', {{'fileall', 'check_guard.m', 'assert_sim_mode(', 'fprintf(''%s '', ', 2}}, 'check_guard no longer asserts run mode 1'; ...
  'mf_f5', 'f', {{'fileall', 'gate_all.m', 'assert_sim_mode(', 'fprintf(''%s '', ', 1}}, 'gate_all no longer refuses once up front (check_9dof then refuses once per scenario: gate_all:failed)'; ...
  'mf_g',  'g', {{'file', 'define_constants.m', OVR, [OVR char(10) 'EnableMotive = 0;  % TEST OVERRIDE (a second assignment)']}}, 'a second assignment of a run flag (with a TEST OVERRIDE mark) in define_constants.m'; ...
  'mf_e2', 'e', {{'file', 'define_constants.m', 'fprintf(''%s\n'', run_mode_cfg.banner);', '% banner print removed'}}, 'define_constants no longer prints the MODE banner at load'; ...
  'mf_j',  'j', {{'param', '', 'PreSaveFcn', ''}}, 'PreSaveFcn emptied: the saved model keeps the last pressed mode in its label'; ...
  'mf_h',  'h', {{'file', 'rome_mode.m', 'if bdIsLoaded(mdl) && ~strcmp(get_param(mdl, ''SimulationStatus''), ''stopped'')', 'if false'}}, 'rome_mode no longer refuses while the model runs'; ...
  'mf_i1', 'i', {{'file', 'arm_freeze_banner.m', 'if ~isempty(badM)', 'if false'}}, 'arm_freeze_banner no longer checks the flags against run_mode'; ...
  'mf_i2', 'i', {{'file', 'arm_freeze_banner.m', 'fprintf(''%s\n'', Mrm.banner);', '% MODE print removed'}}, 'arm_freeze_banner no longer prints the MODE line at a Run'};
if strcmp(brk, 'list'), varargout{1} = B; return; end
ib = [];                                      % row of the break in B (empty: no break)
if ~isempty(brk)
    ib = find(strcmp(B(:,1), brk));
    assert(numel(ib) == 1, 'test_run_mode:brk', 'unknown break ''%s''; test_run_mode(''list'') gives the table', brk);
    fprintf('MAKE-IT-FAIL break %s (check (%s) must FAIL): %s\n', B{ib,1}, B{ib,2}, B{ib,4});
end
fails = {};                                   % one message per failed check (cell of char)

% ---- (g) the real file and the helper (pure, no copy) ------------------------------------------------------------
fid = fopen(fullfile(root, 'define_constants.m'), 'r');  dcReal = char(fread(fid, Inf, '*uint8').');  fclose(fid);
% ---- temporary copy with no-network stubs -------------------------------------------------------------------------
tmp = tempname;                               % temporary copy of ROME_merged (char)
startdir = pwd;  path0 = path;                % restored at exit
if bdIsLoaded(mdl), close_system(mdl, 0); end
mkdir(tmp);
copyfile(root, tmp);
copyfile(fullfile(here, 'stubs', 'ROMECommand.m'), fullfile(tmp, 'ROMECommand.m'), 'f');
copyfile(fullfile(here, 'stubs', 'MotiveData.m'),  fullfile(tmp, 'Optitrack', 'MotiveData.m'), 'f');
mdlcell = {mdl};
cleanup = onCleanup(@() cellfun(@(c) c(), {@() cd(startdir), @() path(path0), ...
          @() cellfun(@(m) close_system(m, 0), mdlcell(bdIsLoaded(mdl))), @() rmdir(tmp, 's')})); %#ok<NASGU>
addpath(here);                                % tests/dc_set_mode.m (behind the copy)
addpath(tmp, fullfile(tmp, 'Ground Vehicle'), fullfile(tmp, 'Optitrack'));
cd(tmp);
evalin('base', 'clear all');
rehash;
fns = {'rome_mode', 'set_run_mode', 'run_mode_table', 'define_constants', 'arm_freeze_banner', 'check_9dof', 'run_9dof_cases', 'ROMECommand', 'MotiveData'};
wh = cellfun(@which, fns, 'UniformOutput', false);
assert(all(startsWith(wh, tmp)), 'test_run_mode:path', 'not resolved inside the copy: %s', strjoin(wh(~startsWith(wh, tmp)), ' | '));
slx = fullfile(tmp, [mdl '.slx']);            % the copy's model (char)
dcf = fullfile(tmp, 'define_constants.m');    % the copy's define_constants.m (char)
fid = fopen(dcf, 'r');  dc0 = char(fread(fid, Inf, '*uint8').');  fclose(fid);   % the copy's text (char)
% (g), second half: dc_set_mode refusals on the real file's text (needs run_mode_table, now on the path from the copy)
idG = cell(1, 3);                             % dc_set_mode error ids: 0 lines, 2 lines, no anchor (char)
for k = 1:3
    switch k
        case 1, tg = regexprep(dcReal, '^run_mode[ \t]*=[ \t]*\d+[ \t]*;', '% run_mode removed', 'lineanchors');  ov = struct();
        case 2, tg = [dcReal lf 'run_mode = 2;' lf];  ov = struct();   % lf first: the file has no final newline
        case 3, tg = strrep(dcReal, OVR, ['% ' OVR]);  ov = struct('EnableMotive', 0);
    end
    idG{k} = '';
    try, dc_set_mode(tg, 1, ov); catch eg, idG{k} = eg.identifier; end
end
okG1 = strcmp(idG{1}, 'dc_set_mode:count') && strcmp(idG{2}, 'dc_set_mode:count') && strcmp(idG{3}, 'dc_set_mode:anchor');   % refusals (logical)

% ---- the break, on the copy only -----------------------------------------------------------------------------------
if ~isempty(ib)
    for e = 1:numel(B{ib,3})
        ed = B{ib,3}{e};
        switch ed{1}
            case 'file'
                f = fullfile(tmp, ed{2});
                fid = fopen(f, 'r');  s = char(fread(fid, Inf, '*uint8').');  fclose(fid);
                n = numel(strfind(s, ed{3}));
                assert(n == 1, 'test_run_mode:brk', 'break edit in %s matches %d times (expected 1)', ed{2}, n);
                fid = fopen(f, 'w');  fwrite(fid, uint8(strrep(s, ed{3}, ed{4})));  fclose(fid);
                fprintf('  break: %s: ''%s'' -> ''%s''\n', ed{2}, ed{3}, ed{4});
            case 'fileall'                    % every occurrence, count asserted (ed{5})
                f = fullfile(tmp, ed{2});
                fid = fopen(f, 'r');  s = char(fread(fid, Inf, '*uint8').');  fclose(fid);
                n = numel(strfind(s, ed{3}));
                assert(n == ed{5}, 'test_run_mode:brk', 'break edit in %s matches %d times (expected %d)', ed{2}, n, ed{5});
                fid = fopen(f, 'w');  fwrite(fid, uint8(strrep(s, ed{3}, ed{4})));  fclose(fid);
                fprintf('  break: %s: %d x ''%s'' -> ''%s''\n', ed{2}, n, ed{3}, ed{4});
            case 'param'
                evalc('load_system(slx)');
                blk = mdl;  if ~isempty(ed{2}), blk = [mdl '/' ed{2}]; end
                fprintf('  break: %s %s ''%s'' -> ''%s''\n', blk, ed{3}, get_param(blk, ed{3}), ed{4});
                set_param(blk, ed{3}, ed{4});
                save_system(mdl);  close_system(mdl, 0);
        end
    end
    clear define_constants set_run_mode arm_freeze_banner check_9dof run_9dof_cases plot_9dof check_guard gate_all rome_mode
    rehash;
    fid = fopen(dcf, 'r');  dc0 = char(fread(fid, Inf, '*uint8').');  fclose(fid);
end
% (g): the copy's file (after the break, if any): one run_mode line, one anchor, one assignment per flag, no TEST OVERRIDE
nRM = numel(regexp(dc0, '^run_mode[ \t]*=[ \t]*\d+[ \t]*;', 'start', 'lineanchors'));   % run_mode lines (count)
nOV = numel(regexp(dc0, ['^' regexptranslate('escape', OVR)], 'start', 'lineanchors'));   % override anchor lines (count)
nFl = cellfun(@(f) numel(regexp(dc0, ['^[ \t]*' f '[ \t]*=(?!=)'], 'start', 'lineanchors')), {'EnableHardware', 'EnableMotive', 'motive_source', 'arm_freeze'});   % assignments per flag (count)
nTO = numel(strfind(dc0, 'TEST OVERRIDE'));   % override marks (count)
okG = okG1 && nRM == 1 && nOV == 1 && all(nFl == 1) && nTO == 0;
fprintf('(g)  copy define_constants.m (= real: %d): run_mode lines %d, override anchor %d, assignments EH/EM/src/arm_freeze %s, TEST OVERRIDE x%d; dc_set_mode on 0 / 2 lines / no anchor: %s %s %s -> %s\n', ...
        strcmp(dc0, dcReal), nRM, nOV, mat2str(nFl), nTO, idG{:}, mat2str(okG));
if ~okG, fails{end+1} = '(g) run_mode line / override anchor / one assignment per flag / no TEST OVERRIDE / dc_set_mode refusals'; end
nMeas = numel(strfind(dc0, MEAS));
assert(nMeas == 1, 'test_run_mode:dc', 'copy define_constants.m: motive_cal measured line x%d (expected 1)', nMeas);
dc1 = dc_set_mode(dc0, 1);                    % run_mode 1, motive_cal.measured false (char)
dcA = strrep(dc1, MEAS, '    ''measured'', true);');   % (a): mode 4 reachable (char)

% ---- (e) buttons, label and JointFromTelemetry, read back from the copy's model ----------------------------------
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dcA));  fclose(fid);
clear define_constants;  rehash;
evalin('base', 'clear all');
ltxtE = evalc('load_system(slx)');            % PostLoadFcn = the copy's define_constants (mode 1); its printed text (char)
okBan = contains(ltxtE, W{1,7});
fprintf('(e)  define_constants prints ''%s'' at load: %d\n', W{1,7}, okBan);
cf = cell(1, 4);  okE = okBan;                 % ClickFcn per button (char), result (logical)
for n = 1:4
    b = [mdl '/Mode ' num2str(n)];
    if getSimulinkBlockHandle(b) == -1
        fprintf('(e)  %s missing\n', b);  okE = false;  cf{n} = '';  continue
    end
    bt = get_param(b, 'BlockType');  tx = get_param(b, 'ButtonText');  cf{n} = get_param(b, 'ClickFcn');
    ok = strcmp(bt, 'CustomCallbackButton') && strcmp(tx, ['Mode ' num2str(n)]) && strcmp(cf{n}, sprintf('rome_mode(%d)', n));
    fprintf('(e)  %s: BlockType %s, ButtonText ''%s'', ClickFcn ''%s'' -> %s\n', b, bt, tx, cf{n}, mat2str(ok));
    okE = okE && ok;
end
hl = find_system(mdl, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'annotation', 'Tag', TAG);   % label handle(s)
lab0 = '';  if numel(hl) == 1, lab0 = get_param(hl, 'Text'); end
jtv = get_param([mdl '/JointFromTelemetry'], 'Value');   % the controller arm-source condition (char)
okL = numel(hl) == 1 && ~contains(lab0, 'MODE ');
fprintf('(e)  label: %d annotation(s) with Tag %s, saved text names no mode %d; JointFromTelemetry Value ''%s'' (want ''%s'')\n', ...
        numel(hl), TAG, okL, jtv, JTV);
okE = okE && okL && strcmp(jtv, JTV);
if ~okE, fails{end+1} = '(e) buttons / label / JointFromTelemetry Value'; end

% ---- (a) and (c): each mode through its button's ClickFcn, from a cleared base ----------------------------------
okA = true;  okC = true;
for n = 1:4
    evalin('base', 'clear all');
    fid = fopen(dcf, 'r');  bb = fread(fid, Inf, '*uint8').';  fclose(fid);   % bytes before (uint8 row)
    ix = strfind(char(bb), [lf 'run_mode = ']);   % the run_mode line start (index)
    exp_ = [];  if numel(ix) == 1, exp_ = bb;  exp_(ix + 12) = uint8('0' + n); end   % expected bytes after (uint8 row)
    out = '';  err = '';
    try, out = evalc(cf{n}); catch ex, err = [ex.identifier ': ' ex.message]; end
    fid = fopen(dcf, 'r');  ba = fread(fid, Inf, '*uint8').';  fclose(fid);   % bytes after (uint8 row)
    v = struct();
    for nmv = {'run_mode', 'EnableHardware', 'EnableMotive', 'motive_source', 'arm_freeze', 'arm_freeze_user', 'dc_sha_at_run'}
        try, v.(nmv{1}) = evalin('base', nmv{1}); catch, v.(nmv{1}) = 'undefined'; end
    end
    js = NaN;  try, js = evalin('base', jtv); catch, end   % controller arm from telemetry (1) or the model (0)
    labn = '';  if numel(hl) == 1, labn = get_param(hl, 'Text'); end
    ok = isempty(err) && isequal(v.run_mode, n) && isequal(v.EnableHardware, W{n,2}) && isequal(v.EnableMotive, W{n,3}) && ...
         isequal(v.motive_source, W{n,4}) && isequal(v.arm_freeze, W{n,5}) && isequal(v.arm_freeze_user, W{n,5}) && ...
         isequal(js, W{n,6}) && ischar(v.dc_sha_at_run) && strcmp(v.dc_sha_at_run, dc_file_sha(dcf)) && ...
         contains(out, W{n,7}) && contains(labn, W{n,7});
    okc = numel(ix) == 1 && isequal(ba, exp_);
    fprintf('(a)  %s: run_mode %s EH %s EM %s src %s arm_freeze %s user %s, arm from telemetry %s (want %d), SHA ok %d, banner printed %d, label %d, error ''%s'' -> %s\n', ...
            cf{n}, mat2str(v.run_mode), mat2str(v.EnableHardware), mat2str(v.EnableMotive), mat2str(v.motive_source), mat2str(v.arm_freeze), ...
            mat2str(v.arm_freeze_user), mat2str(js), W{n,6}, ischar(v.dc_sha_at_run) && strcmp(v.dc_sha_at_run, dc_file_sha(dcf)), ...
            contains(out, W{n,7}), contains(labn, W{n,7}), err, mat2str(ok));
    fprintf('(c)  %s: %d bytes before, %d after, only the run_mode digit changed -> %s\n', cf{n}, numel(bb), numel(ba), mat2str(okc));
    okA = okA && ok;  okC = okC && okc;
end
% mode 4 with the file's own motive_cal (measured false): refused
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc1));  fclose(fid);
clear define_constants;  rehash;
evalin('base', 'clear all');
fid = fopen(dcf, 'r');  bb = fread(fid, Inf, '*uint8').';  fclose(fid);
ix = strfind(char(bb), [lf 'run_mode = ']);
exp_ = [];  if numel(ix) == 1, exp_ = bb;  exp_(ix + 12) = uint8('4'); end
id4 = '';  try, evalc('rome_mode(4)'); catch ex, id4 = ex.identifier; end
fid = fopen(dcf, 'r');  ba = fread(fid, Inf, '*uint8').';  fclose(fid);
afu4 = 'undefined';  try, afu4 = evalin('base', 'arm_freeze_user'); catch, end
lab4 = '';  if numel(hl) == 1, lab4 = get_param(hl, 'Text'); end
rm4 = 'undefined';  try, rm4 = evalin('base', 'run_mode'); catch, end
ok4 = strcmp(id4, 'motive_cal_check:unmeasured') && isequal(afu4, 1) && isequal(rm4, 1) && contains(lab4, 'MODE 4 REFUSED') && contains(lab4, 'MODE 1 restored');
okc4 = isequal(ba, bb);                       % fix round 1: the previous mode is written back
fprintf('(a)  rome_mode(4) with motive_cal.measured false: error ''%s'' (want motive_cal_check:unmeasured), then run_mode %s, arm_freeze_user %s (want 1, 1: mode 1 restored), label ''%s'' -> %s\n', ...
        id4, mat2str(rm4), mat2str(afu4), lab4, mat2str(ok4));
fprintf('(c)  rome_mode(4) refused: file byte-identical to before the call (previous mode restored) -> %s\n', mat2str(okc4));
% (j) the label is never saved with a mode
evalc('rome_mode(2)');
labP = '';  if numel(hl) == 1, labP = get_param(hl, 'Text'); end   % label after a press (char)
save_system(mdl);  close_system(mdl, 0);
evalc('load_system(slx)');
hl = find_system(mdl, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'annotation', 'Tag', TAG);
labS = '';  if numel(hl) == 1, labS = get_param(hl, 'Text'); end   % label after save + reload (char)
okJ = contains(labP, W{2,7}) && strcmp(labS, run_mode_label()) && ~contains(labS, 'MODE ');
fprintf('(j)  label after rome_mode(2): names MODE 2 %d; after save_system + reload: neutral %d -> %s\n', contains(labP, W{2,7}), strcmp(labS, run_mode_label()), mat2str(okJ));
if ~okJ, fails{end+1} = '(j) label saved with a mode'; end
okA = okA && ok4;  okC = okC && okc4;
if ~okA, fails{end+1} = '(a) mode flags / controller arm source / banner / label per button'; end
if ~okC, fails{end+1} = '(c) bytes of define_constants.m other than the run_mode digit'; end

% ---- (d) set_run_mode without re-running define_constants: the Run is refused (stale) -----------------------------
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc1));  fclose(fid);
clear define_constants;  rehash;
evalin('base', 'clear all');
evalc('rome_mode(1)');
pre = strcmp(evalin('base', 'dc_sha_at_run'), dc_file_sha(dcf));   % premise: the workspace matches the file (logical)
% ---- (h) rome_mode refuses while the model runs (called from StartFcn, the model running) ----
sf0 = get_param(mdl, 'StartFcn');             % restored below (char)
set_param(mdl, 'StartFcn', [sf0 '; try, rome_mode(2); assignin(''base'', ''rm_running_id'', ''none''); catch e_rm, assignin(''base'', ''rm_running_id'', e_rm.identifier); end']);
fid = fopen(dcf, 'r');  bh0 = fread(fid, Inf, '*uint8').';  fclose(fid);   % bytes before (uint8 row)
evalin('base', 'rm_running_id = ''not called'';');
hErr = '';  try, evalc('sim(mdl, ''StopTime'', ''0.1'', ''SimulationMode'', ''normal'');'); catch ex, hErr = ex.identifier; end
set_param(mdl, 'StartFcn', sf0);
rmid = evalin('base', 'rm_running_id');
fid = fopen(dcf, 'r');  bh1 = fread(fid, Inf, '*uint8').';  fclose(fid);
okH = strcmp(rmid, 'rome_mode:running') && isequal(bh0, bh1) && isempty(hErr);
fprintf('(h)  rome_mode(2) from StartFcn during a run: id ''%s'' (want rome_mode:running), file unchanged %d, sim error ''%s'' -> %s\n', rmid, isequal(bh0, bh1), hErr, mat2str(okH));
if ~okH, fails{end+1} = '(h) rome_mode during a run'; end
evalin('base', 'clear rm_running_id');
evalin('base', 'clear all');  evalc('rome_mode(1)');   % fresh, consistent mode 1 again
% ---- (i) the banner checks the flags against run_mode and prints the MODE line ----
iTxt = '';  iId = '';  try, iTxt = evalc('arm_freeze_banner(mdl)'); catch ex, iId = ex.identifier; end
assignin('base', 'EnableHardware', 1);        % a student types a flag instead of choosing the mode
i2Id = '';  i2Msg = '';  try, evalc('arm_freeze_banner(mdl)'); catch ex, i2Id = ex.identifier;  i2Msg = ex.message; end
assignin('base', 'run_mode_test_override', {'EnableHardware'});
i3Txt = '';  i3Id = '';  try, i3Txt = evalc('arm_freeze_banner(mdl)'); catch ex, i3Id = ex.identifier; end
assignin('base', 'EnableHardware', 0);  assignin('base', 'run_mode_test_override', {});
okI = isempty(iId) && numel(strfind(iTxt, W{1,7})) == 1 && strcmp(i2Id, 'arm_freeze_banner:mode') && contains(i2Msg, 'EnableHardware') && ...
      isempty(i3Id) && contains(i3Txt, 'TEST OVERRIDE');
fprintf('(i)  banner, mode 1: error ''%s'', MODE 1 line x%d; EnableHardware = 1 typed: ''%s'' names it %d; listed in run_mode_test_override: error ''%s'', TEST OVERRIDE printed %d -> %s\n', ...
        iId, numel(strfind(iTxt, W{1,7})), i2Id, contains(i2Msg, 'EnableHardware'), i3Id, contains(i3Txt, 'TEST OVERRIDE'), mat2str(okI));
if ~okI, fails{end+1} = '(i) banner mode check / MODE line'; end
pre = pre && strcmp(evalin('base', 'dc_sha_at_run'), dc_file_sha(dcf));
evalc('set_run_mode(2, dcf)');
ids = {};  simErr = '';
try
    evalc('sim(mdl, ''SimulationMode'', ''normal'', ''ReturnWorkspaceOutputs'', ''on'');');
catch ex
    simErr = ex.message;  ids = {ex.identifier};  st = ex.cause(:).';
    while ~isempty(st), c = st{1};  st(1) = [];  ids{end+1} = c.identifier;  st = [st, c.cause(:).']; end %#ok<AGROW>
end
okD = pre && any(strcmp(ids, 'arm_freeze_banner:stale'));
fprintf('(d)  premise SHA match %d; after set_run_mode(2) without define_constants the Run: ids %s -> %s\n', pre, strjoin(ids, ', '), mat2str(okD));
if ~okD, fails{end+1} = sprintf('(d) stale-SHA refusal after set_run_mode without a re-run (%s)', simErr(1:min(end, 200))); end
close_system(mdl, 0);

% ---- (b) an invalid run_mode in the file is refused at load ------------------------------------------------------
nR1 = numel(regexp(dc1, '^run_mode = 1;', 'start', 'lineanchors'));
assert(nR1 == 1, 'test_run_mode:dc', 'run_mode = 1 line x%d (expected 1)', nR1);
fid = fopen(dcf, 'w');  fwrite(fid, uint8(regexprep(dc1, '^run_mode = 1;', 'run_mode = 7;', 'once', 'lineanchors')));  fclose(fid);
clear define_constants;  rehash;
evalin('base', 'clear all');
ltxt = evalc('load_system(slx)');             % PostLoadFcn errors -> Simulink warning, model still loads (probe log)
ehDef = evalin('base', 'exist(''EnableHardware'', ''var'')');
afuB = 'undefined';  try, afuB = evalin('base', 'arm_freeze_user'); catch, end
idsB = {};
try
    evalc('sim(mdl, ''SimulationMode'', ''normal'', ''ReturnWorkspaceOutputs'', ''on'');');
catch ex
    idsB = {ex.identifier};  st = ex.cause(:).';
    while ~isempty(st), c = st{1};  st(1) = [];  idsB{end+1} = c.identifier;  st = [st, c.cause(:).']; end %#ok<AGROW>
end
banOk = all(cellfun(@(s) contains(ltxt, s), W(:,7)));
okB = contains(ltxt, 'run_mode must be one of') && banOk && ehDef == 0 && isnumeric(afuB) && isscalar(afuB) && isnan(afuB) && ...
      any(strcmp(idsB, 'apply_pacing:noflag'));
fprintf('(b)  run_mode = 7: load text names run_mode_table:bad %d, lists the 4 modes %d; EnableHardware defined %d; arm_freeze_user %s; Run ids %s -> %s\n', ...
        contains(ltxt, 'run_mode must be one of'), banOk, ehDef, mat2str(afuB), strjoin(idsB, ', '), mat2str(okB));
if ~okB, fails{end+1} = '(b) invalid run_mode refused at load'; end
close_system(mdl, 0);

% ---- (f) the gate and the case runner refuse any mode but 1 -------------------------------------------------------
dc2 = dc_set_mode(dc0, 2);                    % run_mode 2 (char)
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc2));  fclose(fid);
clear define_constants;  rehash;
evalin('base', 'clear all');
RN = {'check_9dof', 'check_9dof();'; 'gate_all', 'gate_all();'; 'run_9dof_cases', 'run_9dof_cases(''sim'');'; ...
      'plot_9dof', 'plot_9dof();'; 'check_guard', 'check_guard();'};   % runner | call
okF = true;
for k = 1:size(RN, 1)
    evalin('base', 'clear all');
    idF = '';  msF = '';
    try, evalc(RN{k,2}); catch ex, idF = ex.identifier;  msF = ex.message; end
    cd(tmp);
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    close all force
    fid = fopen(dcf, 'r');  dcAfter = char(fread(fid, Inf, '*uint8').');  fclose(fid);
    ok = strcmp(idF, 'assert_sim_mode:notsim') && startsWith(msF, RN{k,1}) && contains(msF, 'rome_mode(2)') && ...
         strcmp(dcAfter, dc2) && ~isfile(fullfile(tmp, 'define_constants.PRISTINE'));
    fprintf('(f)  run_mode = 2: %s -> error ''%s'' (want assert_sim_mode:notsim naming %s, how to switch back), file unchanged %d, no lock %d -> %s\n', ...
            RN{k,1}, idF, RN{k,1}, strcmp(dcAfter, dc2), ~isfile(fullfile(tmp, 'define_constants.PRISTINE')), mat2str(ok));
    okF = okF && ok;
end
if ~okF, fails{end+1} = '(f) a runner simulates a mode other than 1'; end
if bdIsLoaded(mdl), close_system(mdl, 0); end
evalin('base', 'clear all');

% ---- verdict ---------------------------------------------------------------------------------------------------------
if ~isempty(ib)
    named = any(startsWith(fails, ['(' B{ib,2} ')']));
    if named
        error('test_run_mode:fail', 'MAKE-IT-FAIL %s: FAILED AS REQUIRED at (%s); failed: %s', B{ib,1}, B{ib,2}, strjoin(fails, ' | '));
    end
    error('test_run_mode:notcaught', 'MAKE-IT-FAIL %s: check (%s) did NOT fail; failed: %s', B{ib,1}, B{ib,2}, strjoin(fails, ' | '));
end
if ~isempty(fails)
    error('test_run_mode:fail', '%d check(s) failed: %s', numel(fails), strjoin(fails, ' | '));
end
fprintf('test_run_mode PASS (a)-(j)\n');
end
