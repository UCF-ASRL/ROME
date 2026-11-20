function varargout = test_motive_source(brk)
%TEST_MOTIVE_SOURCE  motive_source and the camera-calibration refusal after focus plan F7 (and F5b review I1).
%   Pure checks plus one loaded (not simulated) model, all in a TEMPORARY COPY of ROME_merged (no network: the copy's
%   ROMECommand.m is tests/stubs_hw/ROMECommand.m with RobotLinkRecorder). ROME_merged itself is never written.
%   Checks (all evaluated, then one error listing the failed ones):
%     (iv)  narrowing (motive_cal_check, both forms). Argument form motive_cal_check(EH, EM, src, cal):
%           EH1 EM1 'optitrack' measured false -> motive_cal_check:unmeasured; 'emulator' measured false and
%           'emulator' with cal [] -> no error; EH1 EM0 / EH0 EM1 'optitrack' -> no error; 'optitrack' with a valid
%           measured cal -> no error; R = 2 I measured -> badcal; cal [] -> nocal; EH NaN or 'x' -> badflag; src 'foo'
%           -> motive_source_check:invalid. Base form motive_cal_check(): empty base -> noflag; EH NaN -> badflag;
%           EH1 EM1 without motive_source -> nosource; 'emulator' measured false -> no error; 'optitrack' -> unmeasured.
%     (v)   invalid motive_source is refused: motive_source_check accepts exactly 'emulator' (1), 'optitrack' (0),
%           "emulator" (string, 1) and refuses 'Emulator', 'foo', '', 1, {'emulator'}, a 2x8 char with
%           motive_source_check:invalid; the copy's define_constants.m with motive_source = 'foo', run in the BASE
%           workspace, errors with that id and leaves arm_freeze_user NaN (so the banner refuses the model too); a sim
%           of the loaded copy with motive_source = 'foo' set in the workspace (EnableHardware 0, EnableMotive 0) is
%           refused at compile, message naming motive_source (the Constant blocks PoseSelect / MotiveReaderEnable).
%     (I1)  define_constants passes its OWN values (F5b review I1, 19 Nov 2026). The copy's define_constants.m is run
%           INSIDE A FUNCTION WORKSPACE (a helper function file written into the copy, f7_dc_fn.m: evalc of the
%           script, then the values read there), model not loaded:
%           a  fresh base (clear all), the file as it is (EnableHardware 0) -> no error, arm_freeze_user = arm_freeze;
%           s  STALE base (EnableHardware 1, EnableMotive 1, 'optitrack', motive_cal unmeasured in the base), the
%              file as it is -> no error (the file's own values decide);
%           b  fresh base, file EnableHardware 1, EnableMotive 1, 'optitrack' (motive_cal unmeasured) ->
%              motive_cal_check:unmeasured (the define_constants refusal site);
%           c  fresh base, file EnableHardware 1, EnableMotive 1, 'emulator' -> no error, arm_freeze_user finite.
%           La / Ls  model LOADED (so define_constants also calls apply_pacing), base cleared (La) or stale with
%              EnableHardware 1, EnableMotive 1, sensor_model 1, 'optitrack' (Ls), the file as it is -> no error
%              (F7 fix round 1: define_constants passes its own values to apply_pacing too).
%     (pac) the apply_pacing refusal site (model InitFcn/StartFcn), copy model loaded: base EnableHardware 1,
%           EnableMotive 1, sensor_model 0, unmeasured cal: apply_pacing(mdl) with 'optitrack' ->
%           motive_cal_check:unmeasured; with 'emulator' -> no error; and sim() of the model with 'optitrack' is
%           refused before it runs, the message naming motive_cal (set in the base AFTER define_constants ran, the
%           student who skips define_constants).
%   MAKE-IT-FAIL: test_motive_source('<id>') applies one break to the COPY (text edit, exactly one match) and must
%   then error naming its check; test_motive_source('list') returns the table.
%   Flat function without local functions (CLAUDE.md section 5, crash 2). Clears the base workspace.
if nargin < 1 || isempty(brk), brk = ''; end
here = fileparts(mfilename('fullpath'));      % ROME_merged/tests (char)
root = fileparts(here);                       % ROME_merged (char)
mdl  = 'ROME_9DOF';                           % model name (char)
lf   = char(10);                              % line feed (char)
dcCall = 'motive_cal_check(EnableHardware, EnableMotive, motive_source, motive_cal);';   % define_constants call site (char)
B = { ...
  'mf_dcbase',    'I1',  {{'define_constants.m', dcCall, 'motive_cal_check();'}}, 'define_constants back to the base-workspace read (the F5b form): fresh base -> noflag'; ...
  'mf_dcsite',    'I1',  {{'define_constants.m', dcCall, '% refusal call removed (make-it-fail)'}}, 'define_constants refusal site removed'; ...
  'mf_dcpacbase', 'I1',  {{'define_constants.m', 'apply_pacing(''ROME_9DOF'', struct(', 'apply_pacing(''ROME_9DOF''); f7_unused = struct('}, ...
                          {'define_constants.m', '''motive_cal'', motive_cal));', '''motive_cal'', motive_cal);'}}, 'define_constants calls apply_pacing with the BASE read again (model loaded): fresh base -> noflag'; ...
  'mf_pacsite',   'pac', {{'apply_pacing.m', ['    motive_cal_check();' lf], ['    % refusal call removed (make-it-fail)' lf]}}, 'apply_pacing refusal site removed'; ...
  'mf_narrow',    'iv',  {{'motive_cal_check.m', 'if motive_source_check(motive_source) == 1', 'if false && motive_source_check(motive_source) == 1'}}, 'refusal not narrowed: the emulator is refused too'; ...
  'mf_anysource', 'v',   {{'motive_source_check.m', ['if ~ok' lf '    if ischar'], ['if false' lf '    if ischar']}}, 'motive_source_check accepts any text'};
if strcmp(brk, 'list'), varargout{1} = B; return; end
ib = [];                                      % row of the break in B (empty: no break)
if ~isempty(brk)
    ib = find(strcmp(B(:,1), brk));
    assert(numel(ib) == 1, 'test_motive_source:brk', 'unknown break ''%s''; test_motive_source(''list'') gives the table', brk);
    fprintf('MAKE-IT-FAIL break %s (check (%s) must FAIL): %s\n', B{ib,1}, B{ib,2}, B{ib,4});
end

% ---- temporary copy ------------------------------------------------------------------------------------------------
tmp = tempname;                               % temporary copy of ROME_merged (char)
startdir = pwd;  path0 = path;                % restored at exit
if bdIsLoaded(mdl), close_system(mdl, 0); end
mkdir(tmp);
copyfile(root, tmp);
copyfile(fullfile(here, 'stubs_hw', 'ROMECommand.m'), fullfile(tmp, 'ROMECommand.m'), 'f');
copyfile(fullfile(here, 'stubs_hw', 'RobotLinkRecorder.m'), fullfile(tmp, 'RobotLinkRecorder.m'), 'f');
copyfile(fullfile(here, 'stubs_hw', 'MotiveDataCounting.m'), fullfile(tmp, 'MotiveDataCounting.m'), 'f');
mdlcell = {mdl};
cleanup = onCleanup(@() cellfun(@(c) c(), {@() cd(startdir), @() path(path0), ...
          @() cellfun(@(m) close_system(m, 0), mdlcell(bdIsLoaded(mdl))), @() rmdir(tmp, 's')})); %#ok<NASGU>
addpath(tmp, fullfile(tmp, 'Ground Vehicle'), fullfile(tmp, 'Optitrack'), here);   % here: tests/dc_set_mode.m
cd(tmp);
evalin('base', 'clear all; clear classes');
txt = fileread(fullfile(tmp, 'ROMECommand.m'));
assert(isempty(regexp(txt, '(tcpclient|MulticastSocket|DatagramSocket|serialport|udpport)\s*\(', 'once')), 'test_motive_source:network', 'the copy''s ROMECommand.m has a network call');
for c = {'ROMECommand', 'MotiveDataCounting', 'motive_cal_check', 'motive_source_check', 'define_constants', 'apply_pacing'}
    assert(strcmp(fileparts(which(c{1})), tmp), 'test_motive_source:copy', '%s resolves to %s, not the copy %s', c{1}, which(c{1}), tmp);
end
if ~isempty(ib)
    for e = 1:numel(B{ib,3})
        ed = B{ib,3}{e};
        f = fullfile(tmp, ed{1});
        fid = fopen(f, 'r');  s = char(fread(fid, Inf, '*uint8').');  fclose(fid);
        n = numel(strfind(s, ed{2}));
        assert(n == 1, 'test_motive_source:brk', 'break edit in %s matches %d times (expected 1)', ed{1}, n);
        fid = fopen(f, 'w');  fwrite(fid, uint8(strrep(s, ed{2}, ed{3})));  fclose(fid);
        fprintf('  break edit %d applied to the copy: %s\n', e, ed{1});
    end
    clear motive_cal_check motive_source_check apply_pacing define_constants
    rehash;
end
% helper: define_constants run INSIDE a function workspace (written into the copy only)
fid = fopen(fullfile(tmp, 'f7_dc_fn.m'), 'w');
fprintf(fid, '%s\n', 'function out = f7_dc_fn()', ...
    '% test_motive_source helper (temporary copy only): define_constants in THIS function''s workspace', ...
    'evalc(''define_constants'');', ...
    'out = struct();', ...
    'for v = {''EnableHardware'', ''EnableMotive'', ''motive_source'', ''arm_freeze'', ''arm_freeze_user'', ''sensor_model''}', ...
    '    out.(v{1}) = eval(v{1});', ...
    'end', ...
    'end');
fclose(fid);
rehash;
dcf = fullfile(tmp, 'define_constants.m');    % the copy's define_constants.m (char)
fid = fopen(dcf, 'r');  dc0 = char(fread(fid, Inf, '*uint8').');  fclose(fid);   % after the break (char)
dc0 = dc_set_mode(dc0, 1);                    % F12: run_mode 1 (EH 0, EM 1, 'emulator'); count-asserted (char)
fails = {};                                   % one message per failed check (cell of char)
cal0 = struct('R', eye(3), 't', zeros(3, 1), 'scale', 1, 'measured', false);   % define_constants' default (UNMEASURED)
calM = struct('R', [0 -1 0; 1 0 0; 0 0 1], 't', [0.1; 0.2; 0], 'scale', 0.001, 'measured', true);   % a valid measured cal (test-local)
calB = calM;  calB.R = 2 * eye(3);            % malformed: not a rotation

% ---- (iv) narrowing, argument and base forms ----------------------------------------------------------------------
A = {'a1', 1, 1, 'optitrack', cal0, 'motive_cal_check:unmeasured'; 'a2', 1, 1, 'emulator', cal0, ''; 'a3', 1, 1, 'emulator', [], ''; ...
     'a4', 1, 0, 'optitrack', cal0, '';  'a5', 0, 1, 'optitrack', cal0, '';  'a6', 1, 1, 'optitrack', calM, ''; ...
     'a7', 1, 1, 'optitrack', calB, 'motive_cal_check:badcal';  'a8', 1, 1, 'optitrack', [], 'motive_cal_check:nocal'; ...
     'a9', NaN, 1, 'optitrack', cal0, 'motive_cal_check:badflag';  'a10', 'x', 1, 'optitrack', cal0, 'motive_cal_check:badflag'; ...
     'a11', 1, 1, 'foo', cal0, 'motive_source_check:invalid'};   % case | EH | EM | src | cal | expected id
okIv = true;
for k = 1:size(A, 1)
    id = '';
    try
        motive_cal_check(A{k,2}, A{k,3}, A{k,4}, A{k,5});
    catch ea
        id = ea.identifier;
    end
    ok = strcmp(id, A{k,6});
    fprintf('iv  %-4s args EH %s EM %d src %-9s cal %-10s: id ''%s'' (expected ''%s'') -> %s\n', A{k,1}, mat2str(A{k,2}), A{k,3}, A{k,4}, ...
            char(string(isstruct(A{k,5})) + "/" + string(isstruct(A{k,5}) && isequal(A{k,5}.measured, true))), id, A{k,6}, mat2str(ok));
    okIv = okIv && ok;
end
% base form: {case, base assignments (name, value pairs), expected id}
Bf = {'b1', {}, 'motive_cal_check:noflag'; ...
      'b2', {'EnableHardware', NaN, 'EnableMotive', 1}, 'motive_cal_check:badflag'; ...
      'b3', {'EnableHardware', 1, 'EnableMotive', 1, 'motive_cal', cal0}, 'motive_cal_check:nosource'; ...
      'b4', {'EnableHardware', 1, 'EnableMotive', 1, 'motive_source', 'emulator', 'motive_cal', cal0}, ''; ...
      'b5', {'EnableHardware', 1, 'EnableMotive', 1, 'motive_source', 'optitrack', 'motive_cal', cal0}, 'motive_cal_check:unmeasured'};
for k = 1:size(Bf, 1)
    evalin('base', 'clear all');
    for j = 1:2:numel(Bf{k,2}), assignin('base', Bf{k,2}{j}, Bf{k,2}{j+1}); end
    id = '';
    try
        motive_cal_check();
    catch eb
        id = eb.identifier;
    end
    ok = strcmp(id, Bf{k,3});
    fprintf('iv  %-4s base form: id ''%s'' (expected ''%s'') -> %s\n', Bf{k,1}, id, Bf{k,3}, mat2str(ok));
    okIv = okIv && ok;
end
if ~okIv, fails{end+1} = '(iv) refusal narrowing / branches of motive_cal_check'; end

% ---- (v) invalid motive_source ------------------------------------------------------------------------------------
V = {'v1', 'emulator', 1; 'v2', 'optitrack', 0; 'v3', "emulator", 1; 'v4', 'Emulator', NaN; 'v5', 'foo', NaN; 'v6', '', NaN; ...
     'v7', 1, NaN; 'v8', {'emulator'}, NaN; 'v9', ['emulator'; 'emulator'], NaN};   % case | value | expected output (NaN = refused)
okV = true;
for k = 1:size(V, 1)
    id = '';  y = NaN;
    try
        y = motive_source_check(V{k,2});
    catch ev
        id = ev.identifier;
    end
    if isnan(V{k,3}), ok = strcmp(id, 'motive_source_check:invalid'); else, ok = isempty(id) && isequal(y, V{k,3}); end
    fprintf('v   %-3s motive_source_check(%s): output %s, id ''%s'' -> %s\n', V{k,1}, strtrim(evalc('disp(V{k,2})')), mat2str(y), id, mat2str(ok));
    okV = okV && ok;
end
evalin('base', 'clear all');
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc_set_mode(dc0, 1, struct('motive_source', 'foo'))));  fclose(fid);   % F12: override line
clear define_constants;  rehash;
idD = '';  afD = [];                          % error id of define_constants with 'foo', arm_freeze_user after it
try
    evalin('base', 'evalc(''define_constants'');');
catch ed
    idD = ed.identifier;
end
try, afD = evalin('base', 'arm_freeze_user'); catch, afD = []; end
okVd = strcmp(idD, 'motive_source_check:invalid') && isscalar(afD) && isnan(afD);
fprintf('v   define_constants with motive_source = ''foo'' (base): id ''%s'', arm_freeze_user %s -> %s\n', idD, mat2str(afD), mat2str(okVd));
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc0));  fclose(fid);
clear define_constants;  rehash;
evalin('base', 'clear all');
evalc('load_system(fullfile(tmp, [mdl ''.slx'']))');   % PostLoadFcn = the copy's define_constants (file flags)
blkM = [mdl '/Enabled' newline 'Subsystem/MATLAB System'];   % the MotiveData block (SID 105)
sim0 = get_param(blkM, 'SimulateUsing');
set_param(blkM, 'System', 'MotiveDataCounting');   % never the real socket, even if a break lets a sim run
set_param(blkM, 'SimulateUsing', sim0);       % changing System resets it (test_hw_nocam, measured 19 Nov)
assert(strcmp(get_param(blkM, 'System'), 'MotiveDataCounting') && strcmp(get_param(blkM, 'SimulateUsing'), 'Interpreted execution'), ...
       'test_motive_source:stub', 'MotiveData block not switched to MotiveDataCounting');
assignin('base', 'motive_source', 'foo');
assignin('base', 'run_mode_test_override', {'motive_source'});   % F12 fix round 1: set by hand; the compile-time refusal is under test, not the banner's
msC = '';                                     % compile error text with 'foo' (char)
try
    evalc('sim(mdl, ''StopTime'', ''0.1'', ''SimulationMode'', ''normal'');');
catch ec
    msC = ec.message;
    for c1 = 1:numel(ec.cause)
        msC = [msC ' | ' ec.cause{c1}.message]; %#ok<AGROW>
        for c2 = 1:numel(ec.cause{c1}.cause), msC = [msC ' | ' ec.cause{c1}.cause{c2}.message]; end %#ok<AGROW>
    end
end
okVc = contains(msC, 'motive_source');
fprintf('v   sim with motive_source = ''foo'' in the workspace: refused %d, names motive_source %d -> %s\n     %s\n', ~isempty(msC), okVc, mat2str(okVc), msC(1:min(end, 300)));
if ~(okV && okVd && okVc), fails{end+1} = sprintf('(v) invalid motive_source refused (function %d, define_constants %d, compile %d)', okV, okVd, okVc); end

% ---- (pac) apply_pacing refusal site, model loaded ----------------------------------------------------------------
P = {'p1', 'optitrack', 'motive_cal_check:unmeasured'; 'p2', 'emulator', ''};
okP = true;
for k = 1:size(P, 1)
    assignin('base', 'EnableHardware', 1);  assignin('base', 'EnableMotive', 1);  assignin('base', 'sensor_model', 0);
    assignin('base', 'motive_source', P{k,2});  assignin('base', 'motive_cal', cal0);
    id = '';
    try
        apply_pacing(mdl);
    catch ep
        id = ep.identifier;
    end
    ok = strcmp(id, P{k,3});
    fprintf('pac %s apply_pacing(mdl), EnableHardware 1, EnableMotive 1, %s, unmeasured: id ''%s'' (expected ''%s'') -> %s\n', P{k,1}, P{k,2}, id, P{k,3}, mat2str(ok));
    okP = okP && ok;
end
assignin('base', 'motive_source', 'optitrack');
msS = '';  ranS = false;                      % sim refusal text, sim returned (logical)
try
    evalc('sim(mdl, ''StopTime'', ''0.1'', ''SimulationMode'', ''normal'');');
    ranS = true;
catch es
    msS = es.message;
    for c1 = 1:numel(es.cause)
        msS = [msS ' | ' es.cause{c1}.message]; %#ok<AGROW>
        for c2 = 1:numel(es.cause{c1}.cause), msS = [msS ' | ' es.cause{c1}.cause{c2}.message]; end %#ok<AGROW>
    end
end
okS = ~ranS && contains(msS, 'motive_cal');
fprintf('pac p3 sim(), EnableHardware 1, EnableMotive 1, optitrack, unmeasured: refused %d, names motive_cal %d -> %s\n     %s\n', ~ranS, contains(msS, 'motive_cal'), mat2str(okS), msS(1:min(end, 300)));
assignin('base', 'EnableHardware', 0);
try, apply_pacing(mdl); catch, end            % pacing back off before closing (model not saved anyway)
close_system(mdl, 0);
if ~(okP && okS), fails{end+1} = sprintf('(pac) apply_pacing refusal site (apply_pacing %d, sim %d)', okP, okS); end

% ---- (I1) define_constants in a function workspace -----------------------------------------------------------------
D = {'a', dc0, 'fresh', ''; ...
     's', dc0, 'stale', ''; ...
     'b', dc_set_mode(dc0, 4, struct('arm_freeze', 1)), 'fresh', 'motive_cal_check:unmeasured'; ...
     'c', dc_set_mode(dc0, 2), 'fresh', ''};   % case | file text | base | expected id (F12: b = EH 1, EM 1, 'optitrack', arm_freeze 1; c = mode 2, as before)
okI = true;
for k = 1:size(D, 1)
    fid = fopen(dcf, 'w');  fwrite(fid, uint8(D{k,2}));  fclose(fid);
    clear define_constants f7_dc_fn;  rehash;
    evalin('base', 'clear all');
    if strcmp(D{k,3}, 'stale')
        assignin('base', 'EnableHardware', 1);  assignin('base', 'EnableMotive', 1);
        assignin('base', 'motive_source', 'optitrack');  assignin('base', 'motive_cal', cal0);
    end
    assert(~bdIsLoaded(mdl), 'test_motive_source:I1', 'model loaded during the function-workspace runs');
    id = '';  o = struct();
    try
        o = f7_dc_fn();
    catch ei
        id = ei.identifier;
    end
    ok = strcmp(id, D{k,4});
    if isempty(id)
        ok = ok && isfield(o, 'arm_freeze_user') && isequal(o.arm_freeze_user, o.arm_freeze);
        fprintf('I1  %s %-5s base, function workspace: no error; EnableHardware %g EnableMotive %g %s, arm_freeze_user %g (arm_freeze %g) -> %s\n', ...
                D{k,1}, D{k,3}, o.EnableHardware, o.EnableMotive, o.motive_source, o.arm_freeze_user, o.arm_freeze, mat2str(ok));
    else
        fprintf('I1  %s %-5s base, function workspace: id ''%s'' (expected ''%s'') -> %s\n', D{k,1}, D{k,3}, id, D{k,4}, mat2str(ok));
    end
    okI = okI && ok;
end
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc0));  fclose(fid);
clear define_constants f7_dc_fn;  rehash;
% model LOADED (F7 fix round 1): define_constants then also calls apply_pacing, which must get the file's own values.
evalin('base', 'clear all');
evalc('load_system(fullfile(tmp, [mdl ''.slx'']))');   % PostLoadFcn = the copy's define_constants (base)
DL = {'La', 'fresh'; 'Ls', 'stale'};          % case | base workspace
for k = 1:size(DL, 1)
    evalin('base', 'clear all');              % the model stays loaded
    if strcmp(DL{k,2}, 'stale')
        assignin('base', 'EnableHardware', 1);  assignin('base', 'EnableMotive', 1);  assignin('base', 'sensor_model', 1);
        assignin('base', 'motive_source', 'optitrack');  assignin('base', 'motive_cal', cal0);
    end
    id = '';  o = struct();
    try
        o = f7_dc_fn();
    catch ei
        id = ei.identifier;
    end
    ok = isempty(id) && bdIsLoaded(mdl) && isfield(o, 'EnableHardware') && isequal(o.EnableHardware, 0);
    ehs = 'n/a';  if isfield(o, 'EnableHardware'), ehs = mat2str(o.EnableHardware); end   % EnableHardware the function saw (char)
    fprintf('I1  %s model LOADED, %-5s base, function workspace: id ''%s'' (expected none), EnableHardware %s -> %s\n', ...
            DL{k,1}, DL{k,2}, id, ehs, mat2str(ok));
    okI = okI && ok;
end
close_system(mdl, 0);
if ~okI, fails{end+1} = '(I1) define_constants in a function workspace (fresh / stale base, model loaded or not, refusal site)'; end

evalin('base', 'clear all');
if ~isempty(fails)
    tag = '';  if ~isempty(ib), tag = sprintf(' [break %s]', B{ib,1}); end
    error('test_motive_source:fail', '%d check(s) failed%s:\n  %s', numel(fails), tag, strjoin(fails, sprintf('\n  ')));
end
fprintf('\ntest_motive_source PASS: (iv) (v) (pac) (I1)\n');
end
