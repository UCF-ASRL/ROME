function varargout = test_prerun_wiring(brk)
%TEST_PRERUN_WIRING  F2/F11 wiring (20 Nov 2026): the model refuses a robot run at InitFcn with EVERY reason, and the
%   park record calibrateROMEArm writes is the one the check accepts. Runs on a TEMPORARY COPY of ROME_merged with the
%   no-network stubs (tests/stubs: ROMECommand, MotiveData); ROME_merged is never written. No robot run reaches its
%   first step: every sim() here is refused at InitFcn, or bounded to StopTime 0.1 s if a break lets it through.
%   Checks (all evaluated, then one error listing the failed ones):
%     (a) the real ROME_9DOF.slx InitFcn is 'apply_pacing(bdroot); arm_freeze_banner(bdroot); prerun_check(bdroot)'
%     (b) copy, mode 3, NO sign-off and NO park record: sim() is refused with error prerun_check:refused whose message
%         holds BOTH reasons (signoff_partF and park), and the Command Window shows both reason lines and the
%         RUN VERDICT FAIL line
%     (c) the park record written by calibrateROMEArm's OWN record block (its text between '% PARK RECORD' and the
%         function's last 'end', run with here = the copy, the start posture, every joint at target) is accepted:
%         prerun_check (returning form) ok in mode 3 (moving arm), then, calibrated again, in mode 2 (frozen)
%     (d) Mode 3 pressed after that mode-2 calibration (set_run_mode(3), as the button does) refuses the run with the
%         park reason of every robot mode (author: "We need to recalibrate no matter what"). Not covered: a file
%         whose text returns to exactly the calibrated text (Mode 4 then Mode 3) has the same SHA and is accepted
%     (e) mode 1 (simulation): prerun_check returns ok with nothing checked (info.skipped)
%     (f) a recalibration that starts invalidates the old record (wire3 review I5): back in mode 2, whose record from (c)
%         is valid (premise: ok), arm_park_record('invalidate') makes the run refused with a park reason; and the copy's
%         calibrateROMEArm.m calls arm_park_record('invalidate', ...) BEFORE it sends CAL_ARM (text order)
%     (g) the final line of calibrateROMEArm is truthful (clarity audit A2, fixes5 20 Nov 2026): the record block of (c)
%         printed "ROME arm calibration successfully completed." after "Park record written" in each of its two runs;
%         the same block run with a joint not at target (bad(2) true) and with no telemetry (lastRead NaN, bad
%         undefined, as calibrateROMEArm leaves it) prints "NO park record written" and the FAILED line ending
%         "Call the TA.", never the success line, and adds no entry to the park history; the success line occurs
%         exactly once in calibrateROMEArm.m (no unconditional copy elsewhere in the file)
%     (h) the limit-switch test voids the park record (hw audit row 33, fixes5): for run_mode 2, 3 and 4 in turn, a park
%         record of that mode's settings is written (premise: prerun_check gives no park reason), then the copy's
%         testLimitSwitches runs on the fake link (generated with ONLY tcpclient(...) -> RobotLinkTool(...), and the
%         uifigure / uilabel / uilamp / uialert calls -> tests/stubs_hw/gui_stub, whose figure closes after 1 s); the
%         link received exactly TEST_LIMITS then STOP_ALL and no ROME line; prerun_check then refuses with a park
%         reason. Mode 4 only: the copy's lab_settings.m gets motive_cal measured = true first (test-only lab fact, as
%         test_hw_nocam.m:108), else define_constants stops early in mode 4 and no park record can be valid.
%         NOT VERIFIED: the real window and socket (only the tool's code path on the fake link)
%   MAKE-IT-FAIL: test_prerun_wiring('<id>') applies one break to the COPY, then the named check must FAIL:
%     mf_sha       calibrateROMEArm.m record hashed with dc_file_sha (define_constants.m only, the pre-F14a hash)  -> (c)
%     mf_initfcn   the copy model's InitFcn without prerun_check (the F1 text)                                    -> (b)
%     mf_noinval   arm_park_record's 'invalidate' appends nothing (returns at once)                               -> (f)
%     mf_calsucc   calibrateROMEArm.m prints the success line unconditionally again ('if parkWritten' -> 'if true') -> (g)
%     mf_liminval  testLimitSwitches.m without its arm_park_record('invalidate', ...) call                        -> (h)
%   test_prerun_wiring('list') returns the break table.
%   Flat function without local functions (CLAUDE.md section 5, crash 2). Clears the base workspace.
B = {'mf_sha',     'c', {'file', 'calibrateROMEArm.m', 'parkFile, settings_sha(fullfile(here, ''define_constants.m'')));', 'parkFile, dc_file_sha(fullfile(here, ''define_constants.m'')));'}, 'record hashed without lab_settings.m';
     'mf_initfcn', 'b', {'param', 'InitFcn', 'apply_pacing(bdroot); arm_freeze_banner(bdroot)'}, 'InitFcn without prerun_check';
     'mf_noinval', 'f', {'file', fullfile('lab', 'arm_park_record.m'), 'if isempty(hist), rec = []; return; end    % no record: nothing to invalidate', 'rec = []; return;    % no record: nothing to invalidate'}, 'invalidate does nothing';
     'mf_calsucc', 'g', {'file', 'calibrateROMEArm.m', 'if parkWritten', 'if true'}, 'success line printed unconditionally';
     'mf_liminval', 'h', {'file', 'testLimitSwitches.m', 'addpath(fullfile(here, ''lab''));  arm_park_record(''invalidate'', fullfile(here, ''lab'', ''arm_park_record.mat''));', '% invalidate removed (break mf_liminval)'}, 'limit test leaves the park record valid'};
if nargin >= 1 && strcmp(brk, 'list'), varargout{1} = B; return; end
ib = [];
if nargin >= 1 && ~isempty(brk)
    ib = find(strcmp(B(:,1), brk));
    assert(numel(ib) == 1, 'test_prerun_wiring:brk', 'unknown break %s', brk);
    fprintf('MAKE-IT-FAIL break %s (check (%s) must FAIL): %s\n', brk, B{ib,2}, B{ib,4});
end
here = fileparts(mfilename('fullpath'));          % tests/ (char)
root = fileparts(here);                           % ROME_merged (char)
mdl = 'ROME_9DOF';
want = 'apply_pacing(bdroot); arm_freeze_banner(bdroot); prerun_check(bdroot)';   % InitFcn (char)
fails = {};
% ---- (a) the real model's InitFcn, read from the slx without loading callbacks into the run ------------------------
if bdIsLoaded(mdl), close_system(mdl, 0); end
evalc('load_system(fullfile(root, [mdl ''.slx'']))');
ifn = get_param(mdl, 'InitFcn');
close_system(mdl, 0);
okA = strcmp(ifn, want);
fprintf('(a) real model InitFcn ''%s'' -> %s\n', ifn, mat2str(okA));
if ~okA, fails{end+1} = '(a) InitFcn'; end
% ---- temporary copy -------------------------------------------------------------------------------------------------
tmp = tempname;  startdir = pwd;  path0 = path;
mkdir(tmp);  copyfile(root, tmp);
copyfile(fullfile(here, 'stubs', 'ROMECommand.m'), fullfile(tmp, 'ROMECommand.m'), 'f');
copyfile(fullfile(here, 'stubs', 'MotiveData.m'),  fullfile(tmp, 'Optitrack', 'MotiveData.m'), 'f');
if isfile(fullfile(tmp, 'lab', 'arm_park_record.mat')), delete(fullfile(tmp, 'lab', 'arm_park_record.mat')); end
mdlcell = {mdl};
cleanup = onCleanup(@() cellfun(@(c) c(), {@() cd(startdir), @() path(path0), ...
          @() cellfun(@(m) close_system(m, 0), mdlcell(bdIsLoaded(mdl))), @() rmdir(tmp, 's')})); %#ok<NASGU>
addpath(here);
addpath(tmp, fullfile(tmp, 'Ground Vehicle'), fullfile(tmp, 'Optitrack'));   % NOT lab/: the student path has no lab/ either
cd(tmp);  rehash;
wh = cellfun(@which, {'prerun_check', 'prerun_check_core', 'define_constants', 'ROMECommand'}, 'UniformOutput', false);
assert(all(startsWith(wh, tmp)), 'test_prerun_wiring:path', 'not resolved inside the copy: %s', strjoin(wh(~startsWith(wh, tmp)), ' | '));
slx = fullfile(tmp, [mdl '.slx']);  dcf = fullfile(tmp, 'define_constants.m');  parkFile = fullfile(tmp, 'lab', 'arm_park_record.mat');
fid = fopen(dcf, 'r');  dc0 = char(fread(fid, Inf, '*uint8').');  fclose(fid);
if ~isempty(ib) && strcmp(B{ib,3}{1}, 'file')
    f = fullfile(tmp, B{ib,3}{2});
    fid = fopen(f, 'r');  s = char(fread(fid, Inf, '*uint8').');  fclose(fid);
    n = numel(strfind(s, B{ib,3}{3}));
    assert(n == 1, 'test_prerun_wiring:brk', 'break edit matches %d times (expected 1)', n);
    fid = fopen(f, 'w');  fwrite(fid, uint8(strrep(s, B{ib,3}{3}, B{ib,3}{4})));  fclose(fid);
    rehash;
end
% ---- (b) mode 3, no sign-off, no park record: refused at InitFcn with both reasons -------------------------------------
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc_set_mode(dc0, 3)));  fclose(fid);
evalin('base', 'clear all');  clear define_constants lab_settings;  rehash;
evalc('load_system(slx)');
if ~isempty(ib) && strcmp(B{ib,3}{1}, 'param'), set_param(mdl, B{ib,3}{2}, B{ib,3}{3}); end
idB = '';  msgB = '';  txtB = '';
try
    txtB = evalc('sim(mdl, ''StopTime'', ''0.1'', ''SimulationMode'', ''normal'');');
catch ex
    ids = {ex.identifier};  msgs = {ex.message};  st = ex.cause(:).';
    while ~isempty(st), c = st{1};  st(1) = [];  ids{end+1} = c.identifier;  msgs{end+1} = c.message;  st = [st, c.cause(:).']; end %#ok<AGROW>
    k = find(strcmp(ids, 'prerun_check:refused'), 1);
    if ~isempty(k), idB = ids{k};  msgB = msgs{k}; end
    txtB = strjoin(msgs, ' | ');
end
close_system(mdl, 0);
okB = strcmp(idB, 'prerun_check:refused') && contains(msgB, 'signoff_partF:') && contains(msgB, 'park:') && contains(msgB, '2 reason(s)');
fprintf('(b) mode 3, no sign-off, no park record: id ''%s'', signoff_partF %d, park %d -> %s\n     %s\n', idB, ...
        contains(msgB, 'signoff_partF:'), contains(msgB, 'park:'), mat2str(okB), msgB(1:min(end, 600)));
if ~okB, fails{end+1} = '(b) InitFcn refusal with every reason'; end
% ---- (c) calibrateROMEArm's own record block, then the check accepts modes 2 and 3 ------------------------------------
prerun_fixture('signoff', tmp);
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc_set_mode(dc0, 3)));  fclose(fid);
evalin('base', 'clear all');  clear define_constants lab_settings;  rehash;
evalc('load_system(slx)');
ca = strrep(fileread(fullfile(tmp, 'calibrateROMEArm.m')), char([13 10]), newline);   % the file is CRLF (char)
i0 = strfind(ca, '% PARK RECORD');  i1 = strfind(ca, [newline newline 'end' newline newline '% Helper']);
assert(numel(i0) == 1 && numel(i1) == 1, 'test_prerun_wiring:calib', 'calibrateROMEArm record block not found once (%d, %d)', numel(i0), numel(i1));
blk = ca(i0:i1);                                  % the record block, verbatim (char)
here0 = here;  here = tmp; %#ok<NASGU>             % calibrateROMEArm's variable names
q0 = evalin('base', 'q0_9dof');
homePosDeg = evalin('base', 'arm_sign(:)') .* rad2deg(q0(4:9)) + evalin('base', 'arm_offset_deg(:)');
homePosDeg = arm_clamp(homePosDeg, evalin('base', 'arm_fw_switch'), evalin('base', 'arm_fw_other'), evalin('base', 'arm_fw_margin_deg'));
homePosDeg = homePosDeg(:);  lastRead = homePosDeg.';  bad = false(1, 6); %#ok<NASGU>
scr = fullfile(tempdir, 'test_prerun_wiring_blk.m');
fid = fopen(scr, 'w');  fwrite(fid, uint8(blk));  fclose(fid);
txtC = evalc('run(scr)');
delete(scr);  here = here0;
[ok3, r3] = prerun_check(mdl, '');
close_system(mdl, 0);
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc_set_mode(dc0, 2)));  fclose(fid);   % mode 2 needs a new calibration
evalin('base', 'clear all');  clear define_constants lab_settings;  rehash;
evalc('load_system(slx)');
here0 = here;  here = tmp; %#ok<NASGU>
fid = fopen(scr, 'w');  fwrite(fid, uint8(blk));  fclose(fid);
txtC = [txtC evalc('run(scr)')];
delete(scr);  here = here0;
[ok2, r2] = prerun_check(mdl, '');
okC =ok2 && ok3 && contains(txtC, 'Park record written');
fprintf('(c) calibrateROMEArm record block: %s; mode 2 ok %d (%s); mode 3 ok %d (%s) -> %s\n', strtrim(txtC), ok2, strjoin(r2, ' | '), ok3, strjoin(r3, ' | '), mat2str(okC));
if ~okC, fails{end+1} = '(c) calibrateROMEArm record accepted in modes 2 and 3'; end
% ---- (d) Mode button after the calibration: refused ------------------------------------------------------------------
close_system(mdl, 0);
evalc('set_run_mode(3, dcf)');                   % press Mode 3 after the mode-2 calibration
fid = fopen(dcf, 'r');  dcNow = char(fread(fid, Inf, '*uint8').');  fclose(fid);
evalin('base', 'clear all');  clear define_constants lab_settings;  rehash;
evalc('load_system(slx)');
[ok4, r4] = prerun_check(mdl, '');
close_system(mdl, 0);
sameText = strcmp(dcNow, dc_set_mode(dc0, 3));    % the file is mode 3 (logical)
okD = sameText && ~ok4 && any(startsWith(r4, 'park:')) && any(contains(r4, 'every robot mode'));
fprintf('(d) Mode 3 pressed after calibrating in mode 2 (file is mode 3: %d): ok %d, reasons: %s -> %s\n', sameText, ok4, strjoin(r4, ' | '), mat2str(okD));
if ~okD, fails{end+1} = '(d) Mode button after calibration refused'; end
% ---- (e) mode 1 skipped --------------------------------------------------------------------------------------------
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc_set_mode(dc0, 1)));  fclose(fid);
evalin('base', 'clear all');  clear define_constants lab_settings;  rehash;
evalc('load_system(slx)');
[ok1, r1, i1s] = prerun_check(mdl, '');
close_system(mdl, 0);
okE = ok1 && isempty(r1) && i1s.skipped;
fprintf('(e) mode 1: ok %d, %d reasons, skipped %d -> %s\n', ok1, numel(r1), i1s.skipped, mat2str(okE));
if ~okE, fails{end+1} = '(e) mode 1 skipped'; end
% ---- (f) a started recalibration voids the old record -----------------------------------------------------------------
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc_set_mode(dc0, 2)));  fclose(fid);   % mode 2, as calibrated in (c)
evalin('base', 'clear all');  clear define_constants lab_settings;  rehash;
evalc('load_system(slx)');
[okPre, rPre] = prerun_check(mdl, '');
arm_park_record('invalidate', parkFile);
[okInv, rInv] = prerun_check(mdl, '');
close_system(mdl, 0);
caT = fileread(fullfile(tmp, 'calibrateROMEArm.m'));
iInv = strfind(caT, 'arm_park_record(''invalidate''');  iCal = strfind(caT, 'write(c, "CAL_ARM"');
orderOk = numel(iInv) == 1 && numel(iCal) == 1 && iInv < iCal;
okF = okPre && ~okInv && any(startsWith(rInv, 'park:')) && orderOk;
fprintf('(f) mode 2 record valid before %d (%s); after invalidate ok %d, reasons: %s; calibrateROMEArm invalidates before CAL_ARM %d -> %s\n', ...
        okPre, strjoin(rPre, ' | '), okInv, strjoin(rInv, ' | '), orderOk, mat2str(okF));
if ~okF, fails{end+1} = '(f) a started recalibration invalidates the old record'; end
% ---- (g) the calibration's final line is truthful (clarity audit A2, fixes5) ------------------------------------------
SUCC = 'ROME arm calibration successfully completed.';   % the success line (char)
iP = strfind(txtC, 'Park record written');  iS = strfind(txtC, SUCC);   % (c) ran the block twice, both at target
okGs = numel(iP) == 2 && numel(iS) == 2 && all(iS > iP);
H0 = load(parkFile, 'park_history');  nH0 = numel(H0.park_history);   % park history entries before (count)
here0 = here;  here = tmp; %#ok<NASGU>
fid = fopen(scr, 'w');  fwrite(fid, uint8(blk));  fclose(fid);
lastRead = homePosDeg.';  bad = false(1, 6);  bad(2) = true; %#ok<NASGU>   % J2 did not reach the target
txtG1 = evalc('run(scr)');
lastRead = NaN(1, 6);  clear bad                   % no telemetry: calibrateROMEArm never defines bad then
txtG2 = evalc('run(scr)');
delete(scr);  here = here0;
H1 = load(parkFile, 'park_history');  nH1 = numel(H1.park_history);
okG1 = contains(txtG1, 'NO park record written') && contains(txtG1, 'ROME arm calibration FAILED') && contains(txtG1, 'Call the TA.') && ~contains(txtG1, SUCC);
okG2 = contains(txtG2, 'NO park record written') && contains(txtG2, 'ROME arm calibration FAILED') && contains(txtG2, 'Call the TA.') && ~contains(txtG2, SUCC);
nSucc = numel(strfind(fileread(fullfile(tmp, 'calibrateROMEArm.m')), SUCC));   % copies of the success line in the file
okG = okGs && okG1 && okG2 && nH1 == nH0 && nSucc == 1;
fprintf('(g) success line after the park record in both good runs %d; joint not at target: %s; no telemetry: %s; history %d -> %d; success line in the file x%d -> %s\n', ...
        okGs, strtrim(regexprep(txtG1, '\s+', ' ')), strtrim(regexprep(txtG2, '\s+', ' ')), nH0, nH1, nSucc, mat2str(okG));
if ~okG, fails{end+1} = '(g) calibrateROMEArm prints success only when the park record was written'; end
% ---- (h) testLimitSwitches voids the park record (hw audit row 33, fixes5) --------------------------------------------
ltf = fullfile(tmp, 'testLimitSwitches.m');
fid = fopen(ltf, 'r');  lt0 = char(fread(fid, Inf, '*uint8').');  fclose(fid);
OLDT = 'tcpclient(ESP32_IP, PORT, "ConnectTimeout", 5)';           % the one transport line replaced (char)
nT = numel(strfind(lt0, OLDT));
ltG = strrep(lt0, OLDT, 'RobotLinkTool(ESP32_IP, PORT, "ConnectTimeout", 5)');
for gname = {'uifigure', 'uilabel', 'uilamp', 'uialert'}
    ltG = regexprep(ltG, ['(?<![\w.])' gname{1} '\('], ['gui_stub(''' gname{1} ''', ']);
end
fid = fopen(ltf, 'w');  fwrite(fid, uint8(ltG));  fclose(fid);
for f = {'RobotLinkRecorder.m', 'RobotLinkTool.m', 'gui_stub.m'}
    copyfile(fullfile(here, 'stubs_hw', f{1}), fullfile(tmp, f{1}), 'f');
end
recL = fullfile(tmp, 'robot_link_record.txt');                      % the fake link's record (char)
rehash;
whL = cellfun(@which, {'testLimitSwitches', 'RobotLinkTool', 'gui_stub'}, 'UniformOutput', false);
okH = nT == 1 && all(startsWith(whL, tmp));
fprintf('(h) tcpclient line found %d time(s); resolved: %s\n', nT, strjoin(whL, ' | '));
labfH = fullfile(tmp, 'lab_settings.m');
fid = fopen(labfH, 'r');  lab0H = char(fread(fid, Inf, '*uint8').');  fclose(fid);
MEAS = '    ''measured'', false);';                              % motive_cal flag line (char)
for n = 2:4
    if n == 4
        % mode 4 (real cameras): define_constants stops early while motive_cal.measured is false, so no park record can
        % be valid. Test-only lab fact in the COPY, as test_hw_nocam.m:108 does: measured = true.
        assert(numel(strfind(lab0H, MEAS)) == 1, 'test_prerun_wiring:lab', 'motive_cal measured line found %d times', numel(strfind(lab0H, MEAS)));
        fid = fopen(labfH, 'w');  fwrite(fid, uint8(strrep(lab0H, MEAS, '    ''measured'', true);')));  fclose(fid);
    end
    fid = fopen(dcf, 'w');  fwrite(fid, uint8(dc_set_mode(dc0, n)));  fclose(fid);
    evalin('base', 'clear all');  clear define_constants lab_settings;  rehash;
    evalc('load_system(slx)');
    prerun_fixture('park', tmp);                                    % a valid record of this mode's settings
    [~, rA] = prerun_check(mdl, '');
    if isfile(recL), delete(recL); end
    errL = '';  txtL = '';
    try, txtL = evalc('testLimitSwitches()'); catch ex, errL = ex.message; end
    SL = read_link_record(recL);
    lnL = cellfun(@strtrim, SL.line(:).', 'UniformOutput', false);  % lines the fake robot received (cellstr)
    [~, rB] = prerun_check(mdl, '');
    close_system(mdl, 0);
    okN = isempty(errL) && ~any(startsWith(rA, 'park:')) && isequal(lnL, {'TEST_LIMITS', 'STOP_ALL'}) && any(startsWith(rB, 'park:'));
    fprintf('(h) mode %d: before, park reason %d (%s); tool error ''%s''; link received {%s}; after, park reason %d (%s) -> %s\n     tool printed: %s\n', ...
            n, any(startsWith(rA, 'park:')), strjoin(rA, ' | '), errL, strjoin(lnL, ', '), any(startsWith(rB, 'park:')), strjoin(rB, ' | '), mat2str(okN), ...
            strtrim(regexprep(txtL, '\s+', ' ')));
    okH = okH && okN;
end
clear RobotLinkTool RobotLinkRecorder gui_stub testLimitSwitches;   % release the copy's class files before rmdir
if ~okH, fails{end+1} = '(h) testLimitSwitches voids the park record (modes 2, 3, 4 refused with a park reason)'; end
evalin('base', 'clear all');
if ~isempty(fails)
    error('test_prerun_wiring:fail', 'test_prerun_wiring FAILED: %s', strjoin(fails, '; '));
end
fprintf('test_prerun_wiring PASS\n');
end
