function varargout = test_lab_settings(brk)
%TEST_LAB_SETTINGS  The student block and the TA file lab_settings.m (focus plan F14a, 19 Nov 2026).
%   Runs on a TEMPORARY COPY of ROME_merged (tempname; the real folder is never written). Checks (all evaluated, then
%   one error test_lab_settings:fail listing the failed ones):
%     (a) student block: the text of define_constants.m above "Below here is the formulation" assigns exactly
%         scenario, run_mode, speed_factor, size_factor (in this order, once each), each line with a comment.
%     (b) one source: every name lab_settings.m assigns is assigned there exactly once, and the assignment lines of
%         those names anywhere else in the folder (every .m outside tests/) are exactly the six known READS (file and
%         text): define_constants.m's hardware override "sensor_model = 0;  % forced", motive_cal_check.m's two,
%         run_9dof_cases.m's max_rpm read, viewer/viewer_arm_map.m's two parses of lab_settings.m.
%     (c) unit comments: every assignment statement in lab_settings.m has a comment on one of its lines or on the
%         comment line right after it.
%     (d) the stale-file check covers BOTH files: define_constants run in the copy (mode 2, a robot mode), then
%         arm_freeze_banner('') passes (control); a comment appended to lab_settings.m makes it refuse with
%         arm_freeze_banner:stale AND print the RUN VERDICT FAIL line of the refused robot run (run_refused.m), with
%         rome_last_verdict.verdict FAIL and its refusal item FAIL; after define_constants again it passes; a comment
%         appended to define_constants.m makes it refuse again.
%     (e) park record rule covers both files: settings_sha changes when only lab_settings.m changes (define_constants.m's
%         own SHA-256 unchanged), and prerun_check_core refuses a frozen run whose park record carries the settings
%         SHA from before the lab_settings.m edit (a "park:" reason naming the SHA); with the record re-made after the
%         edit, no park reason.
%     (f) the TA file is the SIBLING of the define_constants that runs: with a decoy lab_settings.m (max_rpm = 1) first
%         on the MATLAB path and the current folder elsewhere, define_constants of the copy gives max_rpm = 80 (the
%         copy's value).
%     (g) the values lab_settings.m documents but cannot define (ESP32 192.168.4.1 port 3333, Motive rigid body 1,
%         239.255.42.99, 1511, COM4) are in its header and still in every site it names.
%     (h) collect_run_snapshot records the TA values (snap.lab.max_rpm = 80, every lab name), the 4 user names, and
%         dc_sha_file_now = the base dc_sha_at_run.
%   MAKE-IT-FAIL: test_lab_settings('<id>') applies one break to the COPY (each edit asserted to match once) and must
%   then FAIL at the named check. test_lab_settings('list') returns the table; tests/manual_makefail_f14a.m runs them.
%   Flat function without local functions (CLAUDE.md section 5, crash 2). Clears the base workspace.
if nargin < 1 || isempty(brk), brk = ''; end
here = fileparts(mfilename('fullpath'));  root = fileparts(here);
lf = char(10);
B = {
  'mf_sha_dconly', 'd e', {'settings_sha.m', 's = sprintf(''define_constants.m %s\nlab_settings.m %s\n'', dc_file_sha(dcFile), dc_file_sha(labFile));', ...
                                             's = sprintf(''define_constants.m %s\n'', dc_file_sha(dcFile));'}, 'the settings hash ignores lab_settings.m';
  'mf_bypath',     'f',   {'define_constants.m', 'run(lab_settings_file([mfilename(''fullpath'') ''.m'']));', 'lab_settings;'}, 'define_constants finds lab_settings on the MATLAB path';
  'mf_second',     'b',   {'define_constants.m', 'run_mode_cfg   = run_mode_table(run_mode);', ['max_rpm = 80;  % a second definition' lf 'run_mode_cfg   = run_mode_table(run_mode);']}, 'a second definition of max_rpm in define_constants';
  'mf_student5',   'a b', {'define_constants.m', ['size_factor  = 0.50;' ], ['max_rpm = 80;  % (rpm)' lf 'size_factor  = 0.50;']}, 'a fifth (lab) variable in the student block';
  'mf_nocomment',  'c',   {'lab_settings.m', 'max_rpm = 80;          % wheel limit (rev/min), applied by CommandGuard', 'max_rpm = 80;'}, 'max_rpm without its unit comment';
  'mf_norefline',  'd',   {'arm_freeze_banner.m', '    run_refused(err);                              % the PASS/FAIL line of a refused robot run (F14a)', '    % run_refused removed (make-it-fail)'}, 'a refused robot run prints no verdict line';
};
if strcmp(brk, 'list'), varargout{1} = B; return; end
ib = [];
if ~isempty(brk)
    ib = find(strcmp(B(:,1), brk));
    assert(numel(ib) == 1, 'test_lab_settings:brk', 'unknown break %s', brk);
    fprintf('MAKE-IT-FAIL break %s (check (%s) must FAIL): %s\n', brk, B{ib,2}, B{ib,4});
end
LABN = {'base_override', 'motive_cal', 'max_rpm', 'sensor_model', 'mocap_sigma', 'mocap_seed', 'enc_step_deg', 'enc_step_rad', ...
        'arm_sign', 'arm_offset_deg', 'arm_fw_switch', 'arm_fw_other', 'arm_fw_margin_deg', 'clearance_margin', ...
        'signoff_partF', 'signoff_partC', 'romecommand_sendlog'};   % the moved names, plus the two sign-off dates the TA enters (F2 wiring, 20 Nov) and the send log switch (20 Nov) (cellstr)
READS = {'define_constants.m', 'sensor_model = 0;  % forced: 0 exact state to the controller (flag)';
         'motive_cal_check.m', 'motive_cal = evalin(''base'', ''motive_cal'');         % camera frame calibration struct (see define_constants)';
         'motive_cal_check.m', 'motive_cal = [];                         % undefined: refused below if the source is ''optitrack''';
         'run_9dof_cases.m',   'max_rpm  = evalin(''base'',''max_rpm'');';
         ['viewer' filesep 'viewer_arm_map.m'], 'arm_sign       = grab(txt, ''arm_sign'');';
         ['viewer' filesep 'viewer_arm_map.m'], 'arm_offset_deg = grab(txt, ''arm_offset_deg'');'};   % file | trimmed line (the known reads)

% ---- temporary copy ----------------------------------------------------------------------------------------------
tmp = tempname;  dec = tempname;  elsewhere = tempname;   % copy, decoy folder, an empty current folder (char)
startdir = pwd;  path0 = path;
mkdir(tmp);  copyfile(root, tmp);  mkdir(dec);  mkdir(elsewhere);
cleanup = onCleanup(@() cellfun(@(c) c(), {@() cd(startdir), @() path(path0), @() evalin('base', 'clear all'), ...
          @() rmdir(tmp, 's'), @() rmdir(dec, 's'), @() rmdir(elsewhere, 's')})); %#ok<NASGU>
if ~isempty(ib)
    ed = B{ib,3};
    f = fullfile(tmp, ed{1});
    fid = fopen(f, 'r');  s = char(fread(fid, Inf, '*uint8').');  fclose(fid);
    n = numel(strfind(s, ed{2}));
    assert(n == 1, 'test_lab_settings:brk', 'break edit in %s matches %d times (expected 1)', ed{1}, n);
    fid = fopen(f, 'w');  fwrite(fid, uint8(strrep(s, ed{2}, ed{3})));  fclose(fid);
    fprintf('  break edit applied to the copy: %s\n', ed{1});
end
addpath(tmp, fullfile(tmp, 'Optitrack'), fullfile(tmp, 'Ground Vehicle'), fullfile(tmp, 'lab'), fullfile(tmp, 'tests'));   % tests: dc_set_mode
cd(tmp);  rehash;
for c = {'define_constants', 'lab_settings_file', 'settings_sha', 'arm_freeze_banner', 'run_refused', 'collect_run_snapshot'}
    assert(strcmp(fileparts(which(c{1})), tmp), 'test_lab_settings:path', '%s resolves to %s, not the copy', c{1}, which(c{1}));
end
dcf = fullfile(tmp, 'define_constants.m');  labf = fullfile(tmp, 'lab_settings.m');
dct = fileread(dcf);  labt = fileread(labf);
fails = {};

% ---- (a) student block ---------------------------------------------------------------------------------------------
iEnd = strfind(dct, 'Below here is the formulation');
blk = dct(1:iEnd(1));                              % student block text (char)
[toks, lines] = regexp(blk, '^\s*([A-Za-z]\w*)\s*=[^=][^\n]*', 'tokens', 'match', 'lineanchors'); %#ok<ASGLU>
nmA = cellfun(@(t) t{1}, toks, 'UniformOutput', false);
hasC = cellfun(@(l) ~isempty(regexp(l, '%\s*\S', 'once')), lines);
okA = isequal(nmA, {'scenario', 'run_mode', 'speed_factor', 'size_factor'}) && all(hasC);
fprintf('(a) student block assigns %s; every line commented %d -> %s\n', strjoin(nmA, ', '), all(hasC), mat2str(okA));
if ~okA, fails{end+1} = '(a) student block is not exactly scenario, run_mode, speed_factor, size_factor with comments'; end

% ---- (b) one source ------------------------------------------------------------------------------------------------
labNames = regexp(labt, '^\s*([A-Za-z]\w*)\s*=[^=]', 'tokens', 'lineanchors');
labNames = cellfun(@(t) t{1}, labNames, 'UniformOutput', false);
okB = isequal(sort(labNames), sort(LABN)) && numel(labNames) == numel(unique(labNames));
fprintf('(b) lab_settings.m assigns %d names (%d distinct; the 14 moved names: %d)\n', numel(labNames), numel(unique(labNames)), isequal(sort(unique(labNames)), sort(LABN)));
mf = dir(fullfile(tmp, '**', '*.m'));
mf = mf(~contains({mf.folder}, [tmp filesep 'tests']));
found = cell(0, 2);                                % file (relative) | trimmed line
pat = ['^\s*(' strjoin(LABN, '|') ')\s*=(?!=)[^\n]*'];
for k = 1:numel(mf)
    fp = fullfile(mf(k).folder, mf(k).name);
    rel = fp(numel(tmp) + 2:end);
    if strcmp(rel, 'lab_settings.m'), continue; end
    m = regexp(fileread(fp), pat, 'match', 'lineanchors');
    for j = 1:numel(m), found(end+1, :) = {rel, strtrim(m{j})}; end %#ok<AGROW>
end
key = @(C) sort(strcat(C(:,1), ' | ', C(:,2)));
okB = okB && isequal(key(found), key(READS));
fprintf('(b) assignments of the moved names outside lab_settings.m: %d (the %d known reads: %d)\n', size(found, 1), size(READS, 1), isequal(key(found), key(READS)));
extra = setdiff(key(found), key(READS));
for k = 1:numel(extra), fprintf('      not a known read: %s\n', extra{k}); end
if ~okB, fails{end+1} = '(b) a moved name is defined outside lab_settings.m, or twice in it'; end

% ---- (c) unit comments ---------------------------------------------------------------------------------------------
L = strsplit(labt, lf, 'CollapseDelimiters', false);   % keep blank lines (the default collapses them: a blank line would hide)
bad = {};
for i = 1:numel(L)
    t = regexp(L{i}, '^\s*([A-Za-z]\w*)\s*=[^=]', 'tokens', 'once');
    if isempty(t), continue; end
    j = i;                                         % last line of the statement (continuation with ...)
    while j < numel(L) && ~isempty(regexp(L{j}, '\.\.\.\s*(%.*)?$', 'once')), j = j + 1; end
    cm = any(cellfun(@(l) ~isempty(regexp(l, '%\s*\S', 'once')), L(i:j)));
    if ~cm && j < numel(L), cm = ~isempty(regexp(L{j + 1}, '^\s*%\s*\S', 'once')); end
    if ~cm, bad{end+1} = t{1}; end %#ok<AGROW>
end
okC = isempty(bad);
fprintf('(c) assignment statements without a comment: %s -> %s\n', strjoin([{''} bad], ' '), mat2str(okC));
if ~okC, fails{end+1} = sprintf('(c) no comment on %s', strjoin(bad, ', ')); end

% ---- (d) stale check covers both files, refusal line ------------------------------------------------------------------
evalin('base', 'clear all');
txt2 = dc_set_mode(dct, 2);                        % a robot mode, so a refusal prints its verdict line (char)
fid = fopen(dcf, 'w');  fwrite(fid, uint8(txt2));  fclose(fid);
clear define_constants lab_settings;  rehash;
evalc('evalin(''base'', ''define_constants'')');
e0 = '';  try, evalc('arm_freeze_banner('''')'); catch ex, e0 = ex.identifier; end
fid = fopen(labf, 'a');  fprintf(fid, '%% test_lab_settings (d): one comment line appended\n');  fclose(fid);
e1 = '';  t1 = '';
try, t1 = evalc('arm_freeze_banner('''')'); catch ex, e1 = ex.identifier; end
% evalc returns nothing on error: re-capture the printed refusal line with the error caught inside the captured text
t1 = evalc('try, arm_freeze_banner(''''); catch, end');
V1 = [];  try, V1 = evalin('base', 'rome_last_verdict'); catch, end
refLine = contains(t1, 'RUN VERDICT mode 2: FAIL') && isstruct(V1) && strcmp(V1.verdict, 'FAIL') && ...
          strcmp(V1.items(strcmp({V1.items.id}, 'refusal')).status, 'FAIL');
evalc('evalin(''base'', ''define_constants'')');
e2 = '';  try, evalc('arm_freeze_banner('''')'); catch ex, e2 = ex.identifier; end
fid = fopen(dcf, 'a');  fprintf(fid, '\n%% test_lab_settings (d): one comment line appended');  fclose(fid);
e3 = '';  try, evalc('arm_freeze_banner('''')'); catch ex, e3 = ex.identifier; end
okD = isempty(e0) && strcmp(e1, 'arm_freeze_banner:stale') && refLine && isempty(e2) && strcmp(e3, 'arm_freeze_banner:stale');
fprintf(['(d) mode 2 Run check: fresh ''%s'' (want none); lab_settings.m edited ''%s'' (want arm_freeze_banner:stale), verdict line ' ...
         'printed and stored %d; define_constants re-run ''%s'' (want none); define_constants.m edited ''%s'' (want stale) -> %s\n'], ...
        e0, e1, refLine, e2, e3, mat2str(okD));
if ~isempty(strtrim(t1)), fprintf('    printed at the refusal:\n%s\n', t1); end
if ~okD, fails{end+1} = '(d) the stale check does not cover lab_settings.m, or no verdict line at the refusal'; end
fid = fopen(dcf, 'w');  fwrite(fid, uint8(dct));  fclose(fid);   % the copy's files back
fid = fopen(labf, 'w');  fwrite(fid, uint8(labt));  fclose(fid);

% ---- (e) park record rule ------------------------------------------------------------------------------------------
shaA = settings_sha(dcf);  dcA = dc_file_sha(dcf);
fid = fopen(labf, 'a');  fprintf(fid, '%% test_lab_settings (e): lab_settings.m edited after the calibration\n');  fclose(fid);
shaB = settings_sha(dcf);  dcB = dc_file_sha(dcf);
fid = fopen(labf, 'w');  fwrite(fid, uint8(labt));  fclose(fid);
fz = [-2.16 51.422 104.825 0 -23.754 -1.32];       % a frozen command (firmware deg)
pf = fullfile(tempname, 'park.mat');  mkdir(fileparts(pf));
evalc('arm_park_record(''write'', fz, pf, shaA)');  recA = arm_park_record('read', pf);   % record made before the edit
pf2 = fullfile(fileparts(pf), 'park2.mat');
evalc('arm_park_record(''write'', fz, pf2, shaB)');  recB = arm_park_record('read', pf2);  % record made after it
rmdir(fileparts(pf), 's');
V = struct('EnableHardware', 1, 'EnableMotive', 1, 'motive_source', 'emulator', ...
           'motive_cal', struct('R', eye(3), 't', zeros(3, 1), 'scale', 1, 'measured', false), 'scenario', 1, ...
           'base_override', [NaN; NaN; NaN], 'speed_factor', 1.0, 'size_factor', 0.50, 'max_rpm', 80, 'sensor_model', 0, ...
           'mocap_sigma', [0.3e-3; 0.3e-3; deg2rad(0.05)], 'mocap_seed', [11 12 13], ...
           'enc_step_deg', 1 ./ [44.44444444, 55.55555556, 55.55555556, 46.66666667, 10, 20], 'arm_sign', [1 -1 -1 1 1 1], ...
           'arm_offset_deg', [0 180 90 0 0 0], 'arm_fw_switch', [-180 132 141 -165 90 180], 'arm_fw_other', [160 0 1 165 -90 -170], ...
           'arm_freeze_user', 1, 'arm_fw_margin_deg', 2.0, 'clearance_margin', 0.05);
S = struct('values', V, 'pacing', struct('EnablePacing', 'on', 'PacingRate', 1, 'FastRestart', 'off'), 'banner_error', '', ...
           'dc_sha_at_run', shaB, 'dc_sha_now', shaB, 'release', '2026a', 'signoff', struct('partF', '2026-11-20', 'partC', ''), ...
           'frozen_fw_deg', fz, 'park_record', recA);
[~, RA] = prerun_check_core(S);
S.park_record = recB;
[~, RB] = prerun_check_core(S);
parkA = RA(startsWith(RA, 'park:'));  parkB = RB(startsWith(RB, 'park:'));
okE = ~strcmp(shaA, shaB) && strcmp(dcA, dcB) && numel(parkA) == 1 && contains(parkA{1}, shaA) && contains(parkA{1}, shaB) && isempty(parkB);
fprintf('(e) settings SHA before/after a lab_settings.m edit differ %d; define_constants.m SHA unchanged %d; park reasons with the old record %d, with the new %d -> %s\n', ...
        ~strcmp(shaA, shaB), strcmp(dcA, dcB), numel(parkA), numel(parkB), mat2str(okE));
if ~isempty(parkA), fprintf('    %s\n', parkA{1}(1:min(end, 200))); end
if ~okE, fails{end+1} = '(e) the park-record SHA does not cover lab_settings.m'; end

% ---- (f) the sibling, never the path -------------------------------------------------------------------------------
decTxt = strrep(labt, 'max_rpm = 80;', 'max_rpm = 1;');
assert(~strcmp(decTxt, labt), 'test_lab_settings:decoy', 'decoy edit did not apply');
fid = fopen(fullfile(dec, 'lab_settings.m'), 'w');  fwrite(fid, uint8(decTxt));  fclose(fid);
addpath(dec, '-begin');  cd(elsewhere);  rehash;
assert(strcmp(fileparts(which('lab_settings')), dec) && strcmp(fileparts(which('define_constants')), tmp), 'test_lab_settings:decoy', ...
       'decoy not first (%s) or define_constants not the copy (%s)', which('lab_settings'), which('define_constants'));
evalin('base', 'clear all');  clear define_constants lab_settings;
evalc('evalin(''base'', ''define_constants'')');
mrF = evalin('base', 'max_rpm');
rmpath(dec);  cd(tmp);  rehash;
okF = isequal(mrF, 80);
fprintf('(f) decoy lab_settings.m (max_rpm 1) first on the path, current folder elsewhere: define_constants of the copy gives max_rpm %g (want 80) -> %s\n', mrF, mat2str(okF));
if ~okF, fails{end+1} = '(f) the TA file is found on the path, not next to define_constants.m'; end

% ---- (g) documented hardcoded sites ---------------------------------------------------------------------------------
hdr = labt(1:strfind(labt, '% HOW THE RUN IS MEANT TO WORK') - 1);
G = {'192.168.4.1', {'ROMECommand.m', 'calibrateROMEArm.m', 'jog_joint.m', 'jogROMEArm.m', 'returnHomeROMEArm.m', 'testGV.m', 'testLimitSwitches.m'};
     '3333',        {'ROMECommand.m', 'calibrateROMEArm.m', 'jog_joint.m', 'jogROMEArm.m', 'returnHomeROMEArm.m', 'testGV.m', 'testLimitSwitches.m'};
     'RigidBodyID = 1;', {['Optitrack' filesep 'MotiveData.m']};
     '239.255.42.99', {['Optitrack' filesep 'MotiveData.m']};
     'DataPort = 1511;', {['Optitrack' filesep 'MotiveData.m']};
     'COMPort = ''COM4''', {['Ground Vehicle' filesep 'GroundVehicleCommand.m']}};
docv = {'192.168.4.1', '3333', 'Streaming ID 1', '239.255.42.99', '1511', 'COM4'};   % how the header writes each value
okG = all(cellfun(@(v) contains(hdr, v), docv));
for k = 1:size(G, 1)
    for f = G{k,2}
        has = contains(fileread(fullfile(tmp, f{1})), G{k,1});
        if ~has, fprintf('(g) %s does not hold %s\n', f{1}, G{k,1}); end
        okG = okG && has;
    end
end
fprintf('(g) header documents %s; every named site holds its value -> %s\n', strjoin(docv, ', '), mat2str(okG));
if ~okG, fails{end+1} = '(g) a documented hardcoded value is missing from the header or its site'; end

% ---- (h) snapshot ----------------------------------------------------------------------------------------------------
evalin('base', 'clear all');  clear define_constants lab_settings;  rehash;
evalc('evalin(''base'', ''define_constants'')');
snap = collect_run_snapshot(dcf);
okH = isfield(snap, 'lab') && isfield(snap.lab, 'max_rpm') && isequal(snap.lab.max_rpm, 80) && ...
      isequal(sort(snap.lab_names(:)).', sort(LABN)) && isequal(snap.user_names(:).', {'scenario', 'run_mode', 'speed_factor', 'size_factor'}) && ...
      strcmp(snap.dc_sha_file_now, evalin('base', 'dc_sha_at_run')) && isequal(snap.max_rpm, 80);
fprintf('(h) snapshot: lab.max_rpm %s, %d lab names, user names %s, dc_sha_file_now = dc_sha_at_run %d -> %s\n', ...
        mat2str(snap.lab.max_rpm), numel(snap.lab_names), strjoin(snap.user_names, ', '), strcmp(snap.dc_sha_file_now, evalin('base', 'dc_sha_at_run')), mat2str(okH));
if ~okH, fails{end+1} = '(h) the run snapshot lacks the TA values or the settings hash'; end

if ~isempty(fails)
    tag = '';  if ~isempty(ib), tag = sprintf(' [break %s]', B{ib,1}); end
    error('test_lab_settings:fail', 'test_lab_settings FAILED: %s%s', strjoin(fails, '; '), tag);
end
fprintf('\ntest_lab_settings PASS: (a)-(h)\n');
end
