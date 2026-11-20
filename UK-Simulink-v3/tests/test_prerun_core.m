function test_prerun_core()
%TEST_PRERUN_CORE  Offline test of prerun_check_core.m, allowed_ranges.m and lab/arm_park_record.m (focus plan F2 + F11).
%   Pure: synthetic state structs, a temp .mat for the parked record; no model, no define_constants, no
%   base workspace, no network. Errors with id test_prerun_core:fail and a message 'FAIL: (<check>) ...' on the
%   first failing check; prints 'test_prerun_core PASS' at the end.
%   Checks:
%     (i)    two satisfied baselines -> ok, no reason; together they activate every item:
%            B  camera run with the arm moving (1,1,0), Bf EnableMotive 0 with the arm frozen (1,0,1)
%     (ii)   each item failing alone -> not ok, exactly ONE reason, naming that item (park and arm_freeze_user cases
%            on Bf)
%     (iii)  max_rpm = 150 -> the reason quotes the range (0, 120] and the source ROME_Mega_Code.ino:8
%     (iv)   mode 2 (emulator) with an unmeasured cal and no Part C -> ok; mode 3 with an unmeasured cal -> refused
%     (v)    frozen vs parked: 3 deg on J2 -> refused naming J2; 0.0004 deg -> ok; no record -> refused;
%            arm_freeze_user = 0 with no record -> ok (park item inactive)
%     (vi)   four items failing together -> four reasons, one per item (every failing reason, not the first)
%     (vii)  EnableHardware = 0 -> ok and skipped, whatever else is wrong
%     (viii) arm_park_record: history kept (two writes -> two entries, newest read), missing file -> [],
%            foreign file refused and left unchanged, bad input refused
%     (ix)   allowed_ranges: enforced rows are sourced, unsourced rows are not enforced; the firmware numbers it
%            cites match the lab repo files when they are on disk (else SKIP, printed)
%     (x)    a frozen arm with the camera pipeline on is ALLOWED (author 19 Nov, "Arm from model if frozen"; the
%            frozen_feedback item of Q-F7-a was removed): (1,1,1) optitrack and emulator (mode 2) ok with no reason and
%            no frozen_feedback item; (1,0,1) and (1,1,0) ok; (0,1,1) skipped; EnableMotive NaN with arm_freeze_user
%            NaN still refused (by the range item), never on frozen_feedback
root = fileparts(fileparts(mfilename('fullpath')));      % ROME_merged (or its temp mirror in the make-it-fail harness)
p0 = path;  cln = onCleanup(@() path(p0));
addpath(root, fullfile(root, 'lab'));
tmpd = tempname;  mkdir(tmpd);  cln2 = onCleanup(@() rmdir(tmpd, 's'));

sha  = repmat('ab', 1, 32);                              % fake define_constants SHA-256 (64 hex chars)
sha2 = repmat('cd', 1, 32);                              % a different one
fz   = [-3.0 53.0 100.0 0.0 -27.0 0.0];                  % 1x6 frozen command (firmware deg), synthetic
pfile = fullfile(tmpd, 'park.mat');
arm_park_record('write', fz, pfile, sha);
rec0 = arm_park_record('read', pfile);                   % parked record matching fz and sha

B  = base_state(sha, fz, rec0);                          % mode 3 (1,1,0): arm moving, everything satisfied
Bf = setf(setf(B, {'values', 'EnableMotive'}, 0), {'values', 'arm_freeze_user'}, 1);   % run 0 (1,0,1): cameras off, arm frozen
Bf.values = rmfield(Bf.values, 'motive_source');         % mode 1 file: no camera source read

% ---- (i) ----
% (i) checks the union over the two baselines (kept from the Q-F7-a version; a frozen camera run is allowed again
% since the author's decision of 19 Nov, and is covered by (x)).
[ok, R, info] = prerun_check_core(B);
chk('(i)', ok && isempty(R), 'baseline B (mode 3, arm moving) refused: %s', strjoin(R, ' | '));
[okf, Rf, infof] = prerun_check_core(Bf);
chk('(i)', okf && isempty(Rf), 'baseline Bf (run 0, EnableMotive 0, arm frozen) refused: %s', strjoin(Rf, ' | '));
want = {'ranges', 'pacing', 'sensor_model_hw', 'arm_freeze_banner', 'dc_sha', 'release', 'signoff_partF', ...
        'motive_source', 'motive_cal.measured', 'signoff_partC', 'park'};
items = union(info.items, infof.items);
chk('(i)', isempty(setdiff(want, items)), 'the baselines do not activate every item: missing %s', strjoin(setdiff(want, items), ', '));
fprintf('(i)    PASS  baselines B (mode 3, arm moving) and Bf (run 0, arm frozen) ok, %d items active together\n', numel(items));

% ---- (ii) each item alone ----
cases = {  % label, mutator, expected item token (applied to B unless the label is in onBf below)
  'pacing off',          @(S) setf(S, {'pacing', 'EnablePacing'}, 'off'),        'pacing';
  'pacing rate 2',         @(S) setf(S, {'pacing', 'PacingRate'}, 2),              'pacing';
  'fast restart on',       @(S) setf(S, {'pacing', 'FastRestart'}, 'on'),          'pacing';
  'sensor_model 1',        @(S) setf(S, {'values', 'sensor_model'}, 1),            'sensor_model_hw';
  'banner refused',        @(S) setf(S, {'banner_error'}, 'arm_freeze_banner:notwired: no ArmFreezeSwitch'), 'arm_freeze_banner';
  'banner not run',        @(S) rmfield(S, 'banner_error'),                        'arm_freeze_banner';
  'stale define_constants',@(S) setf(S, {'dc_sha_at_run'}, sha2),                  'dc_sha';
  'release 2025b',         @(S) setf(S, {'release'}, '2025b'),                     'release';
  'release garbage',       @(S) setf(S, {'release'}, 'R20x'),                      'release';
  'no Part F sign-off',    @(S) setf(S, {'signoff', 'partF'}, ''),                 'signoff_partF';
  'Part F bad date',       @(S) setf(S, {'signoff', 'partF'}, '2026-13-01'),       'signoff_partF';
  'motive_source invalid', @(S) setf(S, {'values', 'motive_source'}, 'camera'),    'motive_source';
  'cal unmeasured',        @(S) setf(S, {'values', 'motive_cal', 'measured'}, false), 'motive_cal.measured';
  'no Part C sign-off',    @(S) setf(S, {'signoff', 'partC'}, ''),                 'signoff_partC';
  'no park record',        @(S) setf(S, {'park_record'}, []),                      'park';
  'park record other SHA', @(S) setf(S, {'park_record', 'dc_sha'}, sha2),          'park';
  'scenario 6',            @(S) setf(S, {'values', 'scenario'}, 6),                'scenario';
  'EnableHardware 2',      @(S) setf(S, {'values', 'EnableHardware'}, 2),          'EnableHardware';
  'EnableMotive 2',        @(S) setf(S, {'values', 'EnableMotive'}, 2),            'EnableMotive';
  'speed_factor 0',        @(S) setf(S, {'values', 'speed_factor'}, 0),            'speed_factor';
  'size_factor -0.5',      @(S) setf(S, {'values', 'size_factor'}, -0.5),          'size_factor';
  'max_rpm 0',             @(S) setf(S, {'values', 'max_rpm'}, 0),                 'max_rpm';
  'max_rpm undefined',     @(S) setf(S, {'values'}, rmfield(S.values, 'max_rpm')), 'max_rpm';
  'max_rpm vector',        @(S) setf(S, {'values', 'max_rpm'}, [80 80]),           'max_rpm';
  'mocap_sigma negative',  @(S) setf(S, {'values', 'mocap_sigma'}, [-1e-3; 0; 0]), 'mocap_sigma';
  'enc_step_deg changed',  @(S) setf(S, {'values', 'enc_step_deg'}, S.values.enc_step_deg .* [1 1 1 1 1.01 1]), 'enc_step_deg';
  'arm_sign 2',            @(S) setf(S, {'values', 'arm_sign'}, [1 -1 -1 1 1 2]),  'arm_sign';
  'arm_fw_switch J2 133',  @(S) setf(S, {'values', 'arm_fw_switch'}, [-180 133 141 -165 90 180]), 'arm_fw_switch';
  'arm_fw_other J6 -160',  @(S) setf(S, {'values', 'arm_fw_other'}, [160 0 1 165 -90 -160]), 'arm_fw_other';
  'arm_freeze_user 0.5',   @(S) setf(S, {'values', 'arm_freeze_user'}, 0.5),       'arm_freeze';
  'arm_freeze_user NaN',   @(S) setf(S, {'values', 'arm_freeze_user'}, NaN),       'arm_freeze';
  'arm_fw_margin_deg 66',  @(S) setf(S, {'values', 'arm_fw_margin_deg'}, 66),      'arm_fw_margin_deg';
  'arm_fw_margin_deg -1',  @(S) setf(S, {'values', 'arm_fw_margin_deg'}, -1),      'arm_fw_margin_deg';
  'clearance_margin 0.04', @(S) setf(S, {'values', 'clearance_margin'}, 0.04),     'clearance_margin'};
% park and arm_freeze_user cases run on the frozen baseline Bf
onBf = {'no park record', 'park record other SHA', 'arm_freeze_user 0.5', 'arm_freeze_user NaN'};
for k = 1:size(cases, 1)
    if any(strcmp(cases{k,1}, onBf)), S = cases{k,2}(Bf); else, S = cases{k,2}(B); end
    [ok, R] = prerun_check_core(S);
    lab = sprintf('(ii) %s', cases{k,1});
    chk(lab, ~ok, 'not refused');
    chk(lab, numel(R) == 1, '%d reasons, want exactly 1: %s', numel(R), strjoin(R, ' | '));
    chk(lab, names_item(R{1}, cases{k,3}), 'reason does not name item %s: %s', cases{k,3}, R{1});
    chk(lab, contains(R{1}, 'allowed:') && contains(R{1}, 'source:'), 'reason lacks allowed/source: %s', R{1});
end
fprintf('(ii)   PASS  %d single-item failures, each refused with exactly its own reason\n', size(cases, 1));

% ---- (iii) max_rpm = 150: range and source quoted (literals here on purpose, not read from allowed_ranges) ----
[ok, R] = prerun_check_core(setf(B, {'values', 'max_rpm'}, 150));
chk('(iii)', ~ok && numel(R) == 1 && names_item(R{1}, 'max_rpm'), 'max_rpm 150 not refused alone: %s', strjoin(R, ' | '));
chk('(iii)', contains(R{1}, 'value 150'), 'reason does not quote the value: %s', R{1});
chk('(iii)', contains(R{1}, '(0, 120]'), 'reason does not quote the range (0, 120]: %s', R{1});
chk('(iii)', contains(R{1}, 'ROME_Mega_Code.ino:8'), 'reason does not quote the source ROME_Mega_Code.ino:8: %s', R{1});
[ok, R] = prerun_check_core(setf(B, {'values', 'max_rpm'}, 120));
chk('(iii)', ok && isempty(R), 'max_rpm 120 (the firmware cap itself) refused: %s', strjoin(R, ' | '));
[~, R] = prerun_check_core(setf(B, {'values', 'max_rpm'}, 150));
fprintf('(iii)  PASS  max_rpm 150 refused: %s\n', R{1});

% ---- (iv) emulator vs optitrack with an unmeasured cal ----
E = setf(B, {'values', 'motive_source'}, 'emulator');
E = setf(E, {'values', 'motive_cal', 'measured'}, false);
E = setf(E, {'signoff', 'partC'}, '');
[ok, R, info] = prerun_check_core(E);
chk('(iv)', ok && isempty(R), 'mode 2 (emulator) with unmeasured cal and no Part C refused: %s', strjoin(R, ' | '));
chk('(iv)', ~any(strcmp(info.items, 'motive_cal.measured')) && ~any(strcmp(info.items, 'signoff_partC')), ...
    'mode 2 activated the camera-calibration items');
O = setf(B, {'values', 'motive_cal', 'measured'}, false);
[ok, R] = prerun_check_core(O);
chk('(iv)', ~ok && numel(R) == 1 && names_item(R{1}, 'motive_cal.measured'), 'mode 3 (optitrack) with unmeasured cal not refused alone: %s', strjoin(R, ' | '));
chk('(iv)', contains(R{1}, 'false with motive_source ''optitrack'''), 'reason does not quote the value and source found: %s', R{1});
N = setf(B, {'values', 'EnableMotive'}, 0);              % mode 1: no cameras, motive_source absent
N.values = rmfield(N.values, 'motive_source');
N = setf(N, {'values', 'motive_cal', 'measured'}, false);
N = setf(N, {'signoff', 'partC'}, '');
[ok, R] = prerun_check_core(N);
chk('(iv)', ok && isempty(R), 'mode 1 (cameras off) refused for camera items: %s', strjoin(R, ' | '));
fprintf('(iv)   PASS  emulator ok with unmeasured cal; optitrack refused; cameras off ok\n');

% ---- (v) frozen vs parked (on Bf: a frozen arm is allowed only with EnableMotive = 0 since Q-F7-a) ----
[ok, R] = prerun_check_core(setf(Bf, {'frozen_fw_deg'}, fz + [0 3 0 0 0 0]));
chk('(v)', ~ok && numel(R) == 1 && names_item(R{1}, 'park') && contains(R{1}, 'J2 frozen 56.000 vs parked 53.000'), ...
    '3 deg on J2 not refused naming J2: %s', strjoin(R, ' | '));
chk('(v)', contains(R{1}, 'compares commanded postures only; it does not detect an arm moved by hand, a jog or a power cycle after calibration; re-calibrate after any of these'), ...
    'park reason does not state the physical-arm limit: %s', R{1});
% (the source text of the reason mentions "J2" itself, so the joints are matched as 'Jn frozen')
[ok, R] = prerun_check_core(setf(Bf, {'frozen_fw_deg'}, fz + [0 0 0 0 1 -2]));
chk('(v)', ~ok && numel(R) == 1 && contains(R{1}, 'J5 frozen') && contains(R{1}, 'J6 frozen') && ...
    ~any(arrayfun(@(j) contains(R{1}, sprintf('J%d frozen', j)), 1:4)), ...
    'J5 +1, J6 -2 deg not refused naming exactly J5 and J6: %s', strjoin(R, ' | '));
[ok, R] = prerun_check_core(setf(Bf, {'frozen_fw_deg'}, fz + [0 4e-4 0 0 0 0]));
chk('(v)', ok && isempty(R), '0.0004 deg on J2 (within tolerance) refused: %s', strjoin(R, ' | '));
[ok, R] = prerun_check_core(setf(Bf, {'park_record'}, []));
chk('(v)', ~ok && numel(R) == 1 && names_item(R{1}, 'park') && contains(R{1}, 'no parked record'), 'no record not refused: %s', strjoin(R, ' | '));
U = setf(Bf, {'values', 'arm_freeze_user'}, 0);
U = setf(U, {'park_record'}, []);
[ok, R] = prerun_check_core(U);
chk('(v)', ok && isempty(R), 'arm moving (arm_freeze_user 0) refused for a missing park record: %s', strjoin(R, ' | '));
fprintf('(v)    PASS  J2 +3 deg refused; +0.0004 deg ok; no record refused; unfrozen ignores the record\n');

% ---- (vi) every failing reason, not the first ----
M = setf(Bf, {'pacing', 'EnablePacing'}, 'off');        % on Bf: the park item needs a frozen arm
M = setf(M, {'values', 'max_rpm'}, 150);
M = setf(M, {'park_record'}, []);
M = setf(M, {'signoff', 'partF'}, '');
[ok, R] = prerun_check_core(M);
items = {'pacing', 'max_rpm', 'park', 'signoff_partF'};
chk('(vi)', ~ok && numel(R) == 4, '%d reasons for 4 failing items: %s', numel(R), strjoin(R, ' | '));
for k = 1:4
    chk('(vi)', any(cellfun(@(r) names_item(r, items{k}), R)), 'item %s missing from the reasons', items{k});
end
F = setf(O, {'values', 'motive_source'}, 'camera');      % invalid source + unmeasured cal: fail-safe, treated as optitrack
[ok, R] = prerun_check_core(F);
chk('(vi)', ~ok && numel(R) == 2 && names_item(R{1}, 'motive_source') && names_item(R{2}, 'motive_cal.measured') && ...
    contains(R{2}, 'motive_source ''camera'''), 'invalid source with unmeasured cal not refused on both items, quoting ''camera'': %s', strjoin(R, ' | '));
G = rmfield_path(setf(setf(B, {'values', 'motive_cal', 'measured'}, false), {'signoff', 'partC'}, ''), 'motive_source');   % EnableMotive 1, motive_source missing
[ok, R] = prerun_check_core(G);
chk('(vi)', ~ok && numel(R) == 3 && names_item(R{1}, 'motive_source') && names_item(R{2}, 'motive_cal.measured') && ...
    names_item(R{3}, 'signoff_partC'), 'missing motive_source + unmeasured cal + no Part C: want motive_source, motive_cal.measured, signoff_partC, got: %s', strjoin(R, ' | '));
fprintf('(vi)   PASS  4 failing items -> 4 reasons; invalid source + unmeasured cal -> 2; missing source + unmeasured cal + no Part C -> 3 (fail-safe)\n');

% ---- (vii) simulation only ----
Z = setf(M, {'values', 'EnableHardware'}, 0);
[ok, R, info] = prerun_check_core(Z);
chk('(vii)', ok && isempty(R) && info.skipped, 'EnableHardware 0 not skipped: %s', strjoin(R, ' | '));
fprintf('(vii)  PASS  EnableHardware 0 -> skipped\n');

% ---- (viii) arm_park_record ----
hfile = fullfile(tmpd, 'hist.mat');
chk('(viii)', isempty(arm_park_record('read', hfile)), 'missing file did not read as []');
arm_park_record('write', fz, hfile, sha);
r2 = arm_park_record('write', fz + 1, hfile, sha2);
r = arm_park_record('read', hfile);
chk('(viii)', r.history_count == 2 && r.n == 2 && isequal(r.fw_deg, fz + 1) && strcmp(r.dc_sha, sha2) && r2.n == 2, ...
    'history not kept / newest not returned: count %d, n %d', r.history_count, r.n);
L = load(hfile);
chk('(viii)', numel(L.park_history) == 2 && isequal(L.park_history(1).fw_deg, fz), 'first entry overwritten');
ffile = fullfile(tmpd, 'foreign.mat');  x = 1;  save(ffile, 'x');
b0 = fileread_bytes(ffile);                              % bytes of the foreign file before (uint8)
chk('(viii)', throws(@() arm_park_record('write', fz, ffile, sha), 'arm_park_record:foreign'), 'foreign file not refused on write');
chk('(viii)', throws(@() arm_park_record('read', ffile), 'arm_park_record:foreign'), 'foreign file not refused on read');
chk('(viii)', isequal(b0, fileread_bytes(ffile)), 'foreign file changed');
chk('(viii)', throws(@() arm_park_record('write', [1 2 3], pfile, sha), 'arm_park_record:input'), '3 joints accepted');
chk('(viii)', throws(@() arm_park_record('write', [fz(1:5) NaN], pfile, sha), 'arm_park_record:input'), 'NaN joint accepted');
chk('(viii)', throws(@() arm_park_record('write', fz, pfile, 'abc'), 'arm_park_record:input'), 'short SHA accepted');
chk('(viii)', throws(@() arm_park_record('write', fz, fullfile(tmpd, 'p.txt'), sha), 'arm_park_record:input'), 'non-.mat file accepted');
dd = dir(fullfile(tmpd, '*.mat'));  left = setdiff({dd.name}, {'park.mat', 'hist.mat', 'foreign.mat'});   % stray files, e.g. temp saves (cell)
chk('(viii)', isempty(left), 'temp files left behind: %s', strjoin(left, ', '));
fprintf('(viii) PASS  history kept, newest read, missing -> [], foreign file refused and unchanged, bad input refused\n');

% ---- (ix) allowed_ranges ----
[T, C] = allowed_ranges();
enf = ismember(T.kind, {'interval', 'set', 'equals'});
chk('(ix)', ~any(strcmp(T.status(enf), 'UNSOURCED')), 'an enforced row is UNSOURCED');
chk('(ix)', all(ismember(T.status(strcmp(T.kind, 'none')), {'UNSOURCED', 'NOT ENFORCED (author)'})), 'a row with no limit is not marked UNSOURCED / NOT ENFORCED (author)');
[~, ~, info] = prerun_check_core(B);
unc = regexprep(info.unchecked, ' \(.*$', '');           % names of the unenforced rows (cell of char)
wantU = {'base_override', 'speed_factor', 'size_factor', 'mocap_sigma', 'mocap_seed', 'arm_offset_deg'};
chk('(ix)', isempty(setxor(unc, wantU)), 'info.unchecked is {%s}, want {%s}', strjoin(unc, ', '), strjoin(wantU, ', '));
chk('(ix)', all(cellfun(@(s) ~isempty(strtrim(s)), T.source)), 'a row has no source text');
chk('(ix)', all(strcmp(T.ws_name, T.name) | strcmp(T.name, 'arm_freeze')), 'ws_name differs from name outside arm_freeze');
chk('(ix)', C.park_tol_deg > 0 && C.park_tol_deg < 3, 'park_tol_deg %g cannot separate 3 deg', C.park_tol_deg);
lab = fullfile(root, '..', '..', '..', '..', 'ucf-asrl', 'ROME', 'UK-Arduino');   % lab repo, read-only
mega = fullfile(lab, 'ROME_Mega_Code', 'ROME_Mega_Code.ino');
teen = fullfile(lab, 'ROME_Teensy_Code', 'ROME_Teensy_Code.ino');
if isfile(mega) && isfile(teen)
    ml = strsplit(fileread(mega), newline);
    chk('(ix)', ~isempty(regexp(ml{8}, '^#define MAX_RPM 120\s', 'once')), 'ROME_Mega_Code.ino:8 is not MAX_RPM 120: %s', ml{8});
    tl = strsplit(fileread(teen), newline);
    vec = @(s) cellfun(@str2double, regexp(char(extractBetween(s, '{', '}')), '-?[\d.]+', 'match'));
    chk('(ix)', isequal(vec(tl{29}), T.allowed{strcmp(T.name, 'arm_fw_switch')}{1}), 'Teensy L29 limits[] differ: %s', tl{29});
    chk('(ix)', isequal(vec(tl{30}), T.allowed{strcmp(T.name, 'arm_fw_other')}{1}), 'Teensy L30 otherLimits[] differ: %s', tl{30});
    chk('(ix)', max(abs(1 ./ vec(tl{26}) - T.allowed{strcmp(T.name, 'enc_step_deg')}{1})) < 1e-12, 'Teensy L26 stepsDeg differ: %s', tl{26});
    fprintf('(ix)   PASS  allowed_ranges sourced; firmware lines Mega:8, Teensy:26, :29, :30 match\n');
else
    fprintf('(ix)   PASS  allowed_ranges sourced; SKIP firmware line comparison (lab repo not at %s)\n', lab);
end

% ---- (x) frozen arm with the camera pipeline on: ALLOWED (author 19 Nov; frozen_feedback removed) ----
X = setf(B, {'values', 'arm_freeze_user'}, 1);           % (1,1,1), optitrack
[ok, R, info] = prerun_check_core(X);
chk('(x)', ok && isempty(R) && ~any(strcmp(info.items, 'frozen_feedback')), '(1,1,1) optitrack refused or frozen_feedback active: %s', strjoin(R, ' | '));
[ok, R, info] = prerun_check_core(setf(X, {'values', 'motive_source'}, 'emulator'));   % (1,1,1), emulator (mode 2)
chk('(x)', ok && isempty(R) && ~any(strcmp(info.items, 'frozen_feedback')), '(1,1,1) emulator (mode 2) refused or frozen_feedback active: %s', strjoin(R, ' | '));
[ok, R, info] = prerun_check_core(Bf);                   % (1,0,1) run 0
chk('(x)', ok && isempty(R) && ~any(strcmp(info.items, 'frozen_feedback')), '(1,0,1) run 0 refused or frozen_feedback active: %s', strjoin(R, ' | '));
[ok, R, info] = prerun_check_core(B);                    % (1,1,0) mode 3, arm moving
chk('(x)', ok && isempty(R) && ~any(strcmp(info.items, 'frozen_feedback')), '(1,1,0) refused or frozen_feedback active: %s', strjoin(R, ' | '));
[ok, R, info] = prerun_check_core(setf(X, {'values', 'EnableHardware'}, 0));   % (0,1,1) simulation
chk('(x)', ok && isempty(R) && info.skipped, '(0,1,1) simulation not skipped: %s', strjoin(R, ' | '));
[ok, R] = prerun_check_core(setf(setf(X, {'values', 'EnableMotive'}, NaN), {'values', 'arm_freeze_user'}, NaN));   % fail-safe readings
chk('(x)', ~ok && ~any(cellfun(@(r) names_item(r, 'frozen_feedback'), R)), 'EnableMotive NaN + arm_freeze_user NaN not refused, or refused on the removed frozen_feedback: %s', strjoin(R, ' | '));
fprintf('(x)    PASS  (1,1,1) allowed (optitrack and emulator); (1,0,1), (1,1,0) allowed; (0,1,1) skipped; NaN flags refused, not on frozen_feedback\n');
fprintf('test_prerun_core PASS\n');
end

% -----------------------------------------------------------------------------------------------------------------
function B = base_state(sha, fz, rec)
%BASE_STATE  Mode 4 (EnableHardware 1, EnableMotive 1, optitrack), arm moving (arm_freeze_user 0), every item
%            satisfied. The parked record matches fz, for the frozen cases.
V = struct();
V.EnableHardware = 1;  V.EnableMotive = 1;  V.motive_source = 'optitrack';
V.motive_cal = struct('R', eye(3), 't', zeros(3, 1), 'scale', 1, 'measured', true);
V.scenario = 1;  V.base_override = [NaN; NaN; NaN];  V.speed_factor = 1.0;  V.size_factor = 0.50;
V.max_rpm = 80;  V.sensor_model = 0;  V.mocap_sigma = [0.3e-3; 0.3e-3; deg2rad(0.05)];  V.mocap_seed = [11 12 13];
V.enc_step_deg = 1 ./ [44.44444444, 55.55555556, 55.55555556, 46.66666667, 10, 20];
V.arm_sign = [1 -1 -1 1 1 1];  V.arm_offset_deg = [0 180 90 0 0 0];
V.arm_fw_switch = [-180 132 141 -165 90 180];  V.arm_fw_other = [160 0 1 165 -90 -170];
V.arm_freeze_user = 0;  V.arm_fw_margin_deg = 2.0;  V.clearance_margin = 0.05;
B = struct('values', V, 'pacing', struct('EnablePacing', 'on', 'PacingRate', 1, 'FastRestart', 'off'), ...
           'banner_error', '', 'dc_sha_at_run', sha, 'dc_sha_now', sha, 'release', '2026a', ...
           'signoff', struct('partF', '2026-11-20', 'partC', '2026-11-21'), 'frozen_fw_deg', fz, 'park_record', rec);
end

function S = rmfield_path(S, name)
%RMFIELD_PATH  S with S.values.(name) removed.
S.values = rmfield(S.values, name);
end

function S = setf(S, path, v)
%SETF  S with the nested field path (cell of names) set to v.
S = setfield(S, path{:}, v);
end

function tf = names_item(r, item)
%NAMES_ITEM  true when reason r starts with the item token, followed by ':' or ' ('.
tf = startsWith(r, [item ':']) || startsWith(r, [item ' (']);
end

function chk(check, cond, fmt, varargin)
if ~cond
    error('test_prerun_core:fail', ['FAIL: %s ' fmt], check, varargin{:});
end
end

function tf = throws(f, id)
tf = false;
try, f(); catch err, tf = strcmp(err.identifier, id); end
end

function b = fileread_bytes(f)
fid = fopen(f, 'r');  b = fread(fid, Inf, '*uint8');  fclose(fid);
end
