function varargout = test_arm_freeze(brk)
%TEST_ARM_FREEZE  Upstream arm freeze (Plan 2 Task 5 test (iv), focus plan F1), offline: EnableHardware = 0.
%   Author, 19 Nov 2026: ROMECommand.m stays byte-identical to Keanu's version and the student's arm_freeze
%   (default 1) is applied upstream in the model, with a banner ARM FROZEN / ARM WILL MOVE.
%   Model under test (tools/wire_arm_freeze.m): ArmFreezeSwitch (u2 = Constant arm_freeze_user > 0.5) sends
%   Constant q0_9dof(4:9) or CommandGuard's joint command (model rad) to ROMECommand port 2; To Workspace
%   joint_sent_rad logs the switch output (what reaches ROMECommand), joint_guard_rad the CommandGuard output.
%
%   Runs a TEMPORARY COPY of ROME_merged (tempname), scenario 1, define_constants as in ROME_merged except the
%   arm_freeze line of run M. ROME_merged itself is never written. Three simulations:
%     F1  arm_freeze = 1, fresh load (PostLoadFcn define_constants).
%     F2  a second Run of the SAME loaded model with nothing re-run in between, the way a student presses Run
%         twice. ROMECommand.m L56 wrote arm_freeze = 0 into the base workspace during F1 (Keanu's override,
%         asserted below as the premise); this is the run in which a switch on arm_freeze would let the arm move
%         (docs/verification/2026-11-19_f1_eval_order_probe.log: a Constant is read at compile, before setupImpl).
%     M   arm_freeze = 0 in the copy's define_constants, fresh load.
%   Checks (all evaluated, then one error listing the failed ones):
%     (a) F1: joint_sent_rad equals q0_9dof(4:9) at every fixed step 0..T_stop (bit for bit), and through
%         ROMECommand's map and clamp (arm_sign, arm_offset_deg, firmware window minus arm_fw_margin_deg, limits
%         read from ROME_merged/define_constants.m) it equals arm_freeze_deg within 1e-9 deg; arm_freeze_deg equals
%         the same oracle; WHILE the controller's joint command (joint_guard_rad) moves >= 10 deg on some joint.
%     (b) F2: base arm_freeze was 0 at its start (premise: Keanu's override happened) and arm_freeze_user 1, and
%         (a)'s conditions hold for F2.
%     (c) M: joint_sent_rad equals joint_guard_rad at every step (bit for bit) and moves >= 10 deg; the freeze is
%         downstream only: q_cmd and joint_guard_rad of M equal those of F1 bit for bit.
%     (d) banner (model InitFcn, arm_freeze_banner): F1 and F2 print 'ARM FROZEN at arm_freeze_deg' once and no
%         'ARM WILL MOVE', and the six numbers it prints equal the oracle (map and clamp of q0_9dof(4:9)) within
%         half a unit of its %.3f print; M prints 'ARM WILL MOVE' once and no 'ARM FROZEN'. The banner refuses a
%         wrong flag or a wiring other than tools/wire_arm_freeze.m's (fix round 1), so most breaks of the switch
%         also patch the copy's banner refusal off ('if ~isempty(prob)' -> 'if false') to reach (a)-(c).
%     (e) ROME_merged/ROMECommand.m is Keanu's file of lab commit 63a10aa: its bytes are that blob with every LF
%         as CRLF (the lab repo checks out with core.autocrlf = true), i.e. the git blob SHA-1 of the file with
%         CRLF -> LF equals 05dcb7b42fbcb738942d2d16bee652bb7fca92d1 (git ls-tree 63a10aa
%         UK-Simulink-v3/ROMECommand.m in ucf-asrl/ROME), and no other CR is in it. When the lab repo is on disk,
%         the blob id is also read from it with git (cross-check; printed NOT RUN otherwise).
%     (f) a define_constants that stops partway (an error injected before its seeds, run after arm_freeze_user
%         was 1) leaves arm_freeze_user = NaN, and arm_freeze_banner then refuses with arm_freeze_banner:badflag
%         (fix round 1, M1; model closed, so only the flag is judged).
%     (g) G, after F2 with the model still loaded: the copy's define_constants.m is edited (arm_freeze 1 -> 0) and
%         NOT re-run; the Run is refused at InitFcn with cause arm_freeze_banner:stale (fix round 2, ruling R1).
%     (h) H: a syntax error is written into the user block and define_constants is typed (it fails before it
%         resets arm_freeze_user, which stays 1 from F1); the Run is refused with arm_freeze_banner:stale.
%     (i) after run M (fresh workspace, model closed), arm_freeze_banner on (EnableHardware, EnableMotive,
%         arm_freeze_user) = (1,1,1) (mode 2, arm frozen) prints ARM FROZEN and is NOT refused: the frozenfeedback
%         refusal of Q-F7-a was removed on the author's decision of 19 Nov ("Arm from model if frozen"; the controller
%         reads the model's arm angles when frozen, tools/wire_motive_emulator.m JointSource); (1,0,1) prints ARM
%         FROZEN and (1,1,0) prints ARM WILL MOVE, none refused.
%   MAKE-IT-FAIL: test_arm_freeze('<id>') applies one break of the table below to the COPY and must then error
%   naming its check; test_arm_freeze('list') returns the table. For a break whose fifth column names an
%   arm_freeze_banner refusal, the run F1 must also have been refused with exactly that cause, in the same run;
%   otherwise the test raises test_arm_freeze:wrongreason instead of test_arm_freeze:fail.
%   Flat function without local functions (CLAUDE.md section 5, crash 2). Clears the base workspace.
if nargin < 1 || isempty(brk), brk = ''; end
here = fileparts(mfilename('fullpath'));      % ROME_merged/tests (char)
root = fileparts(here);                       % ROME_merged (char)
mdl  = 'ROME_9DOF';                           % model name (char)
lf   = char(10);                              % line feed (char)
cr   = char(13);                              % carriage return (char)
tolDeg  = 1e-9;                               % freeze vs arm_freeze_deg after map and clamp (deg)
moveMin = 10;                                 % the controller's joint command must move at least this much (deg)
blobWant = '05dcb7b42fbcb738942d2d16bee652bb7fca92d1';   % git blob id of UK-Simulink-v3/ROMECommand.m at lab commit 63a10aa

% ---- break table: id | check that must FAIL | edits on the copy | what the break does | refusal id of run F1 ------
% edit: {'tool', mode} wire_arm_freeze on the copy's model; {'param', block ('' = the model), parameter, value} on the
% copy's model, saved; {'file', file in the copy, old, new} exactly one match; {'cmdbyte'} check (e) reads a copy of
% ROMECommand.m with a space added at the end of line 1.
BYP = {'file', 'arm_freeze_banner.m', 'if ~isempty(prob)', 'if false'};   % banner wiring refusal off (copy only)
CAP = 'arm_freeze_user = arm_freeze;';        % define_constants' capture line (char)
M1L = 'arm_freeze_user = NaN;  % arm freeze flag the model reads (flag): NaN until the LAST line of this file sets it, so a run after a partial define_constants is refused (arm_freeze_banner)';
B = { ...
  'mf_userflag',        'b', {{'param', 'ArmFreezeUser', 'Value', 'arm_freeze'}, BYP}, 'ArmFreezeUser reads arm_freeze instead of arm_freeze_user, banner refusal off: Keanu''s setup writes 0 there, so the second Run moves the arm', ''; ...
  'mf_userflag_banner', 'd', {{'param', 'ArmFreezeUser', 'Value', 'arm_freeze'}}, 'ArmFreezeUser reads arm_freeze, banner intact: the banner must refuse the run', 'arm_freeze_banner:notwired'; ...
  'mf_noswitch',        'a', {{'tool', 'bypass'}}, 'switch removed (wire_arm_freeze(copy, ''bypass'')): CommandGuard feeds ROMECommand directly; the banner refuses', 'arm_freeze_banner:notwired'; ...
  'mf_alwaysfrozen',    'c', {{'param', 'ArmFreezeSwitch', 'Threshold', '-0.5'}, BYP}, 'switch Threshold -0.5, banner refusal off: the arm is held even with arm_freeze = 0', ''; ...
  'mf_nobanner',        'd', {{'param', '', 'InitFcn', 'apply_pacing(bdroot)'}}, 'InitFcn back to apply_pacing(bdroot) only: no banner', ''; ...
  'mf_bannerval',       'd', {{'file', 'arm_freeze_banner.m', 'strtrim(sprintf(''%.3f '', fw))', 'strtrim(sprintf(''%.3f '', fw + 1))'}}, 'banner prints the frozen posture 1 deg off on every joint', ''; ...
  'mf_flag07',          'd', {{'file', 'define_constants.m', CAP, 'arm_freeze_user = 0.7;'}}, 'arm_freeze_user = 0.7 (switch u2 > 0.5 would freeze on a value nobody chose): the banner must refuse', 'arm_freeze_banner:badflag'; ...
  'mf_flagneg1',        'd', {{'file', 'define_constants.m', CAP, 'arm_freeze_user = -1;'}}, 'arm_freeze_user = -1: the banner must refuse', 'arm_freeze_banner:badflag'; ...
  'mf_noflag',          'd', {{'file', 'define_constants.m', CAP, 'clear arm_freeze_user'}}, 'arm_freeze_user undefined after define_constants: the banner must refuse', 'arm_freeze_banner:noflag'; ...
  'mf_noinvalidate',    'f', {{'file', 'define_constants.m', M1L, ''}}, 'define_constants no longer sets arm_freeze_user = NaN at the start of its formulation: a partial run keeps the stale 1', ''; ...
  'mf_romecmd',         'e', {{'cmdbyte'}}, 'check (e) reads a copy of ROMECommand.m with one byte changed (a space appended to line 1)', ''; ...
  'mf_noshacheck',      'g', {{'file', 'arm_freeze_banner.m', 'if ~shaOk', 'if false'}}, 'banner SHA check removed: a Run after define_constants.m was edited (or failed) proceeds on the stale workspace', ''};   % mf_nofrozenfb removed with the frozenfeedback refusal (author 19 Nov)
if strcmp(brk, 'list'), varargout{1} = B; return; end
ib = [];                                      % row of the break in B (empty: no break)
if ~isempty(brk)
    ib = find(strcmp(B(:,1), brk));
    assert(numel(ib) == 1, 'test_arm_freeze:brk', 'unknown break ''%s''; test_arm_freeze(''list'') gives the table', brk);
    fprintf('MAKE-IT-FAIL break %s (check (%s) must FAIL): %s\n', B{ib,1}, B{ib,2}, B{ib,4});
end

% ---- limits and map from the user's define_constants.m (the oracle, independent of the copy) ----------------------
dc0 = fileread(fullfile(root, 'define_constants.m'));   % ROME_merged/define_constants.m text (char)
L = struct();                                 % user-block values: arm_* (deg or sign)
for nm = {'arm_sign', 'arm_offset_deg', 'arm_fw_switch', 'arm_fw_other', 'arm_fw_margin_deg'}
    tok = regexp(dc0, ['(?m)^' nm{1} '\s*=\s*([^;%]+);'], 'tokens');
    assert(numel(tok) == 1, 'test_arm_freeze:dc', '%s: %d user-block lines in define_constants.m (expected 1)', nm{1}, numel(tok));
    L.(nm{1}) = eval(tok{1}{1});
end
fwLo = min(L.arm_fw_switch, L.arm_fw_other) + L.arm_fw_margin_deg;   % 1x6 lowest joint command sent (firmware deg)
fwHi = max(L.arm_fw_switch, L.arm_fw_other) - L.arm_fw_margin_deg;   % 1x6 highest joint command sent (firmware deg)
mapclamp = @(Xrad) min(max(L.arm_sign .* rad2deg(Xrad) + L.arm_offset_deg, fwLo), fwHi);   % Nx6 model rad -> firmware deg, as ROMECommand L98-113

% ---- temporary copy ----------------------------------------------------------------------------------------------
tmp = tempname;                               % temporary copy of ROME_merged (char)
startdir = pwd;  path0 = path;                % restored at exit
if bdIsLoaded(mdl), close_system(mdl, 0); end
mkdir(tmp);
copyfile(root, tmp);
mdlcell = {mdl};
cleanup = onCleanup(@() cellfun(@(c) c(), {@() cd(startdir), @() path(path0), ...
          @() cellfun(@(m) close_system(m, 0), mdlcell(bdIsLoaded(mdl))), @() rmdir(tmp, 's')})); %#ok<NASGU>
addpath(tmp, fullfile(tmp, 'Ground Vehicle'), fullfile(tmp, 'Optitrack'), here);   % here: tests/dc_set_mode.m
cd(tmp);
evalin('base', 'clear all; clear classes');
slx = fullfile(tmp, [mdl '.slx']);            % the copy's model file (char)
cmdf = fullfile(root, 'ROMECommand.m');       % the file check (e) reads (char)

% ---- the break, applied to the copy only ----------------------------------------------------------------------------
if ~isempty(ib)
    edits = B{ib,3};
    for e = 1:numel(edits)
        ed = edits{e};
        switch ed{1}
            case 'tool'
                addpath(fullfile(tmp, 'tools'));
                wire_arm_freeze(slx, ed{2});
            case 'cmdbyte'
                s = fileread(cmdf);
                k1 = find(s == cr, 1);        % end of line 1 (index)
                cmdf = fullfile(tmp, 'ROMECommand_tampered.m');
                fid = fopen(cmdf, 'w');  fwrite(fid, uint8([s(1:k1-1) ' ' s(k1:end)]));  fclose(fid);
            case 'param'
                evalc('load_system(slx)');
                blk = mdl;  if ~isempty(ed{2}), blk = [mdl '/' ed{2}]; end
                fprintf('  break: %s %s ''%s'' -> ''%s''\n', blk, ed{3}, get_param(blk, ed{3}), ed{4});
                set_param(blk, ed{3}, ed{4});
                save_system(mdl);
                close_system(mdl, 0);
            case 'file'
                f = fullfile(tmp, ed{2});     % file in the copy (char)
                fid = fopen(f, 'r');  s = char(fread(fid, Inf, '*uint8').');  fclose(fid);
                n = numel(strfind(s, ed{3}));
                assert(n == 1, 'test_arm_freeze:brk', 'break edit in %s matches %d times (expected 1)', ed{2}, n);
                fid = fopen(f, 'w');  fwrite(fid, uint8(strrep(s, ed{3}, ed{4})));  fclose(fid);
                fprintf('  break: %s: ''%s'' -> ''%s''\n', ed{2}, ed{3}(1:min(end, 60)), ed{4});
        end
    end
    evalin('base', 'clear all; clear classes');
    rehash;
end

% ---- (e) ROMECommand.m is Keanu's file ---------------------------------------------------------------------------
fid = fopen(cmdf, 'r');  raw = fread(fid, Inf, '*uint8').';  fclose(fid);   % bytes (uint8 row)
txtLF = strrep(char(raw), [cr lf], lf);       % CRLF -> LF (char)
crlfOnly = ~any(txtLF == cr) && isequal(char(raw), strrep(txtLF, lf, [cr lf]));   % every LF is CRLF, no other CR
md = java.security.MessageDigest.getInstance('SHA-1');
blob = lower(reshape(dec2hex(typecast(md.digest([uint8(sprintf('blob %d', numel(txtLF))) uint8(0) uint8(txtLF)]), 'uint8'), 2).', 1, []));   % git blob id (hex)
ucf = fileparts(fileparts(fileparts(fileparts(root))));   % .../UCF (char)
lab = fullfile(ucf, 'ucf-asrl', 'ROME');      % lab repo (char)
labTxt = 'NOT RUN (lab repo not on disk)';    % cross-check result (char)
if isfolder(fullfile(lab, '.git'))
    [st, o] = system(sprintf('git -C "%s" rev-parse 63a10aa:UK-Simulink-v3/ROMECommand.m', lab));
    if st == 0
        labTxt = sprintf('lab repo %s says %s -> %s', lab, strtrim(o), mat2str(strcmp(strtrim(o), blobWant)));
        if ~strcmp(strtrim(o), blobWant), crlfOnly = false; end
    else
        labTxt = sprintf('NOT RUN (git exit %d: %s)', st, strtrim(o));
    end
end
ce = crlfOnly && strcmp(blob, blobWant);
fprintf('(e) %s: %d bytes, CRLF-only %d, blob id of the LF form %s (want %s, lab 63a10aa); cross-check: %s -> %s\n', ...
        cmdf, numel(raw), crlfOnly, blob, blobWant, labTxt, mat2str(ce));

% ---- the five runs --------------------------------------------------------------------------------------------------
dcf = fullfile(tmp, 'define_constants.m');    % the copy's define_constants.m (char)
fid = fopen(dcf, 'r');  dcBase = char(fread(fid, Inf, '*uint8').');  fclose(fid);
% F12: the flags are derived from run_mode (dc_set_mode count-asserts the run_mode line and the override anchor).
dcF = dc_set_mode(dcBase, 1);                 % runs F1/F2/H: run_mode 1, arm_freeze 1 from the mode (char)
dcM = dc_set_mode(dcBase, 1, struct('arm_freeze', 0));   % runs M and G: run_mode 1 + arm_freeze 0 override (no student mode moves the arm in simulation) (char)
rows = @(d, N) reshape(permute(d, [find(size(d) == N, 1, 'last'), setdiff(1:ndims(d), find(size(d) == N, 1, 'last'))]), N, []);   % logged Data -> N x n
cfg = {'F1', 1, true, ''; 'F2', 1, false, ''; 'G', 1, false, 'edit'; 'H', 1, false, 'syntax'; 'M', 0, true, ''};   % run | arm_freeze in the copy's file | fresh load | file action before the Run
R = cell(1, size(cfg, 1));
for r = 1:size(cfg, 1)
    nm = cfg{r,1};
    dcErr = '';                               % error of a define_constants typed before this Run (char)
    switch cfg{r,4}
        case 'edit'                           % (g) the file is edited and NOT re-run
            fid = fopen(dcf, 'w');  fwrite(fid, uint8(dcM));  fclose(fid);
        case 'syntax'                         % (h) a user-block syntax error, then define_constants is typed
            nSc = numel(strfind(dcBase, [lf 'scenario = 1;']));
            assert(nSc == 1, 'test_arm_freeze:dc', 'copy define_constants.m: ''scenario = 1;'' x%d (expected 1)', nSc);
            fid = fopen(dcf, 'w');  fwrite(fid, uint8(strrep(dcBase, [lf 'scenario = 1;'], [lf 'scenario = 1;)'])));  fclose(fid);
            clear define_constants
            rehash;
            try, evalc('evalin(''base'', ''define_constants'')'); catch ex, dcErr = [ex.identifier ': ' ex.message]; end
    end
    if cfg{r,3}
        if bdIsLoaded(mdl), close_system(mdl, 0); end
        txt = dcF;
        if cfg{r,2} == 0, txt = dcM; end
        fid = fopen(dcf, 'w');  fwrite(fid, uint8(txt));  fclose(fid);
        evalin('base', 'clear all');
        clear define_constants
        rehash;
        evalc('load_system(slx)');            % PostLoadFcn = define_constants of the copy
    end
    S = struct('name', nm, 'dcErr', dcErr);
    S.afBefore  = evalin('base', 'arm_freeze');
    S.afuBefore = NaN;  try, S.afuBefore = evalin('base', 'arm_freeze_user'); catch, end
    S.err = '';  S.haveOut = false;  S.txt = '';  out = [];
    t0 = tic;
    try
        S.txt = evalc('out = sim(mdl, ''SimulationMode'', ''normal'', ''ReturnWorkspaceOutputs'', ''on'');');
        S.haveOut = true;
    catch ex
        S.err = [ex.identifier ': ' ex.message];
        for c1 = 1:numel(ex.cause)            % the InitFcn error carries the banner's refusal as its cause
            S.err = [S.err ' | ' ex.cause{c1}.identifier ': ' ex.cause{c1}.message];
        end
    end
    S.wall = toc(t0);
    S.afAfter = evalin('base', 'arm_freeze');
    for v = {'q0_9dof', 'arm_freeze_deg', 'T_stop', 'dt_9dof', 'arm_freeze_user'}
        try, S.(v{1}) = evalin('base', v{1}); catch, S.(v{1}) = []; end
    end
    S.t = zeros(0, 1);  S.Q = [];  S.JS = [];  S.JG = [];
    if S.haveOut
        S.t = out.q_cmd.Time;  N = numel(S.t);
        S.Q = rows(out.q_cmd.Data, N);  S.JS = rows(out.joint_sent_rad.Data, N);
        if any(strcmp(who(out), 'joint_guard_rad')), S.JG = rows(out.joint_guard_rad.Data, N); end
    end
    S.nFrozen = numel(strfind(S.txt, 'ARM FROZEN at arm_freeze_deg'));   % banner lines (count)
    S.nMove   = numel(strfind(S.txt, 'ARM WILL MOVE'));
    tk = regexp(S.txt, 'mapped and clamped = \[([^\]]*)\]', 'tokens', 'once');   % numbers the FROZEN banner printed
    S.banVals = [];  if ~isempty(tk), S.banVals = sscanf(tk{1}, '%f').'; end     % 1x6 (firmware deg) or empty
    bl = regexp(S.txt, '\*\*\* ARM [^\n]*', 'match');   % banner lines, for the log
    fprintf('run %-2s arm_freeze before %s (arm_freeze_user %s), after %s; %d steps to t = %.4f s (T_stop %.4f); wall %.2f s; sim error: %s\n      banner: %s\n', ...
            nm, mat2str(S.afBefore), mat2str(S.afuBefore), mat2str(S.afAfter), numel(S.t), max([S.t; NaN]), S.T_stop, S.wall, S.err, strjoin(bl, ' | '));
    R{r} = S;
end
close_system(mdl, 0);

% ---- (i) a frozen arm with arm feedback is refused (Q-F7-a, controller ruling 19 Nov) -----------------------------
% Base workspace: run M's fresh define_constants (SHA matches the copy's file); model closed, so the banner judges
% the flags only. Triples (EnableHardware, EnableMotive, arm_freeze_user).
TR = {'(1,1,1)', 1, 1, 1, '', 'ARM FROZEN at arm_freeze_deg', 2, {}; ...   % mode 2: allowed since the author's decision of 19 Nov
      '(1,0,1)', 1, 0, 1, '', 'ARM FROZEN at arm_freeze_deg', 2, {'EnableMotive'}; ...   % mode 2 with the EnableMotive 0 regression (TEST OVERRIDE)
      '(1,1,0)', 1, 1, 0, '', 'ARM WILL MOVE', 3, {}};   % triple | EnableHardware | EnableMotive | arm_freeze_user | refusal id wanted ('' none) | banner text wanted once | run_mode | run_mode_test_override (F12 fix round 1: the banner checks the flags against run_mode)
ruleTxt = 'For a base-only robot test use EnableMotive = 0 (run 0); for mode 2/3 set arm_freeze = 0';   % fragment of the ruling's reason (char)
eh0 = evalin('base', 'EnableHardware');  em0 = evalin('base', 'EnableMotive');  afu0 = evalin('base', 'arm_freeze_user');   % restored after (i)
rm0 = evalin('base', 'run_mode');  ov0 = evalin('base', 'run_mode_test_override');   % restored after (i)
ciK = false(1, size(TR, 1));                  % per triple result (logical)
for k = 1:size(TR, 1)
    assignin('base', 'EnableHardware', TR{k,2});  assignin('base', 'EnableMotive', TR{k,3});  assignin('base', 'arm_freeze_user', TR{k,4});
    assignin('base', 'run_mode', TR{k,7});  assignin('base', 'run_mode_test_override', TR{k,8});
    bid = '';  bmsg = '';  btxt = '';         % banner error id, message, printed text (char)
    try, btxt = evalc('arm_freeze_banner(mdl)'); catch ex, bid = ex.identifier;  bmsg = ex.message; end
    if isempty(TR{k,5})
        ciK(k) = isempty(bid) && numel(strfind(btxt, TR{k,6})) == 1;
    else
        ciK(k) = strcmp(bid, TR{k,5}) && contains(bmsg, ruleTxt) && contains(bmsg, '1657 deg');
    end
    fprintf('(i)  %s EnableHardware %d EnableMotive %d arm_freeze_user %d: error ''%s'' (want ''%s''), banner ''%s'' x%d -> %s\n      %s\n', ...
            TR{k,1}, TR{k,2}, TR{k,3}, TR{k,4}, bid, TR{k,5}, TR{k,6}, numel(strfind(btxt, TR{k,6})), mat2str(ciK(k)), strtrim([bmsg btxt]));
end
assignin('base', 'EnableHardware', eh0);  assignin('base', 'EnableMotive', em0);  assignin('base', 'arm_freeze_user', afu0);
assignin('base', 'run_mode', rm0);  assignin('base', 'run_mode_test_override', ov0);
ci = all(ciK);

% ---- (f) a partial define_constants leaves arm_freeze_user = NaN, which the banner refuses ------------------------
anchor = '% Distance Calculations (Projected onto the floor)';   % a line before scenario_seeds (char)
assert(numel(strfind(dcBase, anchor)) == 1, 'test_arm_freeze:dc', 'anchor for the partial run not found once');
fid = fopen(fullfile(tmp, 'dc_partial.m'), 'w');
fwrite(fid, uint8(strrep(dcBase, anchor, ['error(''test_arm_freeze:partial'', ''injected stop''); ' anchor])));  fclose(fid);
rehash;
evalin('base', 'clear all');
assignin('base', 'arm_freeze_user', 1);       % a stale 1 from an earlier complete run
partErr = '';  try, evalin('base', 'dc_partial'); catch ex, partErr = ex.identifier; end
afuP = 'undefined';  try, afuP = evalin('base', 'arm_freeze_user'); catch, end
banErr = '';  try, evalc('arm_freeze_banner(mdl)'); catch ex, banErr = ex.identifier; end
cfp = strcmp(partErr, 'test_arm_freeze:partial') && isnumeric(afuP) && isscalar(afuP) && isnan(afuP) && strcmp(banErr, 'arm_freeze_banner:badflag');
fprintf('(f)  partial define_constants: stopped by %s; arm_freeze_user after it %s (want NaN); banner %s (want arm_freeze_banner:badflag) -> %s\n', ...
        partErr, mat2str(afuP), banErr, mat2str(cfp));

% ---- checks -----------------------------------------------------------------------------------------------------------
F1 = R{1};  F2 = R{2};  G = R{3};  H = R{4};  M = R{5};
okRun = @(S) S.haveOut && numel(S.t) == floor(S.T_stop / S.dt_9dof + 1e-9) + 1 && S.t(end) > S.T_stop - S.dt_9dof && size(S.JS, 2) == 6;   % every fixed step 0..T_stop logged (the last one within dt_9dof of T_stop), 6 wide
mv = @(X) max(max(rad2deg(X), [], 1) - min(rad2deg(X), [], 1));   % largest range of any joint over the run (deg)
oracle = mapclamp(F1.q0_9dof(4:9).');         % 1x6 freeze posture through the map and clamp (firmware deg)
dOr = max(abs(F1.arm_freeze_deg(:).' - oracle));   % arm_freeze_deg vs the oracle (deg)
cf = false(1, 2);                             % (a), (b)
for r = 1:2
    S = R{r};
    hold_ = false;  dSent = NaN;  mvG = NaN;  mvS = NaN;
    if okRun(S)
        N = numel(S.t);
        hold_ = isequal(S.JS, repmat(S.q0_9dof(4:9).', N, 1));   % bit for bit, every step
        dSent = max(abs(mapclamp(S.JS) - S.arm_freeze_deg(:).'), [], 'all');
        mvS = mv(S.JS);
        if size(S.JG, 2) == 6, mvG = mv(S.JG); end
    end
    prem = true;  premTxt = '';
    if r == 2
        prem = isequal(S.afBefore, 0) && isequal(S.afuBefore, 1);
        premTxt = sprintf('premise: base arm_freeze %s at the start (Keanu''s override, want 0), arm_freeze_user %s (want 1) -> %d; ', mat2str(S.afBefore), mat2str(S.afuBefore), prem);
    end
    cf(r) = okRun(S) && hold_ && dSent <= tolDeg && dOr <= tolDeg && mvG >= moveMin && prem;
    fprintf('(%s)  run %s: %sran to T_stop %d; joint_sent_rad == q0_9dof(4:9) at all %d steps %d (range %.3g deg); |map+clamp(sent) - arm_freeze_deg| max %.3g deg, |arm_freeze_deg - oracle| %.3g deg (tol %g); controller joint command (joint_guard_rad) moved %.2f deg (need >= %g) -> %s\n', ...
            char('a' + r - 1), S.name, premTxt, okRun(S), numel(S.t), hold_, mvS, dSent, dOr, tolDeg, mvG, moveMin, mat2str(cf(r)));
end
fprintf('      arm_freeze_deg %s, oracle %s (firmware deg)\n', mat2str(F1.arm_freeze_deg(:).', 10), mat2str(oracle, 10));
% (c)
same = false;  mvM = NaN;  sameQ = false;  sameG = false;
if okRun(M) && size(M.JG, 2) == 6
    same = isequal(M.JS, M.JG);  mvM = mv(M.JS);
    sameQ = okRun(F1) && isequal(M.Q, F1.Q);
    sameG = okRun(F1) && isequal(M.JG, F1.JG);
end
cc = same && mvM >= moveMin && sameQ && sameG;
fprintf('(c)  run M: joint_sent_rad == joint_guard_rad at all %d steps %d, moved %.2f deg (need >= %g); q_cmd M == F1 %d, joint_guard_rad M == F1 %d -> %s\n', ...
        numel(M.t), same, mvM, moveMin, sameQ, sameG, mat2str(cc));
% (d)
dBan = Inf;                                   % max |banner number - oracle| over F1 and F2 (deg)
if numel(F1.banVals) == 6 && numel(F2.banVals) == 6
    dBan = max(abs([F1.banVals - oracle, F2.banVals - oracle]));
end
cd_ = F1.nFrozen == 1 && F1.nMove == 0 && F2.nFrozen == 1 && F2.nMove == 0 && M.nMove == 1 && M.nFrozen == 0 && dBan <= 5e-4 + 1e-9;
fprintf('(d)  banner counts [ARM FROZEN, ARM WILL MOVE]: F1 [%d %d] F2 [%d %d] (want [1 0]), M [%d %d] (want [0 1]); printed posture %s vs oracle, max diff %.3g deg (tol 5e-4) -> %s\n', ...
        F1.nFrozen, F1.nMove, F2.nFrozen, F2.nMove, M.nFrozen, M.nMove, mat2str(F1.banVals), dBan, mat2str(cd_));

% (g) and (h): stale workspace refused
cg = ~G.haveOut && contains(G.err, 'arm_freeze_banner:stale') && isequal(G.afuBefore, 1);
fprintf('(g)  run G (define_constants.m edited, not re-run; arm_freeze_user %s): sim returned %d; refusal: %s -> %s\n', ...
        mat2str(G.afuBefore), G.haveOut, G.err(1:min(end, 300)), mat2str(cg));
ch = ~isempty(H.dcErr) && isequal(H.afuBefore, 1) && ~H.haveOut && contains(H.err, 'arm_freeze_banner:stale');
fprintf('(h)  run H (user-block syntax error; define_constants failed: %s; arm_freeze_user left at %s): sim returned %d; refusal: %s -> %s\n', ...
        H.dcErr(1:min(end, 120)), mat2str(H.afuBefore), H.haveOut, H.err(1:min(end, 300)), mat2str(ch));

names = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i'};
pass  = [cf(1), cf(2), cc, cd_, ce, cfp, cg, ch, ci];
lab_  = {'FAIL', 'PASS'};
fprintf('\nsummary: %s\n', strjoin(cellfun(@(n, p) sprintf('(%s) %s', n, lab_{1 + p}), names, num2cell(pass), 'UniformOutput', false), ', '));
if ~isempty(ib) && ~isempty(B{ib,5})
    % a refusal break: run F1 must have been refused with exactly the expected banner cause
    got = regexp(F1.err, 'arm_freeze_banner:[a-z]+', 'match', 'once');   % the banner cause seen (char)
    fprintf('refusal of run F1: expected %s, seen %s\n', B{ib,5}, mat2str(got));
    if ~strcmp(got, B{ib,5})
        error('test_arm_freeze:wrongreason', 'break %s: run F1 refused with %s, expected %s', B{ib,1}, mat2str(got), B{ib,5});
    end
end
if ~all(pass)
    tag = '';
    if ~isempty(ib), tag = sprintf(' [break %s]', B{ib,1}); end
    error('test_arm_freeze:fail', 'test_arm_freeze FAILED checks: %s%s', strjoin(names(~pass), ', '), tag);
end
fprintf('\ntest_arm_freeze PASS: (a) (b) (c) (d) (e) (f) (g) (h) (i)\n');
end
