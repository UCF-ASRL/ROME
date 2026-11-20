function test_bench(mut)
%TEST_BENCH  Offline tests of the Part F bench evaluators (Plan 3 Task 2).
%   Tests lab/bench_joint_signs.m, lab/bench_wheel_order.m and lab/bench_parse_telemetry.m on SYNTHETIC logs.
%   Pure functions only: no define_constants, no gate_all, no check_9dof, no test_suite, no model, no network.
%   The current arm_sign / arm_offset_deg are read from the TEXT of ../define_constants.m (regexp on the two
%   assignment lines, the file is not executed), so the "current constants" of case a1 are the live ones.
%   Cases
%     p1  telemetry parser accepts exactly what ROMECommand.m L149-168 accepts: GV needs >= 9 fields, ARM >= 8,
%         malformed and foreign lines are dropped, READY is read from field 8.
%     a1  jog log built with the current arm_sign / arm_offset_deg: every joint AGREE, no flag.
%     a2  same log, J2 observed inverted: J2 flagged, no other joint.
%     a3  telemetry that does not follow (reading moves 0 deg on J3): J3 flagged as NO FOLLOW only.
%     a4  J1 offset observed 15 deg off: J1 flagged, sign column still agrees.
%     m1-m4  map verdict: physical directions of J2/J5/J6 as rome_dirpin -> 'rome_dirpin'; as arcs_caldir ->
%         'arcs_caldir'; J2 arcs but J5 dirpin -> 'neither'; no observation -> 'inconclusive'. The J2 lean
%         at fw 132 (90 vs -39.6, checklist 24b) decides the map without any direction entry.
%     w1  wheel log, channel k = wheel k: all four (and the -15 repeat) pass.
%     w2  wheels 1 and 3 swapped: exactly the rows of wheel 1 and 3 are flagged.
%     w3  cross-talk (two channels turn), no telemetry, and an eye observation at the wrong corner are flagged.
%     w4  the predicted bottom direction of the wheel follows thesis Table 1 / CLAUDE.md 15 (alpha 45/135/225/315).
%     i1  INDEPENDENT of romefw_to_arcs_deg in its expected values: hard-coded ARCS J2 and lean at fw 51.4 and 100
%         (q1 = 0), the lean read from ar3_fk's DH chain (shoulder frame 1 to elbow frame 2) and, at fw 51.4, from the
%         recorded 9.4 / 41.0 of TRACKER Q-B2; the evaluator verdict is driven by those leans only.
%     i2  an observation consistent with BOTH maps (lean 25.2 at fw 67.2, where the predictions coincide) is
%         'inconclusive', not 'neither'.
%     i3  a NO FOLLOW row must not feed the map verdict.
%   NOTE m1/m2 are circular (the log generator and the evaluator both use romefw_to_arcs_deg); m5 and i1 are not.
%   test_bench(k), k = 1..10, is the MAKE-IT-FAIL hook: it breaks ONE input or expected value (see MUT below)
%   and the run must error with test_bench:fail. tests/manual_makefail_bench_joint.m runs all ten, each once.
%   test_bench() and test_bench(0) are the real test. The wheel-side make-it-fail is tests/manual_makefail_bench.m.
if nargin < 1, mut = 0; end   % 0 = real test; k > 0 = mutation k (see MUT comments)
here = fileparts(mfilename('fullpath'));                 % tests folder
root = fileparts(here);                                  % ROME_merged
addpath(fullfile(root, 'lab'), fullfile(root, 'analysis'));
K = read_constants(fullfile(root, 'define_constants.m')); % struct arm_sign, arm_offset_deg (1x6)

% ---------- p1 parser
lines = ["ARM,1.0,2.0,3.0,4.0,5.0,6.0,READY"; ...         % valid ARM, READY
         "ARM,1.0,2.0,3.0"; ...                            % too short: dropped
         "ARM,1.5,2.5,3.5,4.5,5.5,6.5,BUSY"; ...           % valid ARM, not READY
         "GV,1,2,3,4,5,6,7,8"; ...                         % valid GV
         "GV,1,2,3,4,5,6,7"; ...                           % 8 fields: dropped by the >= 9 rule
         "garbage"; "ARM_READY"];                          % foreign lines: dropped
[arm, rdy] = bench_parse_telemetry(lines, 'ARM');
chk(isequal(size(arm), [2 6]) && isequal(arm(1,:), 1:6) && isequal(arm(2,:), [1.5 2.5 3.5 4.5 5.5 6.5]), 'p1 ARM rows');
chk(isequal(rdy, [true; false]), 'p1 READY flags');
[gv, gvc] = bench_parse_telemetry(lines, 'GV');
chk(isequal(gv, [1 2 3 4]) && isequal(gvc, [5 6 7 8]), 'p1 GV row and cmd columns');
% MUT 1: expect the 8-field GV line to be accepted too (wrong): the >= 9 rule must make this FAIL
nGVexp = 1 + (mut == 1);                                   % expected number of accepted GV rows
chk(size(gv, 1) == nGVexp, 'p1 8-field GV line is dropped');

% ---------- a1 all agree
log = make_jog_log(K, 'rome_dirpin', K.arm_sign);
Kev = K;                                                   % constants given to the evaluator
if mut == 2, Kev.arm_sign(2) = -Kev.arm_sign(2); end       % MUT 2: evaluator constants differ in J2: a1 "no flag" must FAIL
[T, M] = bench_joint_signs(log, Kev);
chk(height(T) == 6, 'a1 six rows');
if mut == 6, chk(strcmp(M.verdict, 'arcs_caldir'), 'MUT6 dirpin log asserted to give arcs_caldir'); end % MUT 6: flipped expectation
chk(~any(T.flag), 'a1 no flag with the current arm_sign');
chk(all(T.status == "AGREE"), 'a1 all AGREE');
chk(isequal(T.implied_arm_sign.', K.arm_sign), 'a1 implied arm_sign equals the constants');
ok = T.joint ~= 4;                                        % J4 has no encoder; its tel columns are not judged
chk(all(T.meas_sign(ok) == 1), 'a1 measured telemetry sign +1 for a +5 deg jog');
chk(all(abs(T.implied_offset_deg - K.arm_offset_deg.') < 1e-9), 'a1 implied offset equals the constants');
chk(strcmp(M.verdict, 'rome_dirpin'), 'a1 map verdict from the dirpin-consistent log');

% ---------- a2 J2 inverted
phys = K.arm_sign;  phys(2) = -phys(2);
if mut == 3, phys = K.arm_sign; end                        % MUT 3: J2 NOT inverted, still asserted flagged: must FAIL
[T2, ~] = bench_joint_signs(make_jog_log(K, 'rome_dirpin', phys), K);
chk(isequal(find(T2.flag).', 2), 'a2 only J2 flagged');
chk(T2.status(2) == "DISAGREE" && T2.implied_arm_sign(2) == -K.arm_sign(2), 'a2 J2 DISAGREE with the opposite implied sign');

% ---------- a3 reading does not follow on J3
log3 = make_jog_log(K, 'rome_dirpin', K.arm_sign);
log3(3).after_lines = log3(3).before_lines;               % encoder reports no motion
if mut == 4, log3 = make_jog_log(K, 'rome_dirpin', K.arm_sign); end % MUT 4: J3 does follow, still asserted NO FOLLOW: must FAIL
[T3, ~] = bench_joint_signs(log3, K);
chk(isequal(find(T3.flag).', 3) && T3.status(3) == "NO FOLLOW", 'a3 only J3 flagged, NO FOLLOW');

% ---------- a4 offset 15 deg off on J1
log4 = make_jog_log(K, 'rome_dirpin', K.arm_sign);
log4(1).model_deg_obs = log4(1).model_deg_obs + (mut ~= 5) * 15 * K.arm_sign(1);   % observed model angle 15 deg off; MUT 5: 0 deg off, still asserted flagged: must FAIL
[T4, ~] = bench_joint_signs(log4, K);
chk(isequal(find(T4.flag).', 1) && T4.status(1) == "DISAGREE", 'a4 only J1 flagged');
chk(abs(T4.implied_offset_deg(1) - (K.arm_offset_deg(1) - 15)) < 1e-9, 'a4 implied offset is 15 deg away');

% ---------- m: map verdicts
physM = K.arm_sign;
[~, Mc] = bench_joint_signs(make_jog_log(K, 'arcs_caldir', physM), K);
chk(strcmp(Mc.verdict, 'arcs_caldir'), 'm2 arcs_caldir log gives arcs_caldir');
logM = make_jog_log(K, 'rome_dirpin', physM);
logM(2).phys_arcs_dir = -logM(2).phys_arcs_dir;           % J2 observed as arcs_caldir, J5/J6 as rome_dirpin
[~, Mn] = bench_joint_signs(logM, K);
chk(strcmp(Mn.verdict, 'neither'), 'm3 mixed observation gives neither');
logE = make_jog_log(K, 'rome_dirpin', physM);
for i = 1:numel(logE), logE(i).phys_arcs_dir = 0; logE(i).lean_deg_obs = NaN; end
[~, Me] = bench_joint_signs(logE, K);
chk(strcmp(Me.verdict, 'inconclusive'), 'm4 no observation gives inconclusive');
% lean at the switch end (checklist 24b): only J2, fw 132, no direction entry
q132 = [0 132 70 0 0 0];
lineJ2 = sprintf("ARM,%g,%g,%g,%g,%g,%g,READY", q132);
lineJ2a = sprintf("ARM,%g,%g,%g,%g,%g,%g,READY", [0 127 70 0 0 0]);   % after a -5 jog (followed)
logL = struct('j', 2, 'delta_deg', -5, 'before_lines', lineJ2, 'after_lines', lineJ2a, ...
              'phys_model_dir', 0, 'phys_arcs_dir', 0, 'model_deg_obs', NaN, 'lean_deg_obs', 90);
[~, Ml] = bench_joint_signs(logL, K);
chk(strcmp(Ml.verdict, 'rome_dirpin'), 'm5 lean 90 at fw 132 gives rome_dirpin');
if mut == 7, chk(strcmp(Ml.verdict, 'arcs_caldir'), 'MUT7 lean 90 asserted to give arcs_caldir'); end % MUT 7: flipped expectation
logL.lean_deg_obs = -39.6;
[~, Ml2] = bench_joint_signs(logL, K);
chk(strcmp(Ml2.verdict, 'arcs_caldir'), 'm5 lean -39.6 at fw 132 gives arcs_caldir');
% the predicted lean follows the checklist formulas lean = fw-42 / 92.4-fw
fw = 51.4;
chk(abs((romefw_to_arcs_deg([0 fw 0 0 0 0], 'rome_dirpin') + 90).' * [0;1;0;0;0;0] - (fw - 42)) < 1e-9, 'm6 dirpin lean = fw-42');
chk(abs((romefw_to_arcs_deg([0 fw 0 0 0 0], 'arcs_caldir') + 90).' * [0;1;0;0;0;0] - (92.4 - fw)) < 1e-9, 'm6 arcs lean = 92.4-fw');

% ---------- i1 independent expected values (hard-coded; ar3_fk DH chain, not romefw_to_arcs_deg)
% Hand derivation from ar3_fk's DH table with q1 = 0: frame-1 origin (shoulder) = (a1, 0, d1); frame 2 is
% Rz(q2) about frame 1's z (= +y0) and a2 = 0.305 m along x2 = (cos q2, 0, -sin q2), so the elbow is at
% shoulder + 0.305 (cos q2, 0, -sin q2) and lean from vertical toward +x0 = atan2(cos q2, -sin q2) = q2 + 90.
% Hard-coded: fw -> ARCS q2 (deg) and lean (deg): fw 51.4: dirpin -80.6 / 9.4, arcs_caldir -49.0 / 41.0 (TRACKER Q-B2);
% fw 100: dirpin -32 / 58, arcs_caldir -97.6 / -7.6.
IND = [51.4 -80.6   9.4  -49.0  41.0;     % fw, q2 dirpin, lean dirpin, q2 arcs, lean arcs
       100  -32.0  58.0  -97.6  -7.6];
for r = 1:2
    chk(abs(lean_of(IND(r,2)) - IND(r,3)) < 1e-9 && abs(lean_of(IND(r,4)) - IND(r,5)) < 1e-9, ...
        sprintf('i1 ar3_fk DH lean equals the hand values at fw %g', IND(r,1)));
    for m = 1:2                                            % m = 1 dirpin, 2 arcs_caldir
        leanObs = lean_of(IND(r, 2*m));                    % what the inclinometer would read under map m
        verd = {'rome_dirpin', 'arcs_caldir'};
        ln = sprintf("ARM,0,%g,70,0,0,0,READY", IND(r,1));
        lna = sprintf("ARM,0,%g,70,0,0,0,READY", IND(r,1) + 5);   % after the +5 jog (followed)
        e1 = struct('j', 2, 'delta_deg', 5, 'before_lines', ln, 'after_lines', lna, 'phys_model_dir', 0, ...
                    'phys_arcs_dir', 0, 'model_deg_obs', NaN, 'lean_deg_obs', leanObs);
        [~, Mi] = bench_joint_signs(e1, K);
        want = verd{m};
        if mut == 8 && r == 1 && m == 1, want = 'arcs_caldir'; end   % MUT 8: flipped expectation
        chk(strcmp(Mi.verdict, want), sprintf('i1 fw %g lean %.1f gives %s', IND(r,1), leanObs, want));
    end
end
% ---------- i2 consistent with both maps -> inconclusive (lean predictions coincide at fw 67.2: 25.2 both)
ln = sprintf("ARM,0,%g,70,0,0,0,READY", 67.2);
e2 = struct('j', 2, 'delta_deg', 5, 'before_lines', ln, 'after_lines', sprintf("ARM,0,%g,70,0,0,0,READY", 72.2), 'phys_model_dir', 0, ...
            'phys_arcs_dir', 0, 'model_deg_obs', NaN, 'lean_deg_obs', 25.2);
[~, Mb] = bench_joint_signs(e2, K);
want = 'inconclusive';  if mut == 9, want = 'rome_dirpin'; end   % MUT 9: wrong expectation
chk(strcmp(Mb.verdict, want), 'i2 observation consistent with both maps is inconclusive');
% ---------- i3 NO FOLLOW row does not feed the verdict
lnB = sprintf("ARM,0,%g,70,0,0,0,READY", 51.4);
e3 = struct('j', 2, 'delta_deg', 5, 'before_lines', lnB, 'after_lines', lnB, 'phys_model_dir', 0, ...
            'phys_arcs_dir', -1, 'model_deg_obs', NaN, 'lean_deg_obs', NaN);   % reading did not follow; arcs-like direction
[T3b, M3b] = bench_joint_signs(e3, K);
want = 'inconclusive';  if mut == 10, want = 'arcs_caldir'; end % MUT 10: wrong expectation
chk(T3b.status == "NO FOLLOW" && strcmp(M3b.verdict, want), 'i3 NO FOLLOW row gives no map verdict');

% ---------- w: wheel order
wl = make_wheel_log([1 2 3 4]);
[W, ~] = bench_wheel_order(wl);
chk(height(W) == 5 && ~any(W.flag), 'w1 identity channel map passes all five rows');
chk(isequal(W.tel_channel.', [1 2 3 4 1]), 'w1 active channel per row');
wl2 = make_wheel_log([3 2 1 4]);                           % wheels 1 and 3 swapped
W2 = bench_wheel_order(wl2);
chk(isequal(W2.k_cmd(W2.flag).', [1 3 1]), 'w2 exactly the rows of wheels 1 and 3 (and the -15 repeat of 1) flagged');
chk(isequal(W2.tel_channel(1:3).', [3 2 1]), 'w2 wheel 1 shows on channel 3, wheel 3 on channel 1');
% w3a cross-talk
wl3 = make_wheel_log([1 2 3 4]);
wl3(2).gv_lines = make_gv(2, 15, 15, [1 2]);               % channels 1 and 2 both turn for k = 2
W3 = bench_wheel_order(wl3);
chk(isequal(find(W3.flag).', 2), 'w3 cross-talk flagged on row 2 only');
% w3b no telemetry
wl4 = make_wheel_log([1 2 3 4]);  wl4(4).gv_lines = ["garbage"; "GV,1,2"];
W4 = bench_wheel_order(wl4);
chk(isequal(find(W4.flag).', 4) && W4.status(4) == "NO TELEMETRY", 'w3 no GV row flagged NO TELEMETRY');
% w3c eye observation wrong corner, and right
wl5 = make_wheel_log([1 2 3 4]);
for i = 1:5, wl5(i).eye_wheel = wl5(i).k_cmd; wl5(i).eye_bottom_xy = [NaN NaN]; end
W5 = bench_wheel_order(wl5);
chk(~any(W5.flag), 'w3 eye wheel agrees: no flag');
wl5(3).eye_wheel = 4;                                      % eye saw wheel 4 turn when wheel 3 was commanded
W5b = bench_wheel_order(wl5);
chk(isequal(find(W5b.flag).', 3), 'w3 eye wheel at the wrong corner flagged');
% w4 predicted bottom direction, ascending alpha
alphas = [45 135 225 315];
pred = [sind(alphas); -cosd(alphas)].';                    % bottom of wheel k for +rpm (checklist 26 table)
wl6 = make_wheel_log([1 2 3 4]);
for i = 1:5, wl6(i).eye_wheel = wl6(i).k_cmd;
             wl6(i).eye_bottom_xy = sign(wl6(i).rpm_cmd) * pred(wl6(i).k_cmd, :); end
W6 = bench_wheel_order(wl6);
chk(~any(W6.flag) && all(W6.eye_bottom_ok), 'w4 bottom direction as predicted for +15 and -15 passes');
wl6(1).eye_bottom_xy = -wl6(1).eye_bottom_xy;
W6b = bench_wheel_order(wl6);
chk(isequal(find(W6b.flag).', 1), 'w4 reversed bottom direction flagged on row 1 only');
fprintf('test_bench: PASS\n');
end

% ----------------------------------------------------------------------------------------------
function lean = lean_of(q2_arcs_deg)
% LEAN_OF  Upper-arm lean from vertical toward +x0 (deg) from ar3_fk's DH chain: shoulder = frame-1 origin,
%   elbow = frame-2 origin, q1 = 0 (independent of romefw_to_arcs_deg).
[~, P] = ar3_fk([0 q2_arcs_deg 0 0 0 0]);
lean = atan2d(P(1,3) - P(1,2), P(3,3) - P(3,2));
end

function chk(cond, msg)
if ~(isscalar(cond) && cond), error('test_bench:fail', 'FAIL: %s', msg); end
end

function K = read_constants(file)
% READ_CONSTANTS  arm_sign and arm_offset_deg from the TEXT of define_constants.m (not executed).
txt = fileread(file);
K.arm_sign = grab(txt, 'arm_sign');           % 1x6 +/-1
K.arm_offset_deg = grab(txt, 'arm_offset_deg'); % 1x6 deg
end

function v = grab(txt, name)
tok = regexp(txt, ['(?m)^' name '\s*=\s*\[([^\]]*)\]'], 'tokens');
if numel(tok) ~= 1, error('test_bench:parse', 'expected exactly one "%s = [...]" in define_constants.m, found %d', name, numel(tok)); end
v = sscanf(tok{1}{1}, '%f').';
if numel(v) ~= 6, error('test_bench:parse', '%s has %d numbers, expected 6', name, numel(v)); end
end

function log = make_jog_log(K, map, physDir)
% MAKE_JOG_LOG  Synthetic +5 deg jog log, one entry per joint. physDir(j) = +1 if the arm moved in the model's
%   positive direction for the +5 deg firmware jog (what the observer writes down); map decides the observed
%   ARCS direction (the slope of romefw_to_arcs_deg).
sw = [-180 132 141 -165 90 180];  ot = [160 0 1 165 -90 -170];   % firmware ends (deg) used only to pick a mid-range start
log = struct('j', {}, 'delta_deg', {}, 'before_lines', {}, 'after_lines', {}, ...
             'phys_model_dir', {}, 'phys_arcs_dir', {}, 'model_deg_obs', {}, 'lean_deg_obs', {});
for j = 1:6
    q = (sw + ot) / 2;                                            % mid-range firmware angles (deg)
    q2 = q;  q2(j) = q2(j) + 5;
    a0 = romefw_to_arcs_deg(q(:), map);  a1 = romefw_to_arcs_deg(q2(:), map);
    log(j).j = j;  log(j).delta_deg = 5;
    log(j).before_lines = ["GV,0,0,0,0,0,0,0,0"; sprintf("ARM,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,READY", q)];
    log(j).after_lines  = [sprintf("ARM,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,READY", (q + q2) / 2 * 0 + q); ...
                           sprintf("ARM,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,READY", q2)];
    log(j).phys_model_dir = physDir(j);
    log(j).phys_arcs_dir = sign(a1(j) - a0(j));
    log(j).model_deg_obs = (q(j) - K.arm_offset_deg(j)) / K.arm_sign(j);   % model angle of joint j before the jog (deg)
    log(j).lean_deg_obs = NaN;
end
end

function L = make_gv(k, rpmAct, rpmCmd, chans)
% MAKE_GV  10 raw GV lines: first 3 at rest, then the channels in chans at rpmAct; cmd column k = rpmCmd.
L = strings(10, 1);
for i = 1:10
    act = zeros(1, 4);  cmd = zeros(1, 4);  cmd(k) = rpmCmd;
    if i > 3, act(chans) = rpmAct; end
    L(i) = sprintf("GV,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", act, cmd);
end
end

function wl = make_wheel_log(chanOfK)
% MAKE_WHEEL_LOG  Rows: wheel 1..4 at +15 rpm, then wheel 1 at -15 rpm. Wheel k turns telemetry channel chanOfK(k).
wl = struct('k_cmd', {}, 'rpm_cmd', {}, 'gv_lines', {}, 'eye_wheel', {}, 'eye_bottom_xy', {});
ks = [1 2 3 4 1];  rp = [15 15 15 15 -15];
for i = 1:5
    wl(i).k_cmd = ks(i);  wl(i).rpm_cmd = rp(i);
    wl(i).gv_lines = make_gv(ks(i), rp(i), rp(i), chanOfK(ks(i)));
    wl(i).eye_wheel = 0;  wl(i).eye_bottom_xy = [NaN NaN];
end
end
