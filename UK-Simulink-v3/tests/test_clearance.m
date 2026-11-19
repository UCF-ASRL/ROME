function test_clearance()
%TEST_CLEARANCE  clearance_9dof: arm link points against the floor and the base footprint (Plan 1 Task 7).
%   Errors if any assertion fails; all assertions are printed first. Two small local functions at the foot
%   (tool_z, oracle_xy), each calling only library code (CLAUDE.md section 5, crash 2). Does not run define_constants.m (it only reads its text).
%   z_min is the lowest link point of the arm (m); xy_min is the smallest signed horizontal distance of the
%   distal link points (elbow to flange) to the base square d x w (m): positive outside, negative inside.
%     a  CONSTANTS read from define_constants.m: d = w = 0.45 m (verified here, the plan says 0.45 x 0.45).
%     b  arm_home, 'paper' (the model's kinematics): z_min > 0.05 m and xy_min > 0 (the elbow sits 18 mm
%        outside the square, see the log), each bracketed by an oracle that does not call clearance_9dof
%        (ik9_fk frame origins, finer sampling).
%     c  FLOOR: a pose whose tool (frame 6) is 0.02 m below the floor, built by a root-finder on J3 (upper arm hanging down, J2 = 0) and
%        confirmed by ik9_fk alone (|z6 + 0.02| < 1e-9): z_min < 0 and z_min <= -0.02.
%     d  DECK: a pose that folds the forearm over the base (every frame from the elbow on inside the
%        square, confirmed from ik9_fk frame origins): xy_min < 0.
%     e  INVARIANCE: the same arm posture with the base at (1.3, -0.7) m, yaw 1.1 rad gives the same z_min
%        and xy_min as at the origin (1e-12), because both are measured in the base body frame.
%     f  'ar3' (vendor DH on the model's mount: 0.20 m up, yaw 0, from ik9_fk, default map rome_dirpin):
%        z_min on arm_home equals the minimum of 0.20 + z of the ar3_fk frame origins (1e-12);
%        a missing arm_sign or an unknown geom is an error.
%     g  TIE to analysis/commanding_check on the 24 Sep run (data/, every 25th sample): 'paper' equals its
%        model_min_link_z and model_min_link_to_base_xy, and 'ar3' equals its min_link_z and
%        min_link_to_base_xy where the clamp did not act, 1e-12.
%     h  MAKE-IT-FAIL of b, c, d built in: the arm_home verdict (z_min > 0.05, xy_min > 0) must be FALSE
%        on the below-floor pose, and the deck verdict (xy_min < 0) must be FALSE on arm_home.
here = fileparts(mfilename('fullpath'));    % ROME_merged/tests
root = fileparts(here);                     % ROME_merged
addpath(root, fullfile(root, 'analysis'));
fails = {};                                 % one message per failed assertion (cell of char)

%% a  constants
txt = fileread(fullfile(root, 'define_constants.m'));          % text of define_constants.m
tkd = regexp(txt, '^d\s*=\s*([0-9.]+);', 'tokens', 'lineanchors');
tkw = regexp(txt, '^w\s*=\s*([0-9.]+);', 'tokens', 'lineanchors');
tks = regexp(txt, '^arm_sign\s*=\s*\[([^\]]*)\]', 'tokens', 'lineanchors');
tko = regexp(txt, '^arm_offset_deg\s*=\s*\[([^\]]*)\]', 'tokens', 'lineanchors');
tkh = regexp(txt, '^arm_home\s*=\s*deg2rad\(\[([^\]]*)\]\)', 'tokens', 'lineanchors');
if numel(tkd) ~= 1 || numel(tkw) ~= 1 || numel(tks) ~= 1 || numel(tko) ~= 1 || numel(tkh) ~= 1
    error('test_clearance:parse', 'define_constants.m: expected one each of d, w, arm_sign, arm_offset_deg, arm_home; found %d %d %d %d %d', ...
          numel(tkd), numel(tkw), numel(tks), numel(tko), numel(tkh));
end
opt = struct();                             % options of clearance_9dof
opt.d = str2double(tkd{1}{1});              % base length (m)
opt.w = str2double(tkw{1}{1});              % base width (m)
opt.arm_sign = str2num(tks{1}{1});          %#ok<ST2NM> 1x6 model -> firmware sign (-)
opt.arm_offset_deg = str2num(tko{1}{1});    %#ok<ST2NM> 1x6 model -> firmware offset (deg)
ah = deg2rad(str2num(tkh{1}{1}));           %#ok<ST2NM> 1x6 arm_home (rad); the file writes it as a column
ah = ah(:);                                 % 6x1 arm_home (rad)
hx = opt.d / 2;  hy = opt.w / 2;            % half sides of the base square (m)
fprintf('a    define_constants.m: d = %g m, w = %g m, arm_home = %s deg\n', opt.d, opt.w, mat2str(rad2deg(ah.'), 5));
if opt.d ~= 0.45 || opt.w ~= 0.45
    fails{end+1} = sprintf('a: d x w = %g x %g m, the plan says 0.45 x 0.45', opt.d, opt.w);
end

%% b  arm_home, paper
q_home = [0; 0; 0; ah];                     % 9x1 arm_home on a base at the origin (m, rad)
[zb, xb] = clearance_9dof(q_home, 'paper', opt);
[~, ~, Pb] = ik9_fk(q_home);                % 3x7 frame origins, body frame (m)
zo = min(Pb(3, 2:7));                       % zo: oracle z_min (m): a straight segment's lowest point is an end point (m)
[xe, xf] = oracle_xy(Pb, hx, hy);           % xe, xf: oracle bracket for xy_min (m), upper then lower: end points only >= true, fine sampling <= true (m)
fprintf('b    arm_home paper: z_min %.6f m (oracle %.6f), xy_min %.6f m (oracle bracket %.6f .. %.6f)\n', zb, zo, xb, xf, xe);
if ~(zb > 0.05),                 fails{end+1} = sprintf('b: arm_home z_min %.4f is not above 0.05 m', zb); end
if abs(zb - zo) > 1e-12,         fails{end+1} = sprintf('b: z_min %.12f differs from the oracle %.12f', zb, zo); end
if ~(xb > 0),                    fails{end+1} = sprintf('b: arm_home xy_min %.4f is not positive', xb); end
if ~(xb <= xe + 1e-12 && xb >= xf - 1e-12), fails{end+1} = sprintf('b: xy_min %.6f outside the oracle bracket [%.6f, %.6f]', xb, xf, xe); end

%% c  tool 0.02 m below the floor
zt = -0.02;                                 % target height of frame 6 (m)
ah0 = ah;  ah0(2) = 0;                      % 6x1 arm_home with the upper arm hanging straight down (J2 = 0, rad)
fz = @(q3) tool_z(q3, ah0);                 % z of frame 6 as a function of J3 (rad), by ik9_fk only
j3 = deg2rad(-10:1:90);                     % scan of J3 from arm_home's -10 deg towards 90 deg, forearm aligned (rad)
k  = find(arrayfun(fz, j3) < zt, 1);        % first J3 in the scan with the tool below the target
if isempty(k) || k == 1
    error('test_clearance:pose', 'no J3 in -10..90 deg puts the tool below %.2f m', zt);
end
j3r = fzero(@(x) fz(x) - zt, [j3(k-1), j3(k)]);   % J3 that puts frame 6 at zt (rad)
ahc = ah0;  ahc(3) = j3r;                   % 6x1 arm joints of the floor pose (rad)
q_fl = [0; 0; 0; ahc];                      % 9x1 floor pose
[pe_c, ~, Pc] = ik9_fk(q_fl);
[zc, xc] = clearance_9dof(q_fl, 'paper', opt);   % z_min, xy_min of the floor pose (m)
fprintf('c    floor pose: J2 = 0, J3 = %.3f deg, ik9_fk tool z = %.9f m; z_min %.6f m, xy_min %.6f m, lowest frame %d\n', ...
        rad2deg(j3r), pe_c(3), zc, xc, find(Pc(3, :) == min(Pc(3, 2:7)), 1));
if abs(pe_c(3) - zt) > 1e-9,     fails{end+1} = sprintf('c: ik9_fk tool z %.9f is not %.2f', pe_c(3), zt); end
if ~(zc < 0),                    fails{end+1} = sprintf('c: z_min %.4f is not below 0', zc); end
if ~(zc <= zt + 1e-9),           fails{end+1} = sprintf('c: z_min %.6f is above the tool height %.2f', zc, zt); end
if abs(zc - min(Pc(3, 2:7))) > 1e-12, fails{end+1} = 'c: z_min differs from the oracle'; end

%% d  forearm folded over the deck
q_dk = [0; 0; 0; 0; deg2rad(-40); deg2rad(-90); 0; 0; 0];   % 9x1: upper arm tilted back, forearm folded over the base
[~, ~, Pd] = ik9_fk(q_dk);
[zd, xd] = clearance_9dof(q_dk, 'paper', opt);
in_d = all(abs(Pd(1, 3:7)) < hx) && all(abs(Pd(2, 3:7)) < hy);   % every frame from the elbow on inside the square (ik9_fk only)
fprintf('d    deck pose: frames 3..7 x = %s, y = %s (half sides %.3f); z_min %.4f m, xy_min %.6f m\n', ...
        mat2str(Pd(1, 3:7), 4), mat2str(Pd(2, 3:7), 4), hx, zd, xd);
if ~in_d,                        fails{end+1} = 'd: the constructed pose does not put the elbow, wrist and flange inside the square'; end
if ~(xd < 0),                    fails{end+1} = sprintf('d: xy_min %.4f is not negative', xd); end
if ~(xd >= -hx),                 fails{end+1} = sprintf('d: xy_min %.4f is deeper than the half side %.3f', xd, hx); end
if ~(zd > 0),                    fails{end+1} = sprintf('d: the deck pose is below the floor (z_min %.4f), not a clean xy test', zd); end

%% e  invariance under base position and yaw
q_mv = q_home;  q_mv(1:3) = [1.3; -0.7; 1.1];       % same arm posture, base moved and turned (m, rad)
[ze, xe2] = clearance_9dof(q_mv, 'paper', opt);
[zf, xf2] = clearance_9dof(q_dk, 'paper', opt);  q_mv2 = q_dk;  q_mv2(1:3) = [-0.4; 2.2; -2.0];
[zg, xg] = clearance_9dof(q_mv2, 'paper', opt);
fprintf('e    moved base: dz %.2e, dxy %.2e (arm_home); dz %.2e, dxy %.2e (deck pose)\n', ze - zb, xe2 - xb, zg - zf, xg - xf2);
if abs(ze - zb) > 1e-12 || abs(xe2 - xb) > 1e-12 || abs(zg - zf) > 1e-12 || abs(xg - xf2) > 1e-12
    fails{end+1} = 'e: z_min or xy_min changes with the base position or yaw';
end

%% f  ar3
[za, xa] = clearance_9dof(q_home, 'ar3', opt);
fwh = opt.arm_sign(:) .* rad2deg(ah) + opt.arm_offset_deg(:);     % 6x1 arm_home, firmware deg
[~, Pa] = ar3_fk(romefw_to_arcs_deg(fwh, 'rome_dirpin'));         % 3x7 AR3 frame origins, arm base frame (m)
zao = Pb(3, 1) + min(Pa(3, 2:7));           % oracle: mount height from ik9_fk frame 0 plus the lowest AR3 frame (m)
fprintf('f    arm_home ar3 (rome_dirpin, mount z %.2f m): z_min %.6f m (oracle %.6f), xy_min %.6f m\n', Pb(3, 1), za, zao, xa);
if abs(za - zao) > 1e-12,        fails{end+1} = sprintf('f: ar3 z_min %.12f differs from the oracle %.12f', za, zao); end
e1 = '';  % e1: error identifier (char) of 'ar3' without arm_sign, '' = no error
try, clearance_9dof(q_home, 'ar3', struct('d', opt.d, 'w', opt.w)); catch me, e1 = me.identifier; end
e2 = '';  % e2: error identifier (char) of an unknown geom, '' = no error
try, clearance_9dof(q_home, 'bogus', opt); catch me, e2 = me.identifier; end
fprintf('f    errors: ar3 without arm_sign -> ''%s'', geom ''bogus'' -> ''%s''\n', e1, e2);
if isempty(e1), fails{end+1} = 'f: ar3 without arm_sign did not error'; end
if isempty(e2), fails{end+1} = 'f: an unknown geom did not error'; end

%% g  tie to commanding_check on the 24 Sep run
matFile = fullfile(root, 'data', '9-24-26-15-33_Case_1_data.mat');   % the 24 Sep run (SHA-256 is pinned in test_commanding_check)
if ~isfile(matFile), error('test_clearance:nofile', 'run file missing: %s', matFile); end
Rc = commanding_check(matFile, ones(6, 1));     % default options: as-used map, rome_dirpin, clamp on, mount 0.20 m
idx = 1:25:numel(Rc.t);                         % sampled rows (-)
ep = 0;  ea = 0;  na = 0;                       % worst paper difference, worst ar3 difference (m), ar3 samples compared
for k = idx
    [zp, xp] = clearance_9dof(Rc.q_cmd(k, :).', 'paper', opt);
    ep = max([ep, abs(zp - Rc.model_min_link_z(k)), abs(xp - Rc.model_min_link_to_base_xy(k))]);
    if ~any(Rc.clamp_hit(k, :))
        [zq, xq] = clearance_9dof(Rc.q_cmd(k, :).', 'ar3', opt);
        ea = max([ea, abs(zq - Rc.min_link_z(k)), abs(xq - Rc.min_link_to_base_xy(k))]);
        na = na + 1;
    end
end
fprintf('g    vs commanding_check, %d samples: paper worst %.2e m; ar3 worst %.2e m over %d unclamped samples\n', numel(idx), ep, ea, na);
if ep > 1e-12,                   fails{end+1} = sprintf('g: paper differs from commanding_check by %.3g', ep); end
if ea > 1e-12,                   fails{end+1} = sprintf('g: ar3 differs from commanding_check by %.3g', ea); end
if na == 0,                      fails{end+1} = 'g: every sampled row was clamped, nothing compared for ar3'; end

%% h  make-it-fail, built in
home_ok = @(z, x) z > 0.05 && x > 0;        % the verdict arm_home must satisfy (b)
deck_ok = @(x) x < 0;                       % the verdict the deck pose must satisfy (d)
h1 = home_ok(zc, xc);  h2 = deck_ok(xb);   % h1, h2: verdicts (logical) that must both be false
fprintf('h    arm_home verdict on the floor pose: %d (must be 0); deck verdict on arm_home: %d (must be 0)\n', h1, h2);
if h1, fails{end+1} = 'h: the arm_home verdict also passes the below-floor pose, so b cannot fail'; end
if h2, fails{end+1} = 'h: the deck verdict also passes arm_home, so d cannot fail'; end

if ~isempty(fails)
    error('test_clearance:fail', '%d assertion(s) failed:\n  %s', numel(fails), strjoin(fails, sprintf('\n  ')));
end
fprintf('test_clearance PASS\n');
end

function z = tool_z(j3, ah)
%TOOL_Z  Height of frame 6 (m) for J3 = j3 (rad), the other arm joints as in ah (6x1, rad); ik9_fk only.
ahx = ah;  ahx(3) = j3;
pe = ik9_fk([0; 0; 0; ahx]);
z = pe(3);
end

function [s_end, s_fine] = oracle_xy(P, hx, hy)
%ORACLE_XY  Bracket of the smallest signed distance to the square, frames 2..6 (columns 3..7 of P, m).
%   s_end: end points only (an upper bound of the true minimum along the segments).
%   s_fine: 2001 points per segment (within spacing of the true minimum below it).
%   Written without clearance_9dof or link_clearance: inside the square the distance is max(dx, dy),
%   outside it is hypot(max(dx, 0), max(dy, 0)).
s_end = inf;  s_fine = inf;
for i = 3:6
    for u = linspace(0, 1, 2001)
        X = P(:, i) + u * (P(:, i+1) - P(:, i));
        dx = abs(X(1)) - hx;  dy = abs(X(2)) - hy;
        if dx < 0 && dy < 0, s = max(dx, dy); else, s = hypot(max(dx, 0), max(dy, 0)); end
        s_fine = min(s_fine, s);
        if u == 0 || u == 1, s_end = min(s_end, s); end
    end
end
end
