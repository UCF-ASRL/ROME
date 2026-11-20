function test_motive_floor()
%TEST_MOTIVE_FLOOR  Pure-function tests of the floor frame (Plan 2 Task 2 = focus plan F5a).
%   Functions under test: Optitrack/motive_to_floor.m, Optitrack/floor_to_motive.m, Optitrack/yaw_unwrap.m,
%   and the consolidation of lab/lab_rec_to_floor.m onto motive_to_floor.
%   NO model, NO define_constants, NO network. Reads runs/case_1.mat (variable q) only; the model is not run.
%   Cal contract under test: p_floor = scale * R * p_motive + t (R Motive -> floor, t floor position of the Motive
%   origin in m, scale m per Motive unit). pose7 = [x y z qx qy qz qw], quaternion body -> Motive, Hamilton.
%   Cal variants (all 9 used in i, ii, iii-a, iii-b, c):
%     1 identity: R = I, t = 0, scale 1
%     2 Y-up Motive: R = Rx(+90 deg) (maps Motive +y to floor +z), t = 0, scale 0.001 (Motive in mm)
%     3 37 deg yaw offset: R = Rz(37 deg), t = [1.2; -0.4; 0] m, scale 1
%     4 Plan 3 Task 1 truth A: R = Rz(37 deg) Rx(0.5 deg), t = [1.2; -0.4; 0] m, scale 1
%     5 Plan 3 Task 1 truth B: R = Rz(37 deg) Rx(90 deg), t = [1.2; -0.4; 0.3] m, scale 0.001
%     6-9 Rx(180), Ry(180), Rz(180), Rz(180) Rx(90) (Y-up with Motive x opposite floor x, mm): they send
%         floor_to_motive's quaternion-of-R conversion through its A11, A22 and A33 branches (Shepperd), which
%         variants 1-5 never reach (all take the trace branch)
%   Checks (each prints a line; the test errors test_motive_floor:fail listing the failed check tags):
%     0     premises: the test's own quaternion of each R equals R (q2R); some scale ~= 1; variant 2 maps
%           Motive +y to floor +z; the variants cover all 4 Shepperd branches (computed here, first maximum
%           of [trace, A11, A22, A33] of A = R', the if/elseif order of floor_to_motive)
%     i     round trip motive_to_floor(floor_to_motive(b)) == b within 1e-12 for 100 random b plus psi = 0 and
%           psi = +-(pi - 1e-9); and the height of the returned point is z_rb within 1e-12
%     i-b   floor_to_motive equals an INDEPENDENT construction (test-local quaternion helpers,
%           p = R'(f - t)/s, q = conj(qR) x qz(psi)) within 1e-12 (quaternion compared up to sign)
%     ii    a pose built with the test helpers at b0 = [0.3; -0.2; 0.4] maps to b0 (absolute, 1e-12); moving
%           the rigid body +0.5 m along floor x / floor y gives Delta base [0.5;0;0] / [0;0.5;0] within 1e-12
%     iii-a a +90 deg rotation about floor z (test helpers) gives Delta psi = +pi/2 within 1e-12, position unchanged
%     iii-b scenario 1 lap: q(:,1:3) of runs/case_1.mat -> floor_to_motive -> motive_to_floor -> yaw_unwrap
%           (seeded with the first heading) reproduces the input within 1e-12 at every sample; no 2 pi jump
%           (max |diff(psi)| <= max |diff(q3)| + 1e-12). Two laps:
%           nominal  q(:,3) as saved. FINDING (first GREEN run, 19 Nov 2026): this saved lap never crosses
%                    +-pi (q3 in [-179.22, -55.85] deg, 0.78 deg from -pi at the closest), so the nominal lap
%                    CANNOT detect a missing unwrap; it is kept because the brief asks for it.
%           offset   the same lap with the heading shifted by dOff = -1 deg (more than the 0.78 deg margin: a
%                    start or tracking error of that size), which crosses -pi. This is the lap that exercises
%                    the unwrap. Premise (test-local arithmetic only, independent of the code under test): the
%                    shifted heading leaves (-pi, pi] at some sample, else the test errors test_motive_floor:setup.
%     iii-c yaw_unwrap on hand values (across +-pi, several turns, inside the band unchanged exactly)
%     c     consolidation: lab_rec_to_floor equals motive_to_floor column by column (within 1e-12; a vectorised product may round differently from a column one) on
%           100 random non-level poses with rejected frames; its source calls motive_to_floor and holds no atan2
%   Make-it-fail: tests/manual_makefail_motive_floor.m (source-level breaks of a temp copy first on the path).
%   Flat, no local functions (CLAUDE.md section 5, crash 2).
here = fileparts(mfilename('fullpath'));    % ROME_merged/tests
root = fileparts(here);                     % ROME_merged
addpath(fullfile(root, 'Optitrack'), fullfile(root, 'lab'), '-end');   % '-end': a mutated copy placed earlier on the path wins
fprintf('using: %s\n       %s\n       %s\n       %s\n', which('motive_to_floor'), which('floor_to_motive'), which('yaw_unwrap'), which('lab_rec_to_floor'));
wtp = which('wrapToPi');                    % where the toolbox wrapToPi resolves, if at all (char); NOT used by yaw_unwrap
if isempty(wtp), wtp = '(not on the path)'; end
fprintf('which wrapToPi: %s (yaw_unwrap wraps inline and does not call it)\n', wtp);
fails = {};                                 % tags of failed checks (cell of char)
rs = RandStream('mt19937ar', 'Seed', 20261120);   % private generator (no global rng change)
tol = 1e-12;                                % tolerance of every equality check (m or rad)
z_rb = 0.15;                                % rigid-body height above the floor used throughout (m), test value

% quaternion helpers, [w x y z], Hamilton (copied from test_lab_frame, checked there against Rodrigues)
qmul  = @(a, b) [a(1)*b(1)-a(2)*b(2)-a(3)*b(3)-a(4)*b(4); a(1)*b(2)+a(2)*b(1)+a(3)*b(4)-a(4)*b(3); ...
                 a(1)*b(3)-a(2)*b(4)+a(3)*b(1)+a(4)*b(2); a(1)*b(4)+a(2)*b(3)-a(3)*b(2)+a(4)*b(1)];
qconj = @(q) [q(1); -q(2:4)];
qax   = @(ax, ang) [cos(ang/2); sin(ang/2) * ax(:) / norm(ax)];   % rotation of ang (rad) about axis ax
q2R   = @(q) [1-2*(q(3)^2+q(4)^2), 2*(q(2)*q(3)-q(4)*q(1)), 2*(q(2)*q(4)+q(3)*q(1)); ...
              2*(q(2)*q(3)+q(4)*q(1)), 1-2*(q(2)^2+q(4)^2), 2*(q(3)*q(4)-q(2)*q(1)); ...
              2*(q(2)*q(4)-q(3)*q(1)), 2*(q(3)*q(4)+q(2)*q(1)), 1-2*(q(2)^2+q(3)^2)];
zax = [0; 0; 1];                            % floor z axis (floor frame)

%% cal variants
qRV = {[1; 0; 0; 0], qax([1 0 0], pi/2), qax(zax, 37*pi/180), ...
       qmul(qax(zax, 37*pi/180), qax([1 0 0], 0.5*pi/180)), qmul(qax(zax, 37*pi/180), qax([1 0 0], pi/2)), ...
       qax([1 0 0], pi), qax([0 1 0], pi), qax(zax, pi), qmul(qax(zax, pi), qax([1 0 0], pi/2))};   % quaternion of R per variant
sV = [1, 0.001, 1, 1, 0.001, 1, 1, 1, 0.001];   % scale per variant (m per Motive unit)
tV = {zeros(3, 1), zeros(3, 1), [1.2; -0.4; 0], [1.2; -0.4; 0], [1.2; -0.4; 0.3], ...
      [0.5; 0.2; 0], [-0.3; 0.6; 0], [0.8; -1.0; 0], [1.2; -0.4; 0.3]};   % floor position of the Motive origin (m)
nmV = {'1 identity', '2 Y-up, mm', '3 yaw 37 deg, t', '4 Task1 truth A', '5 Task1 truth B', ...
       '6 Rx(180)', '7 Ry(180)', '8 Rz(180)', '9 Y-up, x flip'};
RV = {eye(3), [1 0 0; 0 0 -1; 0 1 0], [cosd(37) -sind(37) 0; sind(37) cosd(37) 0; 0 0 1]};   % R written out where simple
for v = 4:numel(qRV), RV{v} = q2R(qRV{v}); end   % the others from the test's own quaternion
nV = numel(sV);                             % number of cal variants (count)
calV = cell(1, nV);
for v = 1:nV
    calV{v} = struct('R', RV{v}, 't', tV{v}, 'scale', sV(v));
end

%% 0  premises
e0 = 0;                                     % max |q2R(qR) - R| over variants (-)
for v = 1:nV, e0 = max(e0, max(abs(q2R(qRV{v}) - RV{v}), [], 'all')); end
brV = zeros(1, nV);                         % Shepperd branch floor_to_motive takes per variant: 1 trace, 2 A11, 3 A22, 4 A33
for v = 1:nV
    At = RV{v}.';                           % A = R' as in floor_to_motive (3x3)
    [~, brV(v)] = max([trace(At), At(1,1), At(2,2), At(3,3)]);   % first maximum on ties, the if/elseif order of floor_to_motive
end
c0 = e0 < 1e-15 && any(sV ~= 1) && isequal(RV{2} * [0; 1; 0], [0; 0; 1]) && all(ismember(1:4, brV));
fprintf('0     premises: max |q2R(qR) - R| %.1e, a scale ~= 1 present %d, Y-up maps +y to +z %d, Shepperd branches per variant %s (all 4 covered %d) -> %s\n', ...
        e0, any(sV ~= 1), isequal(RV{2} * [0; 1; 0], [0; 0; 1]), mat2str(brV), all(ismember(1:4, brV)), mat2str(c0));
if ~c0, error('test_motive_floor:setup', 'test premises do not hold (quaternions of R, scale variety, Y-up map, all 4 Shepperd branches)'); end

%% i  round trip, and i-b  floor_to_motive against an independent construction
nR = 100;                                   % random base poses (count)
Bi = [6 * rand(rs, 2, nR) - 3; (2 * rand(rs, 1, nR) - 1) * pi];   % x, y in [-3, 3] m, psi in (-pi, pi) rad, 3 x nR
Bi = [Bi, [0.1; 0.2; 0], [-0.7; 1.1; pi - 1e-9], [2.0; -1.5; -(pi - 1e-9)]];   % plus psi = 0 and psi near +-pi
nB = size(Bi, 2);                           % columns tested (count)
for v = 1:nV
    cal = calV{v};
    P7 = floor_to_motive(Bi, z_rb, cal);    % 7 x nB Motive poses
    Bo = motive_to_floor(P7, cal);          % 3 x nB back in the floor frame
    eI = max(abs(Bo - Bi), [], 'all');      % round-trip error (m / rad)
    zB = cal.scale * (cal.R * P7(1:3, :)) + cal.t;   % floor position of the returned point (m), 3 x nB
    eZ = max(abs(zB(3, :) - z_rb));         % height error (m)
    ci = isequal(size(P7), [7 nB]) && eI <= tol && eZ <= tol;
    fprintf('i     cal %-16s round trip max err %.2e (tol %.0e), height err %.2e, %d poses -> %s\n', nmV{v}, eI, tol, eZ, nB, mat2str(ci));
    if ~ci, fails{end+1} = sprintf('(i round trip, cal %s: err %.2e)', nmV{v}, eI); end %#ok<AGROW>
    eIb = 0;                                % max deviation from the independent construction
    for k = 1:nB
        pInd = cal.R.' * ([Bi(1:2, k); z_rb] - cal.t) / cal.scale;   % independent Motive position (Motive units)
        qInd = qmul(qconj(qRV{v}), qax(zax, Bi(3, k)));               % independent body quaternion [w x y z]
        qOut = [P7(7, k); P7(4:6, k)];                                % function output as [w x y z]
        eq = min(max(abs(qOut - qInd)), max(abs(qOut + qInd)));       % up to the sign of the quaternion
        eIb = max([eIb, max(abs(P7(1:3, k) - pInd)) * cal.scale, eq]);
    end
    cib = eIb <= tol;
    fprintf('i-b   cal %-16s floor_to_motive vs independent construction: max dev %.2e -> %s\n', nmV{v}, eIb, mat2str(cib));
    if ~cib, fails{end+1} = sprintf('(i-b independent construction, cal %s: dev %.2e)', nmV{v}, eIb); end %#ok<AGROW>
end

%% ii  absolute pose and +0.5 m pushes, iii-a  +90 deg rotation (poses built with the test helpers only)
b0 = [0.3; -0.2; 0.4];                      % start base pose (m, m, rad)
for v = 1:nV
    cal = calV{v}; qR = qRV{v};
    P0 = cal.R.' * ([b0(1:2); z_rb] - cal.t) / cal.scale;   % Motive position at b0 (Motive units)
    qb0 = qmul(qconj(qR), qax(zax, b0(3)));                  % body quaternion at b0 [w x y z]
    pose0 = [P0; qb0([2 3 4 1])];
    poseX = pose0; poseX(1:3) = P0 + cal.R.' * [0.5; 0; 0] / cal.scale;   % pushed +0.5 m along floor x
    poseY = pose0; poseY(1:3) = P0 + cal.R.' * [0; 0.5; 0] / cal.scale;   % pushed +0.5 m along floor y
    qb90 = qmul(qmul(qmul(qconj(qR), qax(zax, pi/2)), qR), qb0);           % rotated +90 deg about floor z
    pose90 = pose0; pose90(4:7) = qb90([2 3 4 1]);
    f0 = motive_to_floor(pose0, cal);
    dX = motive_to_floor(poseX, cal) - f0;  % Delta base for the x push
    dY = motive_to_floor(poseY, cal) - f0;  % Delta base for the y push
    d9 = motive_to_floor(pose90, cal) - f0; % Delta base for the rotation
    eA = max(abs(f0 - b0)); eX = max(abs(dX - [0.5; 0; 0])); eY = max(abs(dY - [0; 0.5; 0]));
    c2 = eA <= tol && eX <= tol && eY <= tol;
    fprintf('ii    cal %-16s absolute err %.2e, dx push %s err %.2e, dy push %s err %.2e -> %s\n', nmV{v}, eA, mat2str(dX.', 6), eX, mat2str(dY.', 6), eY, mat2str(c2));
    if ~c2, fails{end+1} = sprintf('(ii absolute/push, cal %s: %.2e %.2e %.2e)', nmV{v}, eA, eX, eY); end %#ok<AGROW>
    e9 = max(abs(d9 - [0; 0; pi/2]));
    c3a = e9 <= tol;
    fprintf('iii-a cal %-16s +90 deg rotation: Delta base %s (expected [0 0 %.6f]) err %.2e -> %s\n', nmV{v}, mat2str(d9.', 6), pi/2, e9, mat2str(c3a));
    if ~c3a, fails{end+1} = sprintf('(iii-a rotation +90 deg, cal %s: Delta psi %.6f)', nmV{v}, d9(3)); end %#ok<AGROW>
end

%% iii-b  scenario 1 lap through the full chain, with the unwrap
D = load(fullfile(root, 'runs', 'case_1.mat'), 'q');   % saved model run of scenario 1 (the model is not run here)
Q = D.q;                                    % N x 9 coordinates; columns 1:3 = base x (m), y (m), psi (rad)
nQ = size(Q, 1);                            % samples (count)
if size(Q, 2) < 3 || nQ < 2 || ~all(isfinite(Q(:, 1:3)), 'all')
    error('test_motive_floor:setup', 'runs/case_1.mat q is not an N x >=3 finite array (size %s)', mat2str(size(Q)));
end
dOff = -1 * pi/180;                         % heading shift of the offset lap (rad): -1 deg, Claude's call (author to confirm)
lapNm = {'nominal', 'offset'};              % lap names (cell of char)
lapOff = [0, dOff];                         % heading shift per lap (rad)
for L = 1:2
    q3 = Q(:, 3).' + lapOff(L);             % heading fed in and expected back (rad), 1 x N
    nWrapT = nnz(abs(atan2(sin(q3), cos(q3)) - q3) > pi);   % samples outside (-pi, pi], test-local arithmetic (count)
    fprintf('iii-b %s lap (shift %.1f deg): %d samples, start %.2f deg, range [%.2f, %.2f] deg, max |diff| %.4f rad, samples outside (-pi, pi] %d\n', ...
            lapNm{L}, lapOff(L)*180/pi, nQ, q3(1)*180/pi, min(q3)*180/pi, max(q3)*180/pi, max(abs(diff(q3))), nWrapT);
    if L == 2 && nWrapT == 0
        error('test_motive_floor:setup', 'offset lap never leaves (-pi, pi]: the unwrap is not exercised (min %.3f deg)', min(q3)*180/pi);
    end
    for v = 1:nV
        cal = calV{v};
        raw = motive_to_floor(floor_to_motive([Q(:, 1:2).'; q3], z_rb, cal), cal);   % 3 x N, psi in [-pi, pi]
        rawDev = max(abs(raw(3, :) - q3));  % largest raw-vs-true heading difference (rad): ~2 pi where q3 is outside (-pi, pi]
        psi = zeros(1, nQ);                 % unwrapped heading (rad)
        prev = q3(1);                       % seed, as the model block seeds from q0_9dof(3)
        for k = 1:nQ
            psi(k) = yaw_unwrap(raw(3, k), prev);
            prev = psi(k);
        end
        ePsi = max(abs(psi - q3));          % heading error (rad)
        eXY = max(abs(raw(1:2, :) - Q(:, 1:2).'), [], 'all');   % position error (m)
        jmp = max(abs(diff(psi)));          % largest step of the unwrapped heading (rad)
        c3b = ePsi <= tol && eXY <= tol && jmp <= max(abs(diff(q3))) + tol;
        fprintf('iii-b %-7s cal %-16s raw-vs-true max %.4f rad, unwrapped err %.2e rad, xy err %.2e m, max step %.4f rad -> %s\n', ...
                lapNm{L}, nmV{v}, rawDev, ePsi, eXY, jmp, mat2str(c3b));
        if ~c3b, fails{end+1} = sprintf('(iii-b %s lap, cal %s: psi err %.2e, xy err %.2e, max step %.3f)', lapNm{L}, nmV{v}, ePsi, eXY, jmp); end %#ok<AGROW>
    end
end

%% iii-c  yaw_unwrap on hand values
u1 = yaw_unwrap(-pi + 0.01, pi - 0.01);     % crossing +pi upward: expected pi + 0.01
u2 = yaw_unwrap(pi - 0.01, -pi + 0.01);     % crossing -pi downward: expected -pi - 0.01
u3 = yaw_unwrap(0.1, 0.1 + 4*pi);           % two turns up: expected 0.1 + 4 pi
u4 = yaw_unwrap(0.3, 0.2);                  % inside the band: expected 0.3 exactly
u5 = yaw_unwrap(0.2 + pi, 0.2);             % exactly half a turn: (-pi, pi] gives +pi
c3c = abs(u1 - (pi + 0.01)) <= tol && abs(u2 - (-pi - 0.01)) <= tol && abs(u3 - (0.1 + 4*pi)) <= tol && u4 == 0.3 && abs(u5 - (0.2 + pi)) <= tol;
fprintf('iii-c yaw_unwrap: %.6f (exp %.6f), %.6f (exp %.6f), %.6f (exp %.6f), %.17g (exp 0.3 exactly), %.6f (exp %.6f) -> %s\n', ...
        u1, pi + 0.01, u2, -pi - 0.01, u3, 0.1 + 4*pi, u4, u5, 0.2 + pi, mat2str(c3c));
if ~c3c, fails{end+1} = '(iii-c yaw_unwrap hand values)'; end

%% c  consolidation: lab_rec_to_floor is motive_to_floor
nC = 100;                                   % random poses (count)
qr = randn(rs, 4, nC); qr = qr ./ vecnorm(qr);   % random unit quaternions [w x y z], NOT level (4 x nC)
pr = 2000 * rand(rs, 3, nC) - 1000;         % random Motive positions (Motive units)
okr = rand(rs, 1, nC) > 0.1;                % accepted-frame mask (about 10 % rejected)
prR = [pr; qr([2 3 4 1], :)]; prR(:, ~okr) = NaN;   % rejected frames carry NaN, as record_motive stores them
rec = struct('t', (0:nC-1) / 100, 'pose7', prR, 'ok', okr);
eqAll = true;                               % agreement within tol over the variants (logical)
for v = 1:nV
    cal = calV{v};
    [fl, tt, idx] = lab_rec_to_floor(rec, cal);
    ref = zeros(3, numel(idx));             % motive_to_floor column by column
    for k = 1:numel(idx), ref(:, k) = motive_to_floor(rec.pose7(:, idx(k)), cal); end
    eqAll = eqAll && isequal(size(fl), size(ref)) && max(abs(fl - ref), [], 'all') <= tol && isequal(idx, find(okr)) && isequal(tt, rec.t(okr));
end
src = fileread(which('lab_rec_to_floor'));  % source of the lab helper actually on the path
srcOk = contains(src, 'motive_to_floor(') && ~contains(src, 'atan2');
cc = eqAll && srcOk;
fprintf('c     lab_rec_to_floor == motive_to_floor column by column (within 1e-12, %d poses, %d accepted, %d cals) %d; source calls motive_to_floor and has no atan2 %d -> %s\n', ...
        nC, nnz(okr), nV, eqAll, srcOk, mat2str(cc));
if ~cc, fails{end+1} = '(c consolidation: lab_rec_to_floor differs from motive_to_floor or re-implements it)'; end

if isempty(fails)
    fprintf('test_motive_floor: ALL CHECKS PASSED\n');
else
    fprintf('test_motive_floor: %d FAILED:\n', numel(fails));
    fprintf('  %s\n', fails{:});
    error('test_motive_floor:fail', 'FAIL: %d check(s): %s', numel(fails), strjoin(fails, ' '));
end
end
