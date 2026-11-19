function test_ar3_fk()
%TEST_AR3_FK  Physical AR3 kinematics (vendor DH) and the ROME-firmware -> ARCS joint map.
%   Plan 1 Task 5. Errors if any assertion fails; all assertions are printed first.
%     i    200 random in-range ARCS postures: |P(:,3)-P(:,2)| = 0.305 m (a2) and
%          |P(:,5)-P(:,4)| = 0.22263 m (|d4|), within 1e-12 m
%     i-b  two postures worked by hand from the DH table, within 1e-12 m:
%          q = [0 -90 0 0 0 0] (upper arm and forearm vertical) and q = [0 0 0 0 0 0] (upper arm
%          horizontal along +x0)
%     ii   adding D to q1 rotates the T06 position about base z by D, within 1e-12 m
%     iii  T06(1:3,1:3) orthonormal (|R'R - I| < 1e-12) with det = 1 (within 1e-12)
%     iv   romefw_to_arcs_deg, both candidate maps ('rome_dirpin', the default, and 'arcs_caldir'):
%          q_arcs = s.*q_romefw + c, recovered by probing; every s(j) is +1 or -1 (1e-12); the map is
%          affine on random inputs (1e-12); each ROME switch-side limit (arm_fw_switch, read from
%          define_constants.m) lands on that map's ARCS anchor (1e-12); the ROME far end
%          (arm_fw_other) maps to the same side of the anchor as the ARCS far end; the default is
%          'rome_dirpin'; the maps agree on J1, J3, J4 and differ in s on J2, J5, J6 (R5.1).
%   The expected literals are independent of ar3_fk.m and romefw_to_arcs_deg.m: a2, d4, the ARCS
%   limits and the anchors are copied from ARbot.cal / ARCS.py (docs/verification/AR3_GEOMETRY_SOURCES.md),
%   the hand-worked positions from the DH table, arm_fw_switch / arm_fw_other from define_constants.m.
%   Flat, no local functions (CLAUDE.md section 5, crash 2).
here = fileparts(mfilename('fullpath'));    % ROME_merged/tests
root = fileparts(here);                     % ROME_merged
addpath(fullfile(root, 'analysis'));
fails = {};                                 % one message per failed assertion (cell of char)
tol   = 1e-12;                              % tolerance of every check (m, or dimensionless)

a2_m   = 0.305;                             % DH a2 (m), ARbot.cal tuple 60 '305.0' mm
d4_m   = 0.22263;                           % |DH d4| (m), ARbot.cal tuple 68 '-222.63' mm
q_lo   = [-170 -129.6   1   -164.5 -104.15 -148.1];   % 1x6 ARCS lower angle limits (deg), tuple 35..50
q_hi   = [ 170    0   143.7  164.5  104.15  148.1];   % 1x6 ARCS upper angle limits (deg), tuple 36..51

%% i  link lengths over 200 random in-range postures
rng(5);                                     % fixed seed, so a failure repeats
N = 200;                                    % number of random postures
e_a2 = 0;  e_d4 = 0;                        % worst |length error| seen (m)
for k = 1:N
    q = q_lo + rand(1, 6) .* (q_hi - q_lo);         % 1x6 random ARCS posture (deg)
    [~, P] = ar3_fk(q(:));                          % P 3x7 frame origins (m)
    e_a2 = max(e_a2, abs(norm(P(:,3) - P(:,2)) - a2_m));
    e_d4 = max(e_d4, abs(norm(P(:,5) - P(:,4)) - d4_m));
end
fprintf('i    %d postures: max | |P3-P2| - %.5f | = %.3g m, max | |P5-P4| - %.5f | = %.3g m\n', N, a2_m, e_a2, d4_m, e_d4);
if e_a2 > tol, fails{end+1} = sprintf('i: |P3-P2| differs from a2 = %.5f m by up to %.3g m', a2_m, e_a2); end
if e_d4 > tol, fails{end+1} = sprintf('i: |P5-P4| differs from |d4| = %.5f m by up to %.3g m', d4_m, e_d4); end

%% i-b  hand-worked postures
% q = [0 -90 0 0 0 0]: frame 1 at (a1, 0, d1); upper arm along +z0; J3 = 0 keeps the forearm on the
% same line (z3 = -z0 and d4 < 0), so the wrist centre is a further |d4| up.
[~, P] = ar3_fk([0; -90; 0; 0; 0; 0]);
Pexp = [0.0642 0.0642          0.0642;                        % 3x3 expected P(:,[2 3 5]) (m)
        0      0               0;
        0.16977 0.16977+0.305  0.16977+0.305+0.22263];
e_up = max(max(abs(P(:,[2 3 5]) - Pexp)));                    % worst coordinate error (m)
% q = 0: upper arm horizontal along +x0, P3 = P2 + [a2 0 0]
[~, P0] = ar3_fk(zeros(6, 1));
e_hz = max(abs(P0(:,3) - [0.0642 + 0.305; 0; 0.16977]));     % worst coordinate error (m)
fprintf('i-b  q=[0 -90 0 0 0 0]: max |P(:,[2 3 5]) - expected| = %.3g m; q=0: max |P(:,3) - expected| = %.3g m\n', e_up, e_hz);
if e_up > tol, fails{end+1} = sprintf('i-b: vertical posture off by %.3g m', e_up); end
if e_hz > tol, fails{end+1} = sprintf('i-b: q = 0 posture off by %.3g m', e_hz); end

%% ii  base rotation
e_rot = 0;                                  % worst position error (m)
for k = 1:N
    q  = q_lo + rand(1, 6) .* (q_hi - q_lo);        % 1x6 ARCS posture (deg)
    D  = -60 + 120 * rand;                          % added J1 angle (deg)
    T1 = ar3_fk(q(:));
    q2 = q;  q2(1) = q2(1) + D;
    T2 = ar3_fk(q2(:));
    Rz = [cosd(D) -sind(D) 0; sind(D) cosd(D) 0; 0 0 1];   % rotation about base z by D
    e_rot = max(e_rot, norm(T2(1:3,4) - Rz * T1(1:3,4)));
end
fprintf('ii   max |p(q1+D) - Rz(D) p(q)| = %.3g m\n', e_rot);
if e_rot > tol, fails{end+1} = sprintf('ii: base rotation error %.3g m', e_rot); end

%% iii  rotation part
e_orth = 0;  e_det = 0;                     % worst orthonormality and determinant errors (-)
for k = 1:N
    q = q_lo + rand(1, 6) .* (q_hi - q_lo);
    T = ar3_fk(q(:));
    R = T(1:3,1:3);
    e_orth = max(e_orth, max(max(abs(R.' * R - eye(3)))));
    e_det  = max(e_det, abs(det(R) - 1));
end
fprintf('iii  max |R''R - I| = %.3g, max |det R - 1| = %.3g\n', e_orth, e_det);
if e_orth > tol, fails{end+1} = sprintf('iii: R not orthonormal, %.3g', e_orth); end
if e_det  > tol, fails{end+1} = sprintf('iii: det R - 1 = %.3g', e_det); end
T = ar3_fk(zeros(6, 1));
if ~isequal(size(T), [4 4]) || any(T(4,:) ~= [0 0 0 1])
    fails{end+1} = 'iii: T06 is not a 4x4 homogeneous transform';
end

%% iv  firmware map
dct = fileread(fullfile(root, 'define_constants.m'));        % text of define_constants.m
tk  = regexp(dct, '(?m)^arm_fw_switch\s*=\s*\[([^\]]*)\]', 'tokens');
to  = regexp(dct, '(?m)^arm_fw_other\s*=\s*\[([^\]]*)\]', 'tokens');
if numel(tk) ~= 1 || numel(to) ~= 1
    fails{end+1} = sprintf('iv: expected one arm_fw_switch and one arm_fw_other line in define_constants.m, found %d and %d', numel(tk), numel(to));
    sw = nan(1, 6);  ot = nan(1, 6);
else
    sw = str2num(tk{1}{1});                 %#ok<ST2NM> 1x6 ROME switch-side limits (firmware deg)
    ot = str2num(to{1}{1});                 %#ok<ST2NM> 1x6 ROME far-end limits (firmware deg)
end
fprintf('iv   arm_fw_switch = %s, arm_fw_other = %s (define_constants.m)\n', mat2str(sw), mat2str(ot));
cal_end = [-170 -129.6 143.7 -164.5 -104.15  148.1];   % ARCS angle at the switch, CalDir 001001 (ARCS.py calRobot L4893-4963)
far_end = [ 170    0     1    164.5  104.15 -148.1];   % ARCS angle at the other end
maps    = {'rome_dirpin', 'arcs_caldir'};               % candidate map names
anchor  = {[cal_end(1) far_end(2) cal_end(3) cal_end(4) far_end(5) far_end(6)], cal_end};  % ARCS angle at the ROME switch, per map
across  = {[far_end(1) cal_end(2) far_end(3) far_end(4) cal_end(5) cal_end(6)], far_end};  % ARCS end opposite the anchor, per map
S = zeros(2, 6);                            % recovered s per map (-)
for m = 1:2
    f  = @(x) romefw_to_arcs_deg(x, maps{m});
    c  = f(zeros(6, 1));                    % 6x1 recovered offset (deg)
    s  = zeros(6, 1);                       % 6x1 recovered sign (-)
    for j = 1:6
        ej = zeros(6, 1);  ej(j) = 100;
        s(j) = (f(ej) - c)' * ej / 1e4;     % slope of joint j alone (-)
    end
    S(m,:) = s.';
    e_s = max(abs(abs(s) - 1));             % distance of |s| from 1 (-)
    x   = -200 + 400 * rand(6, 50);         % 6x50 random firmware postures (deg)
    e_aff = 0;                              % worst affinity error (deg)
    for k = 1:50
        y = f(x(:,k));
        if ~isequal(size(y), [6 1]), fails{end+1} = sprintf('iv %s: output is not 6x1', maps{m}); break; end %#ok<AGROW>
        e_aff = max(e_aff, max(abs(y - (round(s) .* x(:,k) + c))));
    end
    ysw = f(sw(:));                         % 6x1 ARCS angle at the ROME switch (deg)
    yot = f(ot(:));                         % 6x1 ARCS angle at the ROME far end (deg)
    e_anc = max(abs(ysw.' - anchor{m}));    % worst anchor miss (deg)
    side  = sign(yot.' - anchor{m}) == sign(across{m} - anchor{m});   % 1x6 far end on the right side
    fprintf('iv   %s: s = %s, c = %s\n', maps{m}, mat2str(s.', 6), mat2str(c.', 6));
    fprintf('       max ||s|-1| = %.3g, affine err %.3g deg, switch -> anchor err %.3g deg, far side ok %s\n', e_s, e_aff, e_anc, mat2str(side));
    fprintf('       far ends: ROME %s -> ARCS %s (ARCS end %s)\n', mat2str(ot), mat2str(yot.', 6), mat2str(across{m}));
    if e_s   > tol, fails{end+1} = sprintf('iv %s: a scale is not +-1 (max ||s|-1| = %.3g)', maps{m}, e_s); end %#ok<AGROW>
    if e_aff > tol, fails{end+1} = sprintf('iv %s: map is not s.*q + c (err %.3g deg)', maps{m}, e_aff); end %#ok<AGROW>
    if ~(e_anc <= tol), fails{end+1} = sprintf('iv %s: a switch-side limit misses its anchor by %.3g deg', maps{m}, e_anc); end %#ok<AGROW>
    if ~all(side), fails{end+1} = sprintf('iv %s: far end on the wrong side for joint(s) %s', maps{m}, mat2str(find(~side))); end %#ok<AGROW>
end
x = -200 + 400 * rand(6, 1);
if ~isequal(romefw_to_arcs_deg(x), romefw_to_arcs_deg(x, 'rome_dirpin'))
    fails{end+1} = 'iv: the default map is not rome_dirpin';
end
same = abs(S(1,:) - S(2,:)) < 0.5;          % 1x6 joints where the two maps have the same s
fprintf('iv   same s in both maps: %s (expected [1 0 1 1 0 0])\n', mat2str(same));
if ~isequal(same, logical([1 0 1 1 0 0]))
    fails{end+1} = sprintf('iv: the maps should differ in s on J2, J5, J6 only, got same = %s', mat2str(same));
end
threw = false;
try
    romefw_to_arcs_deg(zeros(6, 1), 'no_such_map');
catch
    threw = true;
end
if ~threw, fails{end+1} = 'iv: an unknown map name did not raise an error'; end

if ~isempty(fails)
    fprintf('\n');
    for k = 1:numel(fails), fprintf('FAIL  %s\n', fails{k}); end
    error('test_ar3_fk:fail', 'test_ar3_fk FAILED: %d assertion(s)', numel(fails));
end
fprintf('\ntest_ar3_fk PASS\n');
end
