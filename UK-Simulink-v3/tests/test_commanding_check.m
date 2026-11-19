function test_commanding_check()
%TEST_COMMANDING_CHECK  Plumbing and anchors of analysis/commanding_check.m (Plan 1 Task 6).
%   Errors if any assertion fails; all assertions are printed first. Flat, no local functions
%   (CLAUDE.md section 5, crash 2). Uses the 24 Sep 2026 run file copied to data/ and refuses to run
%   on any other bytes (SHA-256 below): a missing or changed file is an error, never a skip.
%     a  PLUMBING. The model's own DH table (parsed from ik9_fk.m) and the inverse of the run's
%        model -> firmware map are put in place of the AR3 in the physical path, clamp off, mount = the
%        model's (parsed from ik9_fk.m). Then ee_phys must equal ee_model (ik9_fk) on every sample
%        within 1e-9 m, and the clearance metrics and the tool-axis z component must equal the model's
%        within 1e-9 (m or -). The predicate is the anonymous function pass_a, shared with b.
%     b  MAKE-IT-FAIL of a, built in: the same with a2 lengthened by 1 mm must make pass_a (check a's
%        own assertion) false, with an ee error above 1e-4 m.
%     e  HOLD option (range-end candidates): holding J2 at firmware 130 sets fw_phys(:,2) = 130 on every
%        sample and leaves the other joints as in the as-used run (exact).
%     c  t = 0 ANCHORS from numbers this code did not compute. The run's arm_freeze_deg (the lab's
%        define_constants: the as-used map applied to q0_9dof) and the logged q_cmd(t=0) differ (the
%        block logs its state one step ahead, up to 0.11 deg on the joints, run log), so the check is
%        |fw_sent(t=0) - arm_freeze_deg| = |q_cmd(t=0) - q0_9dof| per joint (deg) within 1e-6 deg,
%        which holds only for a map of the form +-q + arm_offset_deg with the run's offsets. The upper
%        arm is 9.4 deg (rome_dirpin) and 41.0 deg (arcs_caldir) from vertical within 0.1 deg (Task 5,
%        docs/verification/2026-11-19_task5_photo_check.log, computed for the rounded park target
%        [-2.2 51.4 104.8 0 -23.8 -1.3]; q_cmd(t=0) moves J2 by 0.03 deg from it). The tool axis (-z6)
%        z component at t = 0 is -0.297 (rome_dirpin) and -0.928 (arcs_caldir) within 0.01: the same
%        log prints z6 = [.. 0.297] and [.. 0.928] at that target (checks the -z6 sign convention).
%     d  CONVENTION-CONSISTENT MAP. A chain with the AR3 DH structure (alpha, theta_off, signs of d4
%        and d6 from ar3_fk.m) and the MODEL's lengths, driven by q_arcs = conv_S.*q_model + conv_C
%        (the constants commanding_check uses), puts every frame origin where the model DH does
%        (1e-12 m) with every frame rotation equal to the model's times Rx(180 deg) (1e-12), on 200
%        random postures. And for fwmap 'consistent' the ARCS angles reached are conv_S.*q + conv_C
%        under both romefw maps wherever the clamp did not act (1e-9 deg).
here = fileparts(mfilename('fullpath'));    % ROME_merged/tests
root = fileparts(here);                     % ROME_merged
addpath(root, fullfile(root, 'analysis'));
fails = {};                                 % one message per failed assertion (cell of char)

matFile = fullfile(root, 'data', '9-24-26-15-33_Case_1_data.mat');   % the 24 Sep run, cmp-identical to the lab repo copy
sha_ok  = '32431044d61aaf0947736968ea6c7e0ca338065a2896f39f632136ae5bea225e';   % its SHA-256 (docs/verification/2026-11-19_task6_matfile_cmp.log)
if ~isfile(matFile), error('test_commanding_check:nofile', 'run file missing: %s', matFile); end
fid = fopen(matFile, 'r');  by = fread(fid, Inf, '*uint8');  fclose(fid);   % file bytes
md  = java.security.MessageDigest.getInstance('SHA-256');
sha = lower(reshape(dec2hex(typecast(md.digest(by), 'uint8'), 2).', 1, []));   % SHA-256, hex
fprintf('run file %s\n  SHA-256 %s\n', matFile, sha);
if ~strcmp(sha, sha_ok), error('test_commanding_check:sha', 'run file SHA-256 %s, expected %s', sha, sha_ok); end
S = load(matFile, 'arm_sign', 'arm_offset_deg', 'arm_freeze_deg', 'q0_9dof');

%% a  plumbing
txt = fileread(fullfile(root, 'ik9_fk.m'));                         % text of ik9_fk.m
tk  = regexp(txt, 'dh\s*=\s*\[([^\]]*)\]', 'tokens');
th  = regexp(txt, 'T=\[cos\(th\) -sin\(th\) 0 q\(1\); sin\(th\) cos\(th\) 0 q\(2\); 0 0 1 ([0-9.]+); 0 0 0 1\]', 'tokens');
if numel(tk) ~= 1 || numel(th) ~= 1
    error('test_commanding_check:parse', 'ik9_fk.m: expected one dh table and one base transform, found %d and %d', numel(tk), numel(th));
end
dh_m = str2num(['[' tk{1}{1} ']']);         %#ok<ST2NM> 6x4 model DH [a alpha d theta_off] (m, rad)
h0   = str2double(th{1}{1});                % model height of the arm base (m)
fprintf('a    ik9_fk.m: dh = %s, h0 = %.3f m\n', mat2str(dh_m, 6), h0);
arm.dh = dh_m;                              % the model's arm in place of the AR3
arm.s  = 1 ./ S.arm_sign(:);                % 6x1: model deg = (fw - offset) ./ arm_sign
arm.c  = -S.arm_offset_deg(:) ./ S.arm_sign(:);   % 6x1 (deg)
optp = struct('fwmap', 'as_used', 'clamp', false, 'mount', [eye(3) [0; 0; h0]; 0 0 0 1], 'arm', arm);
Rp = commanding_check(matFile, ones(6, 1), optp);
N  = numel(Rp.t);                           % samples
err_a  = @(R) [max(sqrt(sum((R.ee_phys - R.ee_model).^2, 2))), max(abs(R.min_link_z - R.model_min_link_z)), ...
                max(abs(R.min_link_to_base_xy - R.model_min_link_to_base_xy)), max(abs(R.tool_z - R.model_tool_z))];
                                            % 1x4 worst ee (m), min-z (m), signed-xy (m), tool-z (-) differences
pass_a = @(e) all(e <= 1e-9);               % check a's assertion on those four numbers
ea = err_a(Rp);                             % 1x4 check-a errors on the plumbing arm
fprintf('a    %d samples, t = %.3f .. %.3f s: max|ee_phys - ee_model| = %.3g m, max|dz| = %.3g m, max|dxy| = %.3g m, max|dtool_z| = %.3g (tol 1e-9)\n', ...
        N, Rp.t(1), Rp.t(end), ea(1), ea(2), ea(3), ea(4));
if N < 2, fails{end+1} = 'a: fewer than 2 samples'; end
if ~pass_a(ea), fails{end+1} = sprintf('a: plumbing errors %s exceed 1e-9', mat2str(ea, 3)); end

%% b  make-it-fail of a: a2 + 1 mm
armf = arm;  armf.dh(2, 1) = armf.dh(2, 1) + 1e-3;   % a2 lengthened by 1 mm
optf = optp;  optf.arm = armf;
Rf = commanding_check(matFile, ones(6, 1), optf);
ef = err_a(Rf);                             % 1x4 check-a errors on the perturbed arm
verdict = 'pass';                           % what check a's assertion says on the perturbed arm
if ~pass_a(ef), verdict = 'FAIL'; end
fprintf('b    a2 + 1 mm: check-a errors %s; check a''s assertion pass_a -> %s (must be FAIL, ee error > 1e-4 m)\n', mat2str(ef, 3), verdict);
if pass_a(ef), fails{end+1} = 'b: check a passes on an arm with a2 + 1 mm'; end
if ~(ef(1) > 1e-4), fails{end+1} = sprintf('b: a 1 mm a2 error moved ee by only %.3g m', ef(1)); end

%% c  t = 0 anchors
maps = {'rome_dirpin', 'arcs_caldir'};      % candidate firmware -> ARCS maps
ang_exp = [9.4 41.0];                       % upper arm from vertical at the park target (deg), Task 5 log
tz_exp  = [-0.297 -0.928];                  % tool axis (-z6) z component at the park target (-), Task 5 log
for m = 1:2
    Rc = commanding_check(matFile, ones(6, 1), struct('romefw', maps{m}));
    dq0    = abs(rad2deg(Rc.q_cmd(1, 4:9) - S.q0_9dof(4:9).'));      % 1x6 |q_cmd(t=0) - q0_9dof| (deg)
    e_park = max(abs(abs(Rc.fw_sent(1, :) - S.arm_freeze_deg(:).') - dq0));   % (deg)
    u  = Rc.P_body(:, 3, 1) - Rc.P_body(:, 2, 1);                    % upper arm, shoulder -> elbow (m)
    au = acosd(u(3) / norm(u));                                      % angle from vertical (deg)
    fprintf('c    %s: fw(t=0) = %s, |q_cmd(t=0) - q0| = %s deg, max ||fw(t=0) - arm_freeze_deg| - |dq0|| = %.3g deg; upper arm %.3f deg from vertical (expected %.1f)\n', ...
            maps{m}, mat2str(Rc.fw_sent(1, :), 6), mat2str(dq0, 3), e_park, au, ang_exp(m));
    if ~(e_park <= 1e-6), fails{end+1} = sprintf('c %s: t = 0 command off the arm_freeze_deg anchor by %.3g deg', maps{m}, e_park); end %#ok<AGROW>
    fprintf('c    %s: tool axis z component at t = 0 %.4f (expected %.3f)\n', maps{m}, Rc.tool_z(1), tz_exp(m));
    if ~(abs(Rc.tool_z(1) - tz_exp(m)) <= 0.01), fails{end+1} = sprintf('c %s: tool z %.4f, expected %.3f', maps{m}, Rc.tool_z(1), tz_exp(m)); end %#ok<AGROW>
    if ~(abs(au - ang_exp(m)) <= 0.1), fails{end+1} = sprintf('c %s: upper arm %.3f deg from vertical, expected %.1f', maps{m}, au, ang_exp(m)); end %#ok<AGROW>
end

%% d  convention-consistent map
Sc = Rc.conv_S;  Cc = Rc.conv_C;            % 1x6 the constants commanding_check uses
al_a = deg2rad([-90 0 90 -90 90 0]);        % AR3 alpha (rad), ar3_fk.m L20
to_a = deg2rad([0 0 -90 0 0 180]);          % AR3 theta_off (rad), ar3_fk.m L23
a_a  = dh_m(:, 1).';                        % model lengths a (m)
d_a  = dh_m(:, 3).' .* [1 1 1 -1 1 -1];     % model |d| with the AR3 signs of d4, d6 (m), ar3_fk.m L22
Rx180 = diag([1 -1 -1]);                    % rotation of 180 deg about x
rng(6);                                     % fixed seed, so a failure repeats
e_pos = 0;  e_rot = 0;                      % worst position (m) and rotation (-) mismatch
for k = 1:200
    qm = -180 + 360 * rand(1, 6);           % 1x6 random model posture (deg)
    qa = Sc .* qm + Cc;                     % 1x6 ARCS angles from the derived map (deg)
    Tm = eye(4);  Ta = eye(4);
    for i = 1:6
        t1 = deg2rad(qm(i)) + dh_m(i, 4);  b1 = dh_m(i, 2);
        Tm = Tm * [cos(t1) -sin(t1)*cos(b1) sin(t1)*sin(b1) dh_m(i,1)*cos(t1); sin(t1) cos(t1)*cos(b1) -cos(t1)*sin(b1) dh_m(i,1)*sin(t1); 0 sin(b1) cos(b1) dh_m(i,3); 0 0 0 1];
        t2 = deg2rad(qa(i)) + to_a(i);  b2 = al_a(i);
        Ta = Ta * [cos(t2) -sin(t2)*cos(b2) sin(t2)*sin(b2) a_a(i)*cos(t2); sin(t2) cos(t2)*cos(b2) -cos(t2)*sin(b2) a_a(i)*sin(t2); 0 sin(b2) cos(b2) d_a(i); 0 0 0 1];
        e_pos = max(e_pos, norm(Ta(1:3, 4) - Tm(1:3, 4)));
        e_rot = max(e_rot, max(max(abs(Ta(1:3, 1:3) - Tm(1:3, 1:3) * Rx180))));
    end
end
fprintf('d    conv_S = %s, conv_C = %s: 200 postures, max frame position error %.3g m, max |Ra - Rm Rx(180)| = %.3g\n', ...
        mat2str(Sc), mat2str(Cc), e_pos, e_rot);
if ~(e_pos <= 1e-12), fails{end+1} = sprintf('d: derived map misplaces a frame by %.3g m', e_pos); end
if ~(e_rot <= 1e-12), fails{end+1} = sprintf('d: derived map frame rotation off by %.3g', e_rot); end
for m = 1:2
    Rk = commanding_check(matFile, ones(6, 1), struct('romefw', maps{m}, 'fwmap', 'consistent'));
    qexp = Sc .* rad2deg(Rk.q_cmd(:, 4:9)) + Cc;                   % Nx6 ARCS angles the model means (deg)
    ok = ~Rk.clamp_hit;                                              % Nx6 entries the clamp left alone
    e_c = max(abs(Rk.q_arcs(ok) - qexp(ok)));                        % (deg)
    fprintf('d    fwmap consistent, %s: arm_sign = %s, arm_offset_deg = %s, clamp hits per joint %s, max |q_arcs - expected| = %.3g deg\n', ...
            maps{m}, mat2str(Rk.arm_sign), mat2str(Rk.arm_offset_deg, 6), mat2str(sum(Rk.clamp_hit, 1)), e_c);
    if ~(e_c <= 1e-9), fails{end+1} = sprintf('d %s: consistent map reaches ARCS angles off by %.3g deg', maps{m}, e_c); end %#ok<AGROW>
end

%% e  hold option
Rh = commanding_check(matFile, ones(6, 1), struct('hold', [NaN; 130; NaN; NaN; NaN; NaN]));
R1 = commanding_check(matFile, ones(6, 1));
e_h2 = max(abs(Rh.fw_phys(:, 2) - 130));                             % (deg)
e_ho = max(max(abs(Rh.fw_phys(:, [1 3:6]) - R1.fw_phys(:, [1 3:6]))));   % (deg)
fprintf('e    hold J2 at 130: max|fw_phys(:,2) - 130| = %.3g deg, other joints max diff %.3g deg\n', e_h2, e_ho);
if ~(e_h2 == 0), fails{end+1} = sprintf('e: held joint off by %.3g deg', e_h2); end
if ~(e_ho == 0), fails{end+1} = sprintf('e: hold changed other joints by %.3g deg', e_ho); end

if ~isempty(fails)
    fprintf('\n');
    for k = 1:numel(fails), fprintf('FAIL  %s\n', fails{k}); end
    error('test_commanding_check:fail', 'test_commanding_check FAILED: %d assertion(s)', numel(fails));
end
fprintf('\ntest_commanding_check PASS\n');
end
