function R = commanding_check(matFile, signFlips, opt)
%COMMANDING_CHECK  Where the real AR3 put its links on a logged run, against where the model believed.
%   R = commanding_check(matFile, signFlips)
%   R = commanding_check(matFile, signFlips, opt)
%   Plan 1 Task 6 (Part B2 of docs/2026-11-18-rome-sim-to-lab-plan.md). Analysis only.
%
%   matFile    run file saved by the lab ROME_9DOF model: out.q_cmd (timeseries, 9x1xN, the UKDynamics
%              position command [x y theta q1..q6], m and rad) and the define_constants workspace
%              (arm_sign, arm_offset_deg, arm_fw_switch, arm_fw_other, arm_fw_margin_deg, d, w).
%              In the lab model at commit 63a10aa, q_cmd is UKDynamics output 1; elements 4:9 go through
%              CommandGuard (passes finite joints unchanged) to ROMECommand, which sends
%              arm_clamp(arm_sign.*deg + arm_offset_deg) (ROMECommand.m 63a10aa L102, L111-113).
%   signFlips  6x1 of +1 / -1, the direction hypothesis per joint (+1 = as commanded), see opt.flip.
%   opt        struct, every field optional:
%     romefw   'rome_dirpin' (default) | 'arcs_caldir': firmware -> ARCS map (romefw_to_arcs_deg)
%     fwmap    'as_used' (default): arm_sign, arm_offset_deg saved in matFile
%              'consistent': the model -> ARCS map derived below (Sc, Cc), turned into firmware degrees
%              through the inverse of the opt.romefw map, so the firmware would reach the posture the
%              model means
%     flip     'map' (default): signFlips multiply arm_sign, offset kept:
%                  fw = (signFlips.*arm_sign).*q_deg + arm_offset_deg, then the clamp
%              'motion': the command actually sent (as_used map, clamp) is reversed about its t = 0
%                  value on the flipped joints, fw = fw0 + signFlips.*(fw_sent - fw0), then held
%                  inside the firmware range with no margin (the arm stops at a range end)
%     clamp    true (default): arm_clamp with the run's arm_fw_switch, arm_fw_other, arm_fw_margin_deg
%     mount    4x4 pose of the arm base frame 0 in the ROME base body frame (rotation, position in m).
%              Default [eye(3) [0;0;0.20]; 0 0 0 1], the mount the model itself assumes: ik9_fk.m L16
%              and mass_at.m L126 (h0) / L288. NOT MEASURED on the robot.
%     arm      [] (default): the physical AR3, frames ar3_fk(romefw_to_arcs_deg(fw, romefw)).
%              Or a struct with dh (6x4, rows [a (m) alpha (rad) d (m) theta_off (rad)]) and s, c
%              (6x1): joint angle (rad) = deg2rad(s.*fw + c). A plain DH arm in place of the AR3, used
%              by the plumbing test (tests/test_commanding_check.m). Optional field tool_sign (default
%              +1): the tool axis is tool_sign times the last frame's z axis.
%     hold     6x1, NaN (default) or a firmware angle (deg) per joint: that joint is held at that value
%              for the whole run, after map, flip and clamp; the other joints are unchanged. Used for the
%              range-end candidates (the documented J1/J2/J5 bug drove joints to a range end).
%
%   R fields (N samples, SI unless the name says deg):
%     t                 Nx1 time (s)
%     q_cmd             Nx9 logged command [x y theta q1..q6] (m, rad)
%     ee_model          Nx3 end effector the model believed, ik9_fk(q_cmd), world (m)
%     ee_phys           Nx3 flange of the physical arm, world (m)
%     min_link_z        Nx1 lowest point of the physical arm, world z (m): the segments shoulder (frame 1)
%                       to flange (frame 6), 21 points each; the J1 column (frame 0 to 1) is left out
%     min_link_to_base_xy  Nx1 smallest SIGNED horizontal distance of a distal link point to the base
%                       rectangle d x w centred on the base origin, body axes (m): positive outside,
%                       negative inside (depth). Distal = the segments from the elbow (frame 2) to the
%                       flange (frame 6). Left out: the J1 column, the shoulder AND the upper arm
%                       (shoulder to elbow), whose shoulder end sits over the base by construction, so
%                       the unsigned distance would be 0 on every sample and could not fail.
%     flange_xy         Nx1 the same signed distance for the flange alone (m)
%     tool_z            Nx1 vertical (world z) component of the tool axis, unit vector (-): -1 = tool
%                       straight down. AR3: -z6 (the flange is offset along -z5 = -z6, d6 < 0, ar3_fk.m
%                       L22; frames are the model's times Rx(180), header); model: +z6
%     model_min_link_z, model_min_link_to_base_xy, model_flange_xy, model_tool_z  Nx1, the same for the
%                       model's arm
%   Every z depends one-for-one on the mount height (opt.mount, default 0.20 m, ik9_fk.m L16), which is
%   NOT MEASURED; the distances to the base rectangle depend on the mount x, y (default 0, 0), also NOT
%   MEASURED. z = 0 is the plane the sources call the base plate (ik9_fk.m L17-18), the floor
%   (define_constants.m z_work) and the table (define_constants.m arm_home comment).
%     fw_sent           Nx6 firmware command after map and clamp (deg)
%     fw_phys           Nx6 firmware angle the arm is taken to reach (deg)
%     clamp_hit         Nx6 logical, true where the clamp or the range end changed the value
%     q_arcs            Nx6 ARCS angles (deg), NaN when opt.arm is given
%     P_body            3x7xN physical frame origins 0..6, base body frame (m)
%     Pm_body           3x7xN model frame origins 0..6, base body frame (m)
%     arm_sign, arm_offset_deg  1x6 the model -> firmware map used
%     conv_S, conv_C    1x6 the convention-consistent map q_arcs = conv_S.*q_model_deg + conv_C (deg)
%     opt               the options used
%   Flat: no local functions (CLAUDE.md section 5, crash 2).
%
%   CONVENTION-CONSISTENT MAP (derived by hand, checked numerically in tests/test_commanding_check.m).
%   Model DH (ik9_fk.m L12, mass_at.m L142-147): alpha = [90 0 90 -90 90 0] deg, theta_off(2) = -90 deg,
%   d4 = +0.222, d6 = +0.036 m. AR3 DH (ar3_fk.m L20-23): alpha = [-90 0 90 -90 90 0] deg,
%   theta_off = [0 0 -90 0 0 180] deg, d4 = -0.22263, d6 = -0.03625 m. With q_arcs = Sc.*q_model + Cc,
%   every AR3 frame rotation equals the model's times Rx(180 deg): R1a = Rz(q1)Rx(-90) = R1m Rx(180);
%   J2: Rx(180)Rz(q2a) = Rz(-q2a)Rx(180), so q2a = -(q2m - 90) = 90 - q2m (model 90 = horizontal
%   forward = ARCS 0; model 180 = up = ARCS -90); J3: Rz(q3a - 90) after Rx(180) gives q3a = 90 - q3m
%   (model 90 = forearm in line = ARCS 0); J4, J5: q4a = -q4m, q5a = -q5m (the other solution,
%   q4a = 180 - q4m with q5a = q5m, puts the same points in the same places); J6: q6a = 180 - q6m.
%   d4 and d6 change sign because z3 and z5 are reversed, so the link points coincide when the link
%   lengths are equal. Left over: the AR3 has a1 = 64.2 mm (the model 0), d1 169.77 vs 169 mm,
%   d4 222.63 vs 222 mm, d6 36.25 vs 36 mm. J6 does not move any frame origin.
if nargin < 3 || isempty(opt), opt = struct(); end
if ~isfield(opt, 'romefw') || isempty(opt.romefw), opt.romefw = 'rome_dirpin'; end
if ~isfield(opt, 'fwmap')  || isempty(opt.fwmap),  opt.fwmap  = 'as_used';     end
if ~isfield(opt, 'flip')   || isempty(opt.flip),   opt.flip   = 'map';         end
if ~isfield(opt, 'clamp')  || isempty(opt.clamp),  opt.clamp  = true;          end
if ~isfield(opt, 'mount')  || isempty(opt.mount),  opt.mount  = [eye(3) [0; 0; 0.20]; 0 0 0 1]; end
if ~isfield(opt, 'arm'),   opt.arm = [];   end
if ~isfield(opt, 'hold') || isempty(opt.hold), opt.hold = nan(6, 1); end
f = signFlips(:).';                         % 1x6 direction hypothesis per joint (-)
if numel(f) ~= 6 || any(abs(abs(f) - 1) > 0)
    error('commanding_check:flips', 'signFlips must be 6 values of +1 or -1');
end

Sc = [1 -1 -1 -1 -1 -1];                    % 1x6 convention-consistent sign, model -> ARCS (-), header
Cc = [0 90 90  0  0 180];                   % 1x6 convention-consistent offset, model -> ARCS (deg), header
dh_m = [0,pi/2,0.169,0; 0.305,0,0,-pi/2; 0,pi/2,0,0; 0,-pi/2,0.222,0; 0,pi/2,0,0; 0,0,0.036,0];
                                            % 6x4 model DH [a (m) alpha (rad) d (m) theta_off (rad)],
                                            %   copy of ik9_fk.m L12; checked against ik9_fk below
h0_m = 0.20;                                % model height of the arm base above the base origin (m), ik9_fk.m L16

if ~isfile(matFile), error('commanding_check:nofile', 'run file not found: %s', matFile); end
S = load(matFile, 'out', 'arm_sign', 'arm_offset_deg', 'arm_fw_switch', 'arm_fw_other', 'arm_fw_margin_deg', 'd', 'w');
need = {'out', 'arm_sign', 'arm_offset_deg', 'arm_fw_switch', 'arm_fw_other', 'arm_fw_margin_deg', 'd', 'w'};
for k = 1:numel(need)
    if ~isfield(S, need{k}), error('commanding_check:field', '%s not in %s', need{k}, matFile); end
end
ts = S.out.get('q_cmd');                    % timeseries of the 9x1 command
R.t = ts.Time(:);                           % Nx1 time (s)
N = numel(R.t);                             % number of samples
R.q_cmd = reshape(ts.Data, 9, N).';         % Nx9 command (m, rad)
if any(~isfinite(R.q_cmd(:))), error('commanding_check:nonfinite', 'q_cmd has non-finite values'); end

sw = S.arm_fw_switch(:).';  ot = S.arm_fw_other(:).';   % 1x6 firmware range ends (deg)
fw_lo = min(sw, ot);  fw_hi = max(sw, ot);              % 1x6 lower and upper range end (deg)
mg = S.arm_fw_margin_deg;                               % clamp margin (deg)

% romefw map recovered by probing (s_r, c_r: q_arcs = s_r.*fw + c_r)
c_r = romefw_to_arcs_deg(zeros(6, 1), opt.romefw).';  % 1x6 offset (deg)
s_r = zeros(1, 6);                                       % 1x6 sign (-)
for j = 1:6
    ej = zeros(6, 1);  ej(j) = 1;
    s_r(j) = romefw_to_arcs_deg(ej, opt.romefw).' * ej - c_r(j);
end
switch opt.fwmap
    case 'as_used'
        sg = S.arm_sign(:).';  of = S.arm_offset_deg(:).';      % 1x6 sign (-), offset (deg)
    case 'consistent'
        sg = s_r .* Sc;  of = s_r .* (Cc - c_r);                 % fw = s_r.*(q_arcs - c_r), s_r = 1/s_r
    otherwise
        error('commanding_check:fwmap', 'unknown fwmap ''%s''', opt.fwmap);
end
R.arm_sign = sg;  R.arm_offset_deg = of;  R.conv_S = Sc;  R.conv_C = Cc;

qdeg = rad2deg(R.q_cmd(:, 4:9));            % Nx6 model joint angles (deg)
switch opt.flip
    case 'map'
        fw_raw = (f .* sg) .* qdeg + of;    % Nx6 firmware command before the clamp (deg)
        if opt.clamp
            fw_sent = min(max(fw_raw, fw_lo + mg), fw_hi - mg);   % arm_clamp.m L27-29
        else
            fw_sent = fw_raw;
        end
        fw_phys = fw_sent;
        R.clamp_hit = fw_sent ~= fw_raw;
    case 'motion'
        fw_raw = sg .* qdeg + of;
        if opt.clamp
            fw_sent = min(max(fw_raw, fw_lo + mg), fw_hi - mg);
        else
            fw_sent = fw_raw;
        end
        fw0 = fw_sent(1, :);                % 1x6 firmware angle at t = 0, where the arm was parked (deg)
        fw_m = fw0 + f .* (fw_sent - fw0);  % Nx6 reversed motion (deg)
        fw_phys = min(max(fw_m, fw_lo), fw_hi);
        R.clamp_hit = (fw_sent ~= fw_raw) | (fw_phys ~= fw_m);
    otherwise
        error('commanding_check:flip', 'unknown flip ''%s''', opt.flip);
end
hj = find(~isnan(opt.hold(:)));            % joints held at a fixed firmware angle
for j = hj.'
    R.clamp_hit(:, j) = fw_phys(:, j) ~= opt.hold(j);
    fw_phys(:, j) = opt.hold(j);            % held at this firmware angle (deg) for the whole run
end
R.fw_sent = fw_sent;  R.fw_phys = fw_phys;

R.ee_model = zeros(N, 3);  R.ee_phys = zeros(N, 3);
R.min_link_z = zeros(N, 1);  R.min_link_to_base_xy = zeros(N, 1);  R.flange_xy = zeros(N, 1);
R.model_min_link_z = zeros(N, 1);  R.model_min_link_to_base_xy = zeros(N, 1);  R.model_flange_xy = zeros(N, 1);
R.tool_z = zeros(N, 1);  R.model_tool_z = zeros(N, 1);
R.q_arcs = nan(N, 6);  R.P_body = zeros(3, 7, N);  R.Pm_body = zeros(3, 7, N);
hx = S.d / 2;  hy = S.w / 2;                % half sides of the base rectangle, body x and y (m)
e_ik9 = 0;                                  % worst |own model chain - ik9_fk| (m)
for k = 1:N
    q = R.q_cmd(k, :).';                    % 9x1 command (m, rad)
    th = q(3);                              % base yaw (rad)
    Tb = [cos(th) -sin(th) 0 q(1); sin(th) cos(th) 0 q(2); 0 0 1 0; 0 0 0 1];   % 4x4 world <- base body (m)

    % model arm, frames in the body frame
    T = [eye(3) [0; 0; h0_m]; 0 0 0 1];     % 4x4 running transform body <- frame i (m)
    Pm = zeros(3, 7);  Pm(:, 1) = T(1:3, 4);   % 3x7 model frame origins (m)
    for i = 1:6
        ti = q(3+i) + dh_m(i, 4);  al = dh_m(i, 2);   % DH angle and twist (rad)
        T = T * [cos(ti) -sin(ti)*cos(al)  sin(ti)*sin(al) dh_m(i,1)*cos(ti);
                 sin(ti)  cos(ti)*cos(al) -cos(ti)*sin(al) dh_m(i,1)*sin(ti);
                 0        sin(al)          cos(al)         dh_m(i,3);
                 0        0                0               1];
        Pm(:, i+1) = T(1:3, 4);
    end
    pe = ik9_fk(q);                         % 3x1 model end effector, world (m)
    pw = Tb * [Pm(:, 7); 1];
    e_ik9 = max(e_ik9, norm(pw(1:3) - pe));
    R.ee_model(k, :) = pe.';
    R.model_tool_z(k) = T(3, 3);            % model tool axis +z6, its world z component (-)
    R.Pm_body(:, :, k) = Pm;

    % physical arm
    fwk = fw_phys(k, :).';                  % 6x1 firmware angles reached (deg)
    if isempty(opt.arm)
        qa = romefw_to_arcs_deg(fwk, opt.romefw);   % 6x1 ARCS angles (deg)
        [Ta, Pa] = ar3_fk(qa);                       % flange pose, 3x7 frame origins, arm base frame (m)
        R.q_arcs(k, :) = qa.';
        tool = -Ta(1:3, 3);                         % 3x1 AR3 tool axis -z6, arm base frame (-)
    else
        qj = deg2rad(opt.arm.s(:) .* fwk + opt.arm.c(:));   % 6x1 joint angles of the given DH (rad)
        T = eye(4);  Pa = zeros(3, 7);
        for i = 1:6
            ti = qj(i) + opt.arm.dh(i, 4);  al = opt.arm.dh(i, 2);
            T = T * [cos(ti) -sin(ti)*cos(al)  sin(ti)*sin(al) opt.arm.dh(i,1)*cos(ti);
                     sin(ti)  cos(ti)*cos(al) -cos(ti)*sin(al) opt.arm.dh(i,1)*sin(ti);
                     0        sin(al)          cos(al)         opt.arm.dh(i,3);
                     0        0                0               1];
            Pa(:, i+1) = T(1:3, 4);
        end
        ts_ = 1;                                     % tool axis sign for a plain DH arm (-)
        if isfield(opt.arm, 'tool_sign'), ts_ = opt.arm.tool_sign; end
        tool = ts_ * T(1:3, 3);                      % 3x1 tool axis, arm base frame (-)
    end
    tw = opt.mount(1:3, 1:3) * tool;                 % 3x1 tool axis, body frame = world z component (-)
    R.tool_z(k) = tw(3);
    P = opt.mount(1:3, 1:3) * Pa + opt.mount(1:3, 4);   % 3x7 physical frame origins, body frame (m)
    R.P_body(:, :, k) = P;
    pw = Tb * [P(:, 7); 1];
    R.ee_phys(k, :) = pw(1:3).';

    % clearance metrics, physical then model; z of the body frame = world z (planar base at z = 0)
    for pass = 1:2
        if pass == 1, Q = P; else, Q = Pm; end
        [zmin, sdmin, sdf] = link_clearance(Q, hx, hy);   % lowest z (m), smallest signed distance (m), flange's (m); the shared definition
        if pass == 1
            R.min_link_z(k) = zmin;  R.min_link_to_base_xy(k) = sdmin;  R.flange_xy(k) = sdf;
        else
            R.model_min_link_z(k) = zmin;  R.model_min_link_to_base_xy(k) = sdmin;  R.model_flange_xy(k) = sdf;
        end
    end
end
if e_ik9 > 1e-12
    error('commanding_check:ik9', 'own model chain differs from ik9_fk by %.3g m: dh_m or h0_m is stale', e_ik9);
end
R.opt = opt;
end
