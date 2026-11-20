function [T, M] = bench_joint_signs(log, K)
%BENCH_JOINT_SIGNS  Part F item 24 evaluator: jog log -> arm_sign / arm_offset_deg implied, flags, J2/J5/J6 map verdict.
%   [T, M] = bench_joint_signs(log)      constants read by running define_constants (NOT done by the tests)
%   [T, M] = bench_joint_signs(log, K)   K.arm_sign, K.arm_offset_deg (1x6) given explicitly (the tests do this)
%   OFFLINE. Opens no socket, sends nothing to the robot.
%
%   LOG FORMAT (struct array, one element per jog_joint move). NO existing tool saves this: jog_joint.m only
%   prints (L28, L42, L51-52) and returns nothing; testGV.m only prints GV rows (L32). The Plan 3 Task 3
%   recorder must save the raw telemetry lines the jog saw, plus the observer's entries. Fields:
%     j               1..6        joint jogged
%     delta_deg       scalar      commanded firmware jog (deg), +5 in the checklist (jog_joint(j, 5))
%     before_lines    string[]    raw telemetry lines received before the command (as readline gives them)
%     after_lines     string[]    raw telemetry lines after the joint settled. Only "ARM,j1..j6,READY" rows
%                                 are used (rule of ROMECommand.m L156-159 and jog_joint read_arm L64-66);
%                                 the last valid row of each list is the reading, as in jog_joint L35-41.
%                                 The numbers are FIRMWARE degrees (raw, before the arm_sign map).
%     phys_model_dir  +1/-1/0     observer: for the +delta jog, did the arm move in the direction the MODEL
%                                 (ik9_fk) calls increasing joint j? 0 = not observed.
%     phys_arcs_dir   +1/-1/0     observer: did it move in the direction of increasing ARCS angle? 0 = not
%                                 observed. Only J2, J5, J6 are used (map verdict). For J2 the ARCS direction
%                                 is the direction of increasing lean toward the reach (checklist 24b).
%     model_deg_obs   deg or NaN  optional: model angle of joint j before the jog, measured (inclinometer)
%     lean_deg_obs    deg or NaN  optional, J2 only: measured lean of the upper arm from vertical toward the
%                                 reach, before the jog (checklist 24b step 2)
%
%   WHAT IS IMPLIED. define_constants: firmware_deg = arm_sign .* model_deg + arm_offset_deg (L96). A jog of
%   +delta in firmware degrees changes the model angle by delta/arm_sign, so the arm moves in the model's
%   positive direction iff arm_sign * sign(delta) = +1; hence
%       implied arm_sign   = phys_model_dir * sign(delta)
%       implied offset     = fw_before - implied_arm_sign * model_deg_obs        (needs model_deg_obs)
%   The telemetry columns only test that the reading followed the command (|moved - delta| < 2 deg, jog_joint
%   L49) and give its sign; a reading change of 0.3 deg or less counts as "no movement" for the sign column
%   (jog_joint uses its 0.3 deg at L36 as a SETTLE detector, "joint still moving", not as a movement test; the
%   reuse of the number for the sign is my choice). J4 has no encoder (jog_joint L47-48): its
%   telemetry is the command, so it is not judged and only the observer's entry counts.
%
%   STATUS per row: AGREE | DISAGREE (implied sign, or offset beyond 2 deg, differs from K) | NO FOLLOW
%   (reading did not follow the command, J4 excepted) | NO TELEMETRY (no valid ARM row) | NOT OBSERVED (no
%   phys_model_dir). flag = true for DISAGREE, NO FOLLOW and NO TELEMETRY. NOT OBSERVED is not flagged but
%   is not AGREE either: the checklist item is not passed with it.
%
%   MAP VERDICT M (which of romefw_to_arcs_deg's two maps the J2/J5/J6 observations support).
%   M.verdict = 'rome_dirpin' | 'arcs_caldir' | 'neither' | 'inconclusive'. For each observation of J2, J5, J6
%   the predicted ARCS direction of the +delta jog is the slope of romefw_to_arcs_deg(q, map) at the logged
%   firmware posture; it is compared with phys_arcs_dir. Optionally J2 lean_deg_obs is compared with the
%   predicted lean = q2_ARCS + 90 (checklist 24b, "derived, not measured": the +90 is from ar3_fk L15-16 as
%   quoted there; this function does not re-derive it). Lean agreement tolerance 3 deg (my choice, not measured).
%   Every observation must agree with a map for it to be the verdict; mixed observations give 'neither'; an
%   observation consistent with BOTH maps (e.g. lean near fw 67.2, where fw-42 = 92.4-fw) or none at all gives
%   'inconclusive'. Rows with status NO FOLLOW or NO TELEMETRY do not feed the verdict.
%   TRACKER Q-L3: lean = fw-42 (rome_dirpin) vs 92.4-fw (arcs_caldir); at fw J2 = 132, lean 90 vs -39.6.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(fileparts(here), 'analysis'));
if nargin < 2 || isempty(K)
    evalc('define_constants');                       % live values; the offline tests never take this branch
    K.arm_sign = arm_sign;  K.arm_offset_deg = arm_offset_deg; %#ok<NODEF>
end
FOLLOW_TOL = 2.0;      % deg, reading must follow the command within this (jog_joint.m L49)
MOVE_TOL   = 0.3;      % deg, reading change at or below this gives meas_sign 0 (my reuse of jog_joint.m L36's settle-detector value)
OFFSET_TOL = 2.0;      % deg, implied offset may differ from the constant by this (my choice, not measured)
LEAN_TOL   = 3.0;      % deg, lean tolerance for the map verdict (my choice, not measured)
maps = {'rome_dirpin', 'arcs_caldir'};
n = numel(log);
joint = zeros(n,1);  cmd_sign = zeros(n,1);  tel_before = NaN(n,1);  tel_after = NaN(n,1);
tel_delta = NaN(n,1);  meas_sign = NaN(n,1);  followed = true(n,1);  phys_model_dir = zeros(n,1);
implied_sign = NaN(n,1);  define_sign = NaN(n,1);  implied_off = NaN(n,1);  define_off = NaN(n,1);
status = strings(n,1);  flag = false(n,1);
obs = struct('j', {}, 'obs', {}, 'pred', {}, 'lean_obs', {}, 'lean_pred', {});  % map observations
for i = 1:n
    e = log(i);  j = e.j;
    joint(i) = j;  cmd_sign(i) = sign(e.delta_deg);  phys_model_dir(i) = e.phys_model_dir;
    define_sign(i) = K.arm_sign(j);  define_off(i) = K.arm_offset_deg(j);
    fb = bench_parse_telemetry(e.before_lines, 'ARM');
    fa = bench_parse_telemetry(e.after_lines, 'ARM');
    if isempty(fb) || isempty(fa) || any(isnan(fb(end,:))) || any(isnan(fa(end,:)))
        status(i) = "NO TELEMETRY";  flag(i) = true;  continue;
    end
    qb = fb(end,:);  qa = fa(end,:);                 % last valid reading, firmware deg (1x6)
    tel_before(i) = qb(j);  tel_after(i) = qa(j);  tel_delta(i) = qa(j) - qb(j);
    if abs(tel_delta(i)) > MOVE_TOL, meas_sign(i) = sign(tel_delta(i)); else, meas_sign(i) = 0; end
    if j ~= 4, followed(i) = abs(tel_delta(i) - e.delta_deg) < FOLLOW_TOL; end
    if e.phys_model_dir ~= 0, implied_sign(i) = e.phys_model_dir * cmd_sign(i); end
    if isfinite(e.model_deg_obs) && ~isnan(implied_sign(i))
        implied_off(i) = qb(j) - implied_sign(i) * e.model_deg_obs;
    end
    if ~followed(i)
        status(i) = "NO FOLLOW";  flag(i) = true;
    elseif isnan(implied_sign(i))
        status(i) = "NOT OBSERVED";
    elseif implied_sign(i) ~= define_sign(i) || (~isnan(implied_off(i)) && abs(implied_off(i) - define_off(i)) > OFFSET_TOL)
        status(i) = "DISAGREE";  flag(i) = true;
    else
        status(i) = "AGREE";
    end
    % map observations: J2, J5, J6 only
    if followed(i) && any(j == [2 5 6]) && (e.phys_arcs_dir ~= 0 || (j == 2 && isfinite(e.lean_deg_obs)))
        q0 = qb(:);  q1 = q0;  q1(j) = q1(j) + e.delta_deg;
        pred = zeros(1,2);  lp = NaN(1,2);
        for m = 1:2
            a0 = romefw_to_arcs_deg(q0, maps{m});  a1 = romefw_to_arcs_deg(q1, maps{m});
            pred(m) = sign(a1(j) - a0(j));           % predicted ARCS direction of the +delta jog
            if j == 2, lp(m) = a0(2) + 90; end       % predicted lean before the jog (deg), checklist 24b
        end
        obs(end+1) = struct('j', j, 'obs', e.phys_arcs_dir, 'pred', pred, ...
                            'lean_obs', e.lean_deg_obs, 'lean_pred', lp); %#ok<AGROW>
    end
end
T = table(joint, cmd_sign, tel_before, tel_after, tel_delta, meas_sign, followed, phys_model_dir, ...
          implied_sign, define_sign, implied_off, define_off, status, flag, ...
          'VariableNames', {'joint','cmd_sign','tel_before','tel_after','tel_delta','meas_sign','followed', ...
          'phys_model_dir','implied_arm_sign','define_arm_sign','implied_offset_deg','define_offset_deg','status','flag'});
% ---- map verdict
agree = zeros(1,2);  total = 0;                      % observations agreeing with each map, observations used
for k = 1:numel(obs)
    o = obs(k);
    if o.obs ~= 0
        total = total + 1;  agree = agree + double(o.pred == o.obs);
    end
    if o.j == 2 && isfinite(o.lean_obs)
        total = total + 1;  agree = agree + double(abs(o.lean_pred - o.lean_obs) < LEAN_TOL);
    end
end
if total == 0 || (agree(1) == total && agree(2) == total), verdict = 'inconclusive';   % nothing, or consistent with both
elseif agree(1) == total && agree(2) < total, verdict = 'rome_dirpin';
elseif agree(2) == total && agree(1) < total, verdict = 'arcs_caldir';
else, verdict = 'neither';
end
M = struct('verdict', verdict, 'n_observations', total, 'agree_rome_dirpin', agree(1), ...
           'agree_arcs_caldir', agree(2), 'observations', obs);
end
