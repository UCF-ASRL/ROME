function [T, info] = bench_wheel_order(log)
%BENCH_WHEEL_ORDER  Part F item 26 evaluator: wheel k commanded alone -> which telemetry channel responds.
%   [T, info] = bench_wheel_order(log)     OFFLINE. Opens no socket, sends nothing to the robot.
%
%   WHAT THIS CAN AND CANNOT CHECK. It checks the CHANNEL INDEX only: that when wheel k is commanded, GV
%   telemetry channel k (and only k) is non-zero. It CANNOT check wheel DIRECTION. Each Mega encoder is a single
%   pin counting pulses (ROME_Mega_Code.ino L58-61, ISR pulseCount[i]++), and the reported rpm is
%   actualRPMs[i] = motorDirection[i] * rpm (L283-284): the sign in "actN" is the commanded sign echoed back, not
%   a measurement. Direction of rotation is judged BY EYE (checklist item 26: bottom of the wheel, in x_B/y_B).
%   The echo sign is still tested here, but only as an anomaly check (the echo must match the command).
%   Wheel numbering: thesis Fig. 6 and Table 1, alpha counter-clockwise from +x_B, ascending 45/135/225/315
%   (CLAUDE.md section 15, 2b); drive direction of wheel k is [-sin a, cos a], so the BOTTOM of the wheel for
%   +rpm moves along [sin a, -cos a] (checklist 26 predicted table). The Mega/telemetry order is
%   channel k = wheel k (Mega assigns the first four ROME values in order, MEGA L160-170; ESP L190-194). Which
%   physical wire belongs to which motor is NOT VERIFIED; that is what this test measures.
%
%   LOG FORMAT (struct array, one element per trial). NO existing tool saves this: testGV.m commands all four
%   wheels at once and only prints GV rows (L24, L32). The Plan 3 Task 3 recorder (or a per-wheel variant of
%   testGV) must save the raw lines. Fields:
%     k_cmd          1..4        wheel commanded (the others 0.0)
%     rpm_cmd        scalar      commanded rpm (15.0 or -15.0 in the checklist)
%     gv_lines       string[]    raw telemetry lines during the run, "GV,act1..act4,cmd1..cmd4"; parsed with the
%                                rule of ROMECommand.m L149-152 (>= 9 fields). The LAST HALF of the valid rows is
%                                used (ramp-up excluded; arbitrary choice, median taken).
%     eye_wheel      0..4        optional: wheel seen turning by eye (0 = not recorded)
%     eye_bottom_xy  1x2 or NaN  optional: direction the bottom of the turning wheel moved, in (x_B, y_B)
%   A channel is "active" when its median |act| over the window exceeds 30 percent of |rpm_cmd| (my choice).
%   Row is flagged when: no valid GV row (NO TELEMETRY); the active set is not exactly {k_cmd} (WRONG CHANNEL,
%   incl. cross-talk and silence); the echoed sign differs from the command; the eye wheel differs from k_cmd;
%   or the eye bottom direction has a negative dot product with the predicted one (sign(rpm)*[sin a, -cos a]).
ALPHA = [45 135 225 315];       % deg, wheel angles from +x_B, thesis Table 1 / Fig. 6
FRAC = 0.30;                    % fraction of |rpm_cmd| above which a channel counts as turning
n = numel(log);
k_cmd = zeros(n,1);  rpm_cmd = zeros(n,1);  tel_active = strings(n,1);  tel_channel = NaN(n,1);
index_ok = false(n,1);  echo_sign_ok = false(n,1);  eye_ok = true(n,1);  eye_bottom_ok = true(n,1);
status = strings(n,1);  flag = false(n,1);
for i = 1:n
    e = log(i);  k_cmd(i) = e.k_cmd;  rpm_cmd(i) = e.rpm_cmd;
    gv = bench_parse_telemetry(e.gv_lines, 'GV');
    gv = gv(~any(isnan(gv), 2), :);
    if isempty(gv)
        status(i) = "NO TELEMETRY";  flag(i) = true;  continue;
    end
    w = gv(ceil(end/2):end, :);                      % last half of the rows (rpm), Mx4
    med = median(w, 1);                              % 1x4 median act per channel (rpm)
    act = find(abs(med) > FRAC * abs(e.rpm_cmd));    % active channels
    tel_active(i) = strjoin(string(act), ",");
    if isscalar(act), tel_channel(i) = act; end
    index_ok(i) = isequal(act, e.k_cmd);
    echo_sign_ok(i) = index_ok(i) && sign(med(e.k_cmd)) == sign(e.rpm_cmd);
    if isfield(e, 'eye_wheel') && ~isempty(e.eye_wheel) && e.eye_wheel ~= 0
        eye_ok(i) = e.eye_wheel == e.k_cmd;
    end
    if isfield(e, 'eye_bottom_xy') && numel(e.eye_bottom_xy) == 2 && all(isfinite(e.eye_bottom_xy))
        a = ALPHA(e.k_cmd);
        pred = sign(e.rpm_cmd) * [sind(a), -cosd(a)];  % bottom direction predicted for this command
        eye_bottom_ok(i) = dot(pred, e.eye_bottom_xy(:).') > 0;
    end
    if ~index_ok(i), status(i) = "WRONG CHANNEL";
    elseif ~echo_sign_ok(i), status(i) = "ECHO SIGN";
    elseif ~eye_ok(i), status(i) = "EYE WHEEL";
    elseif ~eye_bottom_ok(i), status(i) = "EYE DIRECTION";
    else, status(i) = "OK";
    end
    flag(i) = status(i) ~= "OK";
end
T = table(k_cmd, rpm_cmd, tel_active, tel_channel, index_ok, echo_sign_ok, eye_ok, eye_bottom_ok, status, flag, ...
          'VariableNames', {'k_cmd','rpm_cmd','tel_active','tel_channel','index_ok','echo_sign_ok','eye_ok','eye_bottom_ok','status','flag'});
info = struct('alpha_deg', ALPHA, 'direction_from_telemetry', false, ...
              'note', 'channel index only; wheel direction is judged by eye (single-pin encoders, echoed sign)');
end
