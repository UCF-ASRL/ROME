function test_base_plant()
%TEST_BASE_PLANT  Unit tests of base_plant_step.m, the firmware + motor wheel plant (Plan 2 Task 4a).
%   Pure function tests: no model, no define_constants, no network. Flat function without local functions
%   (CLAUDE.md section 5, crash 2). Errors test_base_plant:fail, message "FAIL: <checks>", if any check fails.
%   Firmware source: ucf-asrl/ROME/UK-Arduino/ROME_Mega_Code/ROME_Mega_Code.ino ("ino"); Lnn = its line nn.
%   Checks:
%     (i)   firmware fidelity: a 3-step, 4-wheel sequence computed by hand below from the ino equations, written
%           out step by step with line citations, matches base_plant_step to 1e-12 (pwm and dir exactly).
%           The wheels cover: wheel 1 a command dropped to 0, so at step 2 the direction flips to reverse while
%           the wheel still turns forward, and at step 3 the encoder reports that forward motion with the
%           commanded (reverse) sign (L283-284); wheel 2 a reversal whose speed crosses zero inside step 2
%           (the encoder counts |w|); wheel 3 saturating (150 rpm, PWM clamp L250); wheel 4 reverse.
%           Anchor literals (pwm, measured rpm, correct) were worked out from the same equations by
%           ROME_merged/tools/p2t4a_margins.py (Python), output docs/verification/2026-11-19_p2t4a_margins.log,
%           which also shows the closest floor() argument to an integer: 0.1222 (PWM) and 0.0893 (encoder
%           edges), so round-off cannot flip a count.
%     (ii)  cap: 150 rpm on every wheel (signs + - + -) with motor_gain 0.8 for 400 steps. The speed never
%           exceeds fw_max_rpm*motor_gain = 96 rpm, ends within 1e-9 of +/-96, the PWM ends at 255, and the
%           integral ends clamped at exactly +/-INTEGRAL_MAX = 500 (L219).
%     (iii) lag: Kp = Ki = 0, commands 96, 50, -75, 0 rpm. The speed after k steps equals the closed form
%           T*(1 - exp(-k*dt/tau)) to 1e-12 for k = 1..60, with T taken through the (int) cast of L249
%           (PWM 204, 106, 159, 0). A forward-Euler lag would give pole 1 - dt/tau instead of exp(-dt/tau).
%     (p)   pass-through: with P.ideal = 1, w_act equals w_cmd exactly (needed by Task 4b test iv).
here = fileparts(mfilename('fullpath'));       % ROME_merged/tests (char)
addpath(fileparts(here));                      % ROME_merged, which holds base_plant_step.m
fails = {};                                    % names of the failed checks (cell of char)
OKS = {'--', 'OK'};                            % printed verdict, indexed by 1 + pass (cell of char)
fprintf('test_base_plant: base_plant_step resolves to %s\n', which('base_plant_step'));

% ---- firmware constants, read from the ino ------------------------------------------------------------
MAX_RPM = 120;         % L8  MAX_RPM, rpm at PWM 255 (rpm)
DT_PID = 0.050;        % L9  PID loop period (s)
PPR = 12;              % L6  PULSES_PER_REVOLUTION (counts per motor revolution)
GR = 50;               % L7  GEAR_RATIO (-)
Kp = 0.190;            % L33 (-)
Ki = 0.125;            % L34 (1/s)
INT_MAX = 500.0;       % L35 INTEGRAL_MAX (rpm*s)
q = 60 / (DT_PID * PPR * GR);  % rpm per counted edge in one DT_PID, from L276-281: = 2 (rpm per count).
                       % In double it is 1.9999999999999998 = 2 - 2^-52 (0.05*12*50 rounds up to
                       % 30.000000000000004; 2026-11-19_p2t4a_margins.log line 3), so a measured rpm counts*q
                       % is not an exact integer: the anchor checks on w_meas use tol 1e-12, not 0 (first GREEN
                       % run, docs/verification/2026-11-19_p2t4a_green.log: diffs 3.6e-15, 7.1e-15). One
                       % encoder edge moves w_meas by 2 rpm, so 1e-12 cannot hide a count error.
tau = 0.10;            % motor time constant for (i) and (ii), UNMEASURED, a test value (s)
gain = 0.9;            % motor gain for (i), UNMEASURED, a test value (-)

P = struct('fw_Kp', Kp, 'fw_Ki', Ki, 'fw_int_max', INT_MAX, 'fw_dt', DT_PID, 'fw_max_rpm', MAX_RPM, ...
    'enc_rpm_quant', q, 'motor_tau', tau, 'motor_gain', gain, 'ideal', 0);  % parameters for (i)
z = zeros(4, 1);       % 4x1 zeros
st0 = struct('e_int', z, 'w_mot', z, 'w_avg', z, 'w_absavg', z, 'enc_frac', z, 'w_meas', z, ...
    'dir', ones(4, 1), 'correct', z, 'pwm', z);  % at rest after setup(): integralError 0 (L32), actualRPMs 0
                                                 % (L24), motorDirection 1 (L26), correctRPMs 0 (L28)

%% ======================= (i) firmware fidelity, 3 steps by hand ==========================================
c1 = [40; 60; 150; -30];   % desiredRPMs at step 1 (rpm)
c2 = [0; -60; 150; -30];   % step 2 (rpm)
c3 = [0; -60; 150; -30];   % step 3 (rpm)
a = exp(-DT_PID / tau);    % exact pole of the motor dw/dt = (T - w)/tau over one step (-)

% ---- step 1. Start state: everything 0, motorDirection +1 (L26).
h1_edges = z + z / q;                 % no motion in the previous interval, 4x1 (counts)
h1_counts = floor(h1_edges);          % L267 pulseCount (counts)
h1_frac = h1_edges - h1_counts;       % carried fraction (counts)
h1_meas = [1; 1; 1; 1] .* h1_counts * q;      % L276-284: counts/DT_PID*60/(PPR*GR), times motorDirection (rpm)
h1_e = c1 - h1_meas;                  % L215 (rpm)
h1_int = 0 + h1_e * DT_PID;           % L216 (rpm*s)
h1_int = min(max(h1_int, -INT_MAX), INT_MAX);  % L219 (rpm*s)
h1_corr = c1 + Kp * h1_e + Ki * h1_int;        % L221-222 (rpm)
h1_dir = ones(4, 1);  h1_dir(h1_corr < 0) = -1;  % L228 (-)
h1_pwm = fix(abs(h1_corr) / MAX_RPM * 255);      % L248-249, (int) truncates toward 0 (PWM counts)
h1_pwm(h1_pwm > 255) = 255;  h1_pwm(h1_pwm < 0) = 0;  % L250 (PWM counts)
h1_T = gain * h1_pwm / 255 * MAX_RPM .* h1_dir;  % motor target (rpm)
h1_w = h1_T + (0 - h1_T) * a;                    % speed at end of step 1 (rpm)
h1_iw = h1_T * DT_PID + (0 - h1_T) * tau * (1 - a);  % integral of w over step 1 (rpm*s)
h1_absavg = abs(h1_iw) / DT_PID;      % no wheel starts moving and crosses 0 from rest (rpm)
% anchors: correct = 1.19625 * cmd; 47.85/120*255 = 101.68 -> 101; 71.775 -> 152.52 -> 152; 179.4375 -> 381.3
% -> 255 (L250); 35.8875 -> 76.26 -> 76
A1_corr = [47.85; 71.775; 179.4375; -35.8875];  A1_pwm = [101; 152; 255; 76];  A1_meas = z;

% ---- step 2
h2_edges = h1_frac + h1_absavg / q;   % (counts)
h2_counts = floor(h2_edges);          % (counts)
h2_frac = h2_edges - h2_counts;       % (counts)
h2_meas = h1_dir .* h2_counts * q;    % L283-284 (rpm)
h2_e = c2 - h2_meas;                  % L215 (rpm)
h2_int = h1_int + h2_e * DT_PID;      % L216 (rpm*s)
h2_int = min(max(h2_int, -INT_MAX), INT_MAX);  % L219 (rpm*s)
h2_corr = c2 + Kp * h2_e + Ki * h2_int;        % L221-222 (rpm)
h2_dir = ones(4, 1);  h2_dir(h2_corr < 0) = -1;  % L228 (-)
h2_pwm = fix(abs(h2_corr) / MAX_RPM * 255);      % L249 (PWM counts)
h2_pwm(h2_pwm > 255) = 255;  h2_pwm(h2_pwm < 0) = 0;  % L250 (PWM counts)
h2_T = gain * h2_pwm / 255 * MAX_RPM .* h2_dir;  % (rpm)
h2_w = h2_T + (h1_w - h2_T) * a;                 % (rpm)
h2_iw = h2_T * DT_PID + (h1_w - h2_T) * tau * (1 - a);  % (rpm*s)
h2_tc = tau * log((h2_T - h1_w) ./ h2_T);        % zero-crossing time where the signs differ (s)
h2_cross = (h1_w .* h2_T < 0) & (h2_tc < DT_PID); % crossing inside step 2 (logical)
if ~isequal(h2_cross, [false; true; false; false])
    error('test_base_plant:setup', 'hand premise broken: only wheel 2 should cross zero in step 2, got %s', mat2str(h2_cross'));
end
h2_i1 = h2_T(2) * h2_tc(2) + (h1_w(2) - h2_T(2)) * tau * (1 - exp(-h2_tc(2) / tau));  % wheel 2, 0..tc (rpm*s)
h2_iabs = abs(h2_iw);  h2_iabs(2) = abs(h2_i1) + abs(h2_iw(2) - h2_i1);  % integral of |w| (rpm*s)
h2_absavg = h2_iabs / DT_PID;         % (rpm)
% independent check of the split: adaptive quadrature of |w(t)| for wheel 2 over step 2
h2_iabs_quad = integral(@(t) abs(h2_T(2) + (h1_w(2) - h2_T(2)) * exp(-t / tau)), 0, DT_PID, ...
    'AbsTol', 1e-14, 'RelTol', 1e-13);  % (rpm*s)
A2_meas = [8; 12; 22; -6];  A2_pwm = [2; 156; 255; 74];  % anchors: counts 4, 6, 11, 3 (times dir, times 2)

% ---- step 3
h3_edges = h2_frac + h2_absavg / q;   % (counts)
h3_counts = floor(h3_edges);          % (counts)
h3_frac = h3_edges - h3_counts;       % (counts)
h3_meas = h2_dir .* h3_counts * q;    % L283-284 (rpm); wheel 1: dir -1 while turning forward
h3_e = c3 - h3_meas;                  % L215 (rpm)
h3_int = h2_int + h3_e * DT_PID;      % L216 (rpm*s)
h3_int = min(max(h3_int, -INT_MAX), INT_MAX);  % L219 (rpm*s)
h3_corr = c3 + Kp * h3_e + Ki * h3_int;        % L221-222 (rpm)
h3_dir = ones(4, 1);  h3_dir(h3_corr < 0) = -1;  % L228 (-)
h3_pwm = fix(abs(h3_corr) / MAX_RPM * 255);      % L249 (PWM counts)
h3_pwm(h3_pwm > 255) = 255;  h3_pwm(h3_pwm < 0) = 0;  % L250 (PWM counts)
h3_T = gain * h3_pwm / 255 * MAX_RPM .* h3_dir;  % (rpm)
h3_w = h3_T + (h2_w - h3_T) * a;                 % (rpm)
h3_iw = h3_T * DT_PID + (h2_w - h3_T) * tau * (1 - a);  % (rpm*s)
h3_tc = tau * log((h3_T - h2_w) ./ h3_T);        % (s)
if any((h2_w .* h3_T < 0) & (h3_tc < DT_PID))
    error('test_base_plant:setup', 'hand premise broken: no wheel should cross zero in step 3');
end
h3_absavg = abs(h3_iw) / DT_PID;      % (rpm)
A3_meas = [-14; -10; 56; -16];  A3_pwm = [6; 148; 255; 70];  % anchors; wheel 1 -14 = -1 * 7 counts * 2

% ---- run base_plant_step and compare
[g1, s1] = base_plant_step(c1, st0, P);   % step 1 output (rpm), state
[g2, s2] = base_plant_step(c2, s1, P);    % step 2
[g3, s3] = base_plant_step(c3, s2, P);    % step 3
tol = 1e-12;                              % (rpm, rpm*s, counts)
C = { ...   % name, got, want, tolerance (0 = exact)
  'i step1 w_act', g1, h1_w, tol;  'i step1 w_meas', s1.w_meas, h1_meas, tol;  'i step1 e_int', s1.e_int, h1_int, tol;
  'i step1 correct', s1.correct, h1_corr, tol;  'i step1 pwm', s1.pwm, h1_pwm, 0;  'i step1 dir', s1.dir, h1_dir, 0;
  'i step1 w_avg', s1.w_avg, h1_iw / DT_PID, tol;  'i step1 w_absavg', s1.w_absavg, h1_absavg, tol;
  'i step1 enc_frac', s1.enc_frac, h1_frac, tol;
  'i step1 anchor correct', s1.correct, A1_corr, tol;  'i step1 anchor pwm', s1.pwm, A1_pwm, 0;
  'i step1 anchor meas', s1.w_meas, A1_meas, tol;
  'i step2 w_act', g2, h2_w, tol;  'i step2 w_meas', s2.w_meas, h2_meas, tol;  'i step2 e_int', s2.e_int, h2_int, tol;
  'i step2 correct', s2.correct, h2_corr, tol;  'i step2 pwm', s2.pwm, h2_pwm, 0;  'i step2 dir', s2.dir, h2_dir, 0;
  'i step2 w_avg', s2.w_avg, h2_iw / DT_PID, tol;  'i step2 w_absavg', s2.w_absavg, h2_absavg, tol;
  'i step2 enc_frac', s2.enc_frac, h2_frac, tol;
  'i step2 anchor meas', s2.w_meas, A2_meas, tol;  'i step2 anchor pwm', s2.pwm, A2_pwm, 0;
  'i step2 wheel-2 |w| split vs quadrature', h2_iabs(2), h2_iabs_quad, 1e-9;
  'i step3 w_act', g3, h3_w, tol;  'i step3 w_meas', s3.w_meas, h3_meas, tol;  'i step3 e_int', s3.e_int, h3_int, tol;
  'i step3 correct', s3.correct, h3_corr, tol;  'i step3 pwm', s3.pwm, h3_pwm, 0;  'i step3 dir', s3.dir, h3_dir, 0;
  'i step3 w_avg', s3.w_avg, h3_iw / DT_PID, tol;  'i step3 w_absavg', s3.w_absavg, h3_absavg, tol;
  'i step3 enc_frac', s3.enc_frac, h3_frac, tol;
  'i step3 anchor meas', s3.w_meas, A3_meas, tol;  'i step3 anchor pwm', s3.pwm, A3_pwm, 0};
for r = 1:size(C, 1)
    d = max(abs(C{r,2}(:) - C{r,3}(:)));          % largest difference (units of the quantity)
    ok = isequal(size(C{r,2}), size(C{r,3})) && d <= C{r,4};
    fprintf('  [%s] (%s) max diff %.3e (tol %.0e)\n', OKS{1 + double(ok)}, C{r,1}, d, C{r,4});
    if ~ok, fails{end+1} = ['(' C{r,1} ')']; end %#ok<AGROW>
end
fprintf('  step 3 wheel 1: true mean speed %+.4f rpm over step 2, reported %+.1f rpm (sign echo L283-284)\n', ...
    h2_iw(1) / DT_PID, s3.w_meas(1));

%% ======================= (ii) cap ========================================================================
P2 = P;  P2.motor_gain = 0.8;             % test gain (-)
cap = MAX_RPM * P2.motor_gain;            % fw_max_rpm * motor_gain = 96 (rpm)
sgn = [1; -1; 1; -1];                     % command signs (-)
st = st0;  wmax = 0;  w = z;              % state, largest |w| seen (rpm), last output (rpm)
for k = 1:400
    [w, st] = base_plant_step(150 * sgn, st, P2);
    wmax = max(wmax, max(abs(w)));
end
C2 = {'ii never above cap', wmax <= cap + 1e-12, sprintf('max |w| %.12f, cap %.1f', wmax, cap);
      'ii settles at cap', max(abs(w - cap * sgn)) < 1e-9, sprintf('max |w - 96*sgn| %.3e', max(abs(w - cap * sgn)));
      'ii pwm 255', isequal(st.pwm, 255 * ones(4, 1)), sprintf('pwm %s', mat2str(st.pwm'));
      'ii integral clamped at 500 (L219)', isequal(st.e_int, INT_MAX * sgn), sprintf('e_int %s', mat2str(st.e_int', 15))};
for r = 1:size(C2, 1)
    fprintf('  [%s] (%s) %s\n', OKS{1 + double(C2{r,2})}, C2{r,1}, C2{r,3});
    if ~C2{r,2}, fails{end+1} = ['(' C2{r,1} ')']; end %#ok<AGROW>
end

%% ======================= (iii) lag, PI off ===============================================================
P3 = P;  P3.fw_Kp = 0;  P3.fw_Ki = 0;  P3.motor_tau = 0.13;  P3.motor_gain = 0.95;  % test values (-, 1/s, s, -)
c = [96; 50; -75; 0];                    % commands (rpm)
pwm_hand = [204; 106; 159; 0];           % L249: 96/120*255 = 204, 50 -> 106.25 -> 106, 75 -> 159.375 -> 159
dir_hand = [1; 1; -1; 1];                % L228, 0 -> +1 (-)
T = P3.motor_gain * pwm_hand / 255 * MAX_RPM .* dir_hand;  % target (rpm)
st = st0;  err3 = 0;  pwm_ok = true;     % state, largest error (rpm), PWM matched every step
for k = 1:60
    [w, st] = base_plant_step(c, st, P3);
    w_cf = T * (1 - exp(-k * DT_PID / P3.motor_tau));   % analytic discrete first-order response (rpm)
    err3 = max(err3, max(abs(w - w_cf)));
    pwm_ok = pwm_ok && isequal(st.pwm, pwm_hand) && isequal(st.dir, dir_hand);
end
C3 = {'iii closed form 1e-12', err3 <= 1e-12, sprintf('max |w - T(1-exp(-k dt/tau))| = %.3e rpm over 60 steps', err3);
      'iii pwm and dir', pwm_ok, sprintf('pwm %s dir %s', mat2str(st.pwm'), mat2str(st.dir'))};
for r = 1:size(C3, 1)
    fprintf('  [%s] (%s) %s\n', OKS{1 + double(C3{r,2})}, C3{r,1}, C3{r,3});
    if ~C3{r,2}, fails{end+1} = ['(' C3{r,1} ')']; end %#ok<AGROW>
end

%% ======================= (p) pass-through ================================================================
P4 = P;  P4.ideal = 1;                   % pass-through mode
cp = [12.3; -45.6; 0; 150];              % commands (rpm)
[wp, ~] = base_plant_step(cp, st0, P4);  % (rpm)
okp = isequal(wp, cp);
fprintf('  [%s] (p ideal pass-through) w_act %s\n', OKS{1 + double(okp)}, mat2str(wp'));
if ~okp, fails{end+1} = '(p ideal pass-through)'; end

if ~isempty(fails)
    error('test_base_plant:fail', 'FAIL: %d check(s): %s', numel(fails), strjoin(fails, ', '));
end
fprintf('test_base_plant: PASS (i) %d comparisons, (ii) 4, (iii) 2, (p) 1\n', size(C, 1));
end
