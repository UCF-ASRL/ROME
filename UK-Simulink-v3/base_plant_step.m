function [w_act, st] = base_plant_step(w_cmd, st, P)
%#codegen
%BASE_PLANT_STEP  One step of the ROME base wheel plant: Mega firmware PI loop, PWM cap, first-order motor.
%   [w_act, st] = base_plant_step(w_cmd, st, P)          (Plan 2 Task 4a. Wiring into the .slx is Task 4b.)
%
%   SOURCE (read-only). "ino" below = ucf-asrl/ROME/UK-Arduino/ROME_Mega_Code/ROME_Mega_Code.ino, and
%   ROME.cpp / ROME.h = its motor library in the same folder. Every Lnn below is a line of the ino.
%   Units: the units convention is stated once in code/rome_9dof_model.m. Here every speed is rpm of the
%   wheel (output shaft, after the 50:1 gear, as the firmware reports it, L279-281), times are s.
%
%   TIMING. One call = one pass of the firmware's 20 Hz block, L95-110, with started = true (L100), followed
%   by the motor running for P.fw_dt seconds under the PWM that pass wrote. The caller must call it once every
%   P.fw_dt. In the merged model dt_9dof = 0.05 s (define_constants.m:174) = DT_PID (L9), so one model step
%   = one firmware pass.
%
%   ORDER (mirrors L96-102: calculateSpeed for all wheels, then applyPID, then applyWheelRPM):
%     1. measure   calculateSpeed L255-285: encoder edges counted over the previous interval -> rpm,
%                  signed with the direction the firmware last COMMANDED (L283-284), not the true sign.
%     2. PI        applyPID L215-221 on e = cmd - measured, integral clamped to +/-INTEGRAL_MAX (L219).
%     3. correct   L222: correct = cmd + Kp*e + Ki*int(e).
%     4. PWM       applyWheelRPM L228-229 (direction = sign(correct), 0 counts as forward) and
%                  setMotorSpeedRPM L248-250: pwm = (int)(|correct|/MAX_RPM*255), constrained to [0, 255].
%     5. motor     NOT firmware: a first-order lag, over P.fw_dt, toward
%                  motor_gain * pwm/255 * MAX_RPM * direction. motor_tau and motor_gain are UNMEASURED
%                  (to be fitted in Plan 3 from logged wheel_meas).
%
%   INPUTS
%     w_cmd  4x1 commanded wheel speed after CommandGuard = desiredRPMs (L170) (rpm)
%     st     state struct, fields below (carried from the previous call)
%     P      parameter struct, fields below
%   OUTPUTS
%     w_act  4x1 actual wheel speed at the END of this step (rpm) = st.w_mot
%     st     updated state
%
%   STATE st (all 4x1 double). Initial values for a robot at rest after setup() (L66-88):
%     e_int    integralError (L32, L216-219), (rpm*s). Initial 0 (L32).
%     w_mot    true wheel speed at the end of the last step, signed (rpm). Initial 0.
%     w_avg    signed mean wheel speed over the last step, (1/dt) * integral of w dt (rpm). Initial 0.
%              Not read by the firmware model; kept for Task 4b (base displacement over the step).
%     w_absavg mean |wheel speed| over the last step, (1/dt) * integral of |w| dt (rpm). Initial 0.
%              This is what a single-channel encoder counts (edges come in both directions, L58-61, L82-85).
%     enc_frac fractional encoder edge left over after the last count (counts, in [0,1)). Initial 0.
%              Only used when P.enc_rpm_quant > 0.
%     w_meas   actualRPMs, the measured speed (L24, L283-284) (rpm). Initial 0 (L24).
%     dir      motorDirection, +1 forward / -1 reverse, from the last applyWheelRPM (L26, L228-229) (-).
%              Initial +1 (L26).
%     correct  correctRPMs, the PI output sent to applyWheelRPM (L28, L222) (rpm). Initial 0 (L28).
%     pwm      PWM duty written to the driver, integer 0..255 (L249-252) (PWM counts). Initial 0.
%
%   PARAMETERS P (scalars)
%     fw_Kp         Kp, L33 (rpm per rpm = -)           fw_Ki       Ki, L34 (1/s)
%     fw_int_max    INTEGRAL_MAX, L35 (rpm*s)           fw_dt       DT_PID, L9 (s)
%     fw_max_rpm    MAX_RPM, L8 (rpm at PWM 255)
%     enc_rpm_quant rpm per encoder edge in one fw_dt interval, 60/(fw_dt*PULSES_PER_REVOLUTION*GEAR_RATIO)
%                   = 60/(0.05*12*50) = 2 rpm from L6, L7, L9, L276-281 (rpm per count). 0 = no quantisation.
%     motor_tau     motor time constant (s), UNMEASURED. 0 is allowed (instant motor).
%     motor_gain    rpm reached per rpm-equivalent PWM (-), UNMEASURED.
%     ideal         0 = firmware + motor (the model above). Nonzero = exact pass-through, w_act = w_cmd,
%                   for the plumbing test (Task 4b, test iv). It is needed because tau -> 0 alone is not a
%                   pass-through: the (int) cast at L249 truncates the target by up to one PWM count,
%                   fw_max_rpm/255 = 0.47 rpm.
%
%   MODELLED from the firmware: the order above; the integral clamp (L219); the (int) truncation (L249) and
%   [0,255] clamp (L250); direction from sign(correct) with 0 -> forward (L228); the encoder reading that
%   echoes the commanded direction (L283-284), so a wheel still spinning the old way after a reversal is
%   reported with the new sign; the encoder counting |w| (single channel, CHANGE interrupt, L82-85), with an
%   optional integer-edge quantisation that carries the fractional edge to the next interval.
%   NOT MODELLED, with the reason:
%     - disableMotor on a direction change (L231-233). ROME.cpp:30-32 writes PWM 0 and the same
%       applyWheelRPM call then writes the new PWM (L235 -> ROME.cpp:19), so PWM is 0 only between two
%       consecutive analogWrite calls (microseconds, NOT measured), far below the 50 ms step.
%     - Relay switching time on a direction change (motorForward/motorReverse, ROME.cpp:22-28): UNMEASURED.
%     - started = false, STOP_ALL and the 250 ms watchdog (L100-107, L143-153, L180-190): the plant assumes a
%       started board receiving a command every step.
%     - The L261 timer: the firmware updates actualRPMs only when >= 50 ms of micros() have elapsed and divides
%       by the real elapsed time (L273-274). Here every interval is exactly fw_dt.
%     - Arithmetic precision: the firmware is AVR code, where float and (with avr-gcc's default) double are
%       IEEE single, about 6e-8 relative rounding; e.g. float Kp = 0.190 is stored as 0.1899999976. This
%       function computes in double. That avr-gcc built this sketch with 32-bit double is NOT VERIFIED.
%     - Encoder counts per wheel revolution: L6 says 12 pulses per motor revolution, ROME.h:8 says 6 (unused
%       by the ino), and the interrupt counts both edges (CHANGE, L82-85). Which is physically right is
%       NOT VERIFIED; enc_rpm_quant follows the ino's own formula (L276-281).

% ---- pass-through mode (P.ideal), see the header -------------------------------------------------------
if P.ideal ~= 0
    w_act = w_cmd;                                   % actual = commanded (rpm)
    st.w_mot = w_cmd;                                % true speed (rpm)
    st.w_avg = w_cmd;                                % signed mean over the step (rpm)
    st.w_absavg = abs(w_cmd);                        % mean |speed| over the step (rpm)
    st.w_meas = w_cmd;                               % measured speed, exact in this mode (rpm)
    st.dir = 2 * double(w_cmd >= 0) - 1;             % direction, 0 -> forward as L228 (-)
    st.correct = w_cmd;                              % no PI in this mode (rpm)
    st.e_int = zeros(4, 1);                          % no PI in this mode (rpm*s)
    st.pwm = zeros(4, 1);                            % not computed in this mode (PWM counts)
    return
end

dt = P.fw_dt;                                        % step = DT_PID, L9 (s)

% ---- 1. measure: calculateSpeed, L255-285 -------------------------------------------------------------
if P.enc_rpm_quant > 0
    edges_real = st.enc_frac + st.w_absavg / P.enc_rpm_quant;  % edges since the last read, real-valued (counts)
    counts = floor(edges_real);                      % integer edges the ISRs counted, L58-61, L267 (counts)
    st.enc_frac = edges_real - counts;               % edge fraction carried to the next interval (counts)
    rpm_mag = counts * P.enc_rpm_quant;              % L276-281 with timeInSeconds = fw_dt (rpm)
else
    rpm_mag = st.w_absavg;                           % unquantised |speed| over the last interval (rpm)
end
w_meas = st.dir .* rpm_mag;                          % L283-284: sign = last COMMANDED direction (rpm)

% ---- 2. PI: applyPID, L215-221 --------------------------------------------------------------------------
e = w_cmd - w_meas;                                  % L215 error (rpm)
e_int = st.e_int + e * dt;                           % L216 integral (rpm*s)
e_int = min(max(e_int, -P.fw_int_max), P.fw_int_max);  % L219 constrain to +/-INTEGRAL_MAX (rpm*s)
delta = P.fw_Kp * e + P.fw_Ki * e_int;               % L221 deltaSpeed (rpm)

% ---- 3. correct, L222 -----------------------------------------------------------------------------------
correct = w_cmd + delta;                             % correctRPMs (rpm)

% ---- 4. direction and PWM: applyWheelRPM L228-229, setMotorSpeedRPM L248-250 ------------------------------
dir = 2 * double(correct >= 0) - 1;                  % L228 currDir: >= 0 -> +1, else -1 (-)
pwm = floor(abs(correct) / P.fw_max_rpm * 255);      % L248-249: (int) of a value >= 0 = floor (PWM counts)
pwm = min(max(pwm, 0), 255);                         % L250 constrain(pwm, 0, 255) (PWM counts)

% ---- 5. motor: first-order lag over dt, exact for a PWM held constant over the step (NOT firmware) -----
T = P.motor_gain * pwm / 255 * P.fw_max_rpm .* dir;  % speed the motor tends to under this PWM (rpm)
w0 = st.w_mot;                                       % speed at the start of the step (rpm)
tau = P.motor_tau;                                   % time constant (s)
if tau > 0
    a = exp(-dt / tau);                              % exact discrete pole of dw/dt = (T - w)/tau (-)
    w1 = T + (w0 - T) * a;                           % speed at the end of the step (rpm)
    int_w = T * dt + (w0 - T) * tau * (1 - a);       % integral of w over the step (rpm*s)
    int_abs = abs(int_w);                            % integral of |w|, if w keeps its sign (rpm*s)
    for i = 1:4                                      % wheel index (-)
        if w0(i) * T(i) < 0                          % heading through zero: may cross inside the step
            tc = tau * log((T(i) - w0(i)) / T(i));   % time at which w = 0, from w(tc) = 0 (s)
            if tc < dt
                int_1 = T(i) * tc + (w0(i) - T(i)) * tau * (1 - exp(-tc / tau));  % integral 0..tc (rpm*s)
                int_abs(i) = abs(int_1) + abs(int_w(i) - int_1);  % |0..tc| + |tc..dt| (rpm*s)
            end
        end
    end
else
    w1 = T;                                          % instant motor (rpm)
    int_w = T * dt;                                  % integral of w over the step (rpm*s)
    int_abs = abs(int_w);                            % integral of |w| over the step (rpm*s)
end

% ---- outputs and state ----------------------------------------------------------------------------------
w_act = w1;                                          % actual wheel speed, end of step (rpm)
st.w_mot = w1;                                       % (rpm)
st.w_avg = int_w / dt;                               % (rpm)
st.w_absavg = int_abs / dt;                          % read by the next call's encoder model (rpm)
st.w_meas = w_meas;                                  % measured speed used by this step's PI (rpm)
st.dir = dir;                                        % motorDirection for the next call's L283-284 (-)
st.e_int = e_int;                                    % (rpm*s)
st.correct = correct;                                % (rpm)
st.pwm = pwm;                                        % (PWM counts)
end
