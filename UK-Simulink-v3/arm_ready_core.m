function [go, waiting, refuse, S] = arm_ready_core(S, armReady, jointDeg, t, cfg)
%ARM_READY_CORE  Hold the run start until the first real ARM telemetry line has arrived (focus plan F6 note, Q-F7-b).
%   S0 = arm_ready_core()                                        initial state (call once per run)
%   [go, waiting, refuse, S] = arm_ready_core(S, armReady, jointDeg, t, cfg)   one sample
%
%   Pure: no persistent variables, no workspace, no file, no print. Reads ROMECommand's EXISTING outputs only;
%   ROMECommand.m is not edited (read-only at 68ce036).
%
%   Why (focus plan, F6 note): ROMECommand outputs rad2deg(arm_home) on output 2 until the first ARM line is parsed
%   (ROMECommand.m:42-46 seed; :156-168 ARM line), a jump of up to 4.8 deg when the real angles arrive. In modes 3
%   and 4 the controller takes the arm from that output (JointSource(257) via TelemetryDelay(223)), so the run must
%   not start on the seed.
%
%   What ROMECommand offers, and why armReady alone is NOT enough (ROMECommand.m at 68ce036):
%     out 2 actualJointDeg  6x1 model degrees: the seed until an "ARM,j1..j6,STATE" line with >= 8 fields
%                           (:156-168); a garbled number becomes NaN through str2double (:159)
%     out 3 armReady        logical: true when an ARM line's 8th field is READY (:166), ALSO true on a bare
%                           "ARM_READY" line (:170-172) that carries no angles (the Teensy sends it once after
%                           CAL_ARM, ROME_Teensy_Code.ino:164); false again 0.5 s after the last GV/ARM/ARM_READY line
%                           (:182-184)
%   There is no output that counts ARM lines. The proxy used here: a sample is a REAL ARM LINE when armReady is true
%   AND every angle is finite AND at least one angle differs from the seed cfg.home_deg (bit for bit: the seed is
%   rad2deg(arm_home(:)) as ROMECommand computes it, so an unchanged output compares exactly equal).
%   Blind spot: an arm whose telemetry, mapped back to model degrees, equals rad2deg(arm_home) EXACTLY in all six
%   joints is never seen (the run is then refused when max_wait_s runs out, never started on the seed). An exact
%   signal needs a line counter output in ROMECommand: an edit for the author to decide (f6core-report.md).
%
%   Inputs
%     S         state from arm_ready_core() or the previous call
%     armReady  ROMECommand output 3 (logical or double)
%     jointDeg  ROMECommand output 2, 6x1 (model deg)
%     t         sample time (s), finite and strictly increasing (else arm_ready_core:time)
%     cfg       struct, every value from the caller:
%                 use_telemetry  1 when the controller takes the arm from telemetry (modes 3, 4): the same expression as
%                                JointFromTelemetry(256), double(EnableHardware ~= 0 && arm_freeze_user == 0).
%                                0: go at the first sample, nothing is awaited (modes 1, 2; armReady stays false there)
%                 home_deg       6x1 the seed ROMECommand outputs before any ARM line (model deg):
%                                rad2deg(arm_home(:)), or zeros(6,1) when arm_home does not exist (ROMECommand.m:42-46)
%                 dt             sample period (s), > 0 (dt_9dof)
%                 max_wait_s     longest wait for the first real ARM line (s), >= 0, a whole number of dt. UNSOURCED
%                                (f6core-report.md, limits table)
%   Outputs
%     go        1 from the sample on which the first real ARM line is seen (or at once with use_telemetry 0); LATCHED
%     waiting   1 while not go and not refused
%     refuse    1 from the first sample after max_wait_s without a real ARM line (n_wait_max waiting samples are
%               allowed, the next one refuses); LATCHED. The caller stops the run with the reason S.reason
%     S         state: n_steps, n_wait (waiting samples), line_seen (angles changed from the seed at least once, while
%               armReady was false: an ARM line whose state is not READY), go, refused, reason (0 none | 1 no ARM
%               line seen | 2 ARM lines seen but the arm state is not READY, e.g. UNCALIBRATED: run calibrateROMEArm),
%               t_go, t_refuse (s, NaN until then), n_lost (samples after go with armReady false: informational,
%               ROMECommand drops armReady 0.5 s after the last line)
%   Errors: arm_ready_core:cfg, arm_ready_core:time.
%   Test: tests/test_camera_guard_core.m checks (l)-(q); make-it-fail: tests/manual_makefail_camera_guard_core.m.
if nargin == 0
    go = struct('n_steps', 0, 'n_wait', 0, 'line_seen', false, 'go', false, 'refused', false, 'reason', 0, ...
        't_go', NaN, 't_refuse', NaN, 'n_lost', 0, 't_prev', -Inf);   % the initial state, returned as the first output
    return
end
n_wait_max = check_cfg(cfg);                        % allowed waiting samples (count)
if ~(isscalar(t) && isfinite(t) && t > S.t_prev)
    error('arm_ready_core:time', 'sample time %g s is not finite and after the previous one (%g s): state not reset for a new run?', t, S.t_prev);
end
S.t_prev = t;
S.n_steps = S.n_steps + 1;
rdy = isscalar(armReady) && double(armReady) == 1;  % ROMECommand's READY flag this sample (logical)
go = 0;  waiting = 0;  refuse = 0;
if S.refused
    refuse = 1;
    return
end
if S.go
    go = 1;
    if ~rdy, S.n_lost = S.n_lost + 1; end
    return
end
if cfg.use_telemetry == 0
    S.go = true;  S.t_go = t;  go = 1;
    return
end
j = double(jointDeg(:));                            % telemetry angles this sample (model deg)
fresh = numel(j) == 6 && all(isfinite(j)) && any(j ~= cfg.home_deg(:));   % angles that are not the seed (logical)
if rdy && fresh
    S.go = true;  S.t_go = t;  go = 1;
    return
end
if fresh, S.line_seen = true; end
S.n_wait = S.n_wait + 1;
if S.n_wait > n_wait_max
    S.refused = true;  S.t_refuse = t;  refuse = 1;
    S.reason = 1 + double(S.line_seen);
else
    waiting = 1;
end
end

function n_wait_max = check_cfg(cfg)
% Checks cfg; returns max_wait_s as whole samples (count).
if ~(isscalar(cfg.use_telemetry) && (cfg.use_telemetry == 0 || cfg.use_telemetry == 1))
    error('arm_ready_core:cfg', 'cfg.use_telemetry must be 0 or 1');
end
if numel(cfg.home_deg) ~= 6 || ~all(isfinite(cfg.home_deg(:)))
    error('arm_ready_core:cfg', 'cfg.home_deg must be 6 finite values (model deg)');
end
dt = cfg.dt;                                        % sample period (s)
if ~(isscalar(dt) && isfinite(dt) && dt > 0)
    error('arm_ready_core:cfg', 'cfg.dt must be a finite positive scalar (s)');
end
x = cfg.max_wait_s;                                 % (s)
if ~(isscalar(x) && isfinite(x) && x >= 0)
    error('arm_ready_core:cfg', 'cfg.max_wait_s must be a finite scalar >= 0 (s)');
end
n_wait_max = round(x / dt);                         % samples
if abs(x / dt - n_wait_max) > 1e-9
    error('arm_ready_core:cfg', 'cfg.max_wait_s = %g s is not a whole number of samples of dt = %g s', x, dt);
end
end
