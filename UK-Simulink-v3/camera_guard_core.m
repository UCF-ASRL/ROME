function [pose_out, wait, hold, stop, S] = camera_guard_core(S, pose, valid, t, cfg)
%CAMERA_GUARD_CORE  NaN-safe camera path, pure core (focus plan F6 = Plan 2 Task 1b items 1-3). One call per sample.
%   S0 = camera_guard_core(n)                                   initial state for an n x 1 pose (call once per run)
%   [pose_out, wait, hold, stop, S] = camera_guard_core(S, pose, valid, t, cfg)   one sample
%
%   Pure: no persistent variables, no workspace, no file, no print. The state lives in S, which the caller keeps
%   (a test keeps it in a variable; the model wrapper keeps it in its own block state), so the core is testable
%   offline and a new run starts clean (Fast Restart included) as soon as the caller re-creates S0.
%
%   Where it sits (wiring note f6core-report.md): downstream of PoseSource and MotiveFloor, on the floor-frame base
%   pose, so the emulator path (modes 1-3) and the OptiTrack path (mode 4) both pass through it (focus plan, F6 note).
%
%   Inputs
%     S      state struct from camera_guard_core(n) or from the previous call
%     pose   n x 1 pose (floor frame: [x (m); y (m); psi (rad)] for MotiveFloor's output). May hold NaN.
%     valid  this sample's frame-accepted flag (double or logical): MotiveData output 8 with 'optitrack', 1 with
%            'emulator'. A sample counts as a GOOD FRAME only when valid is finite and nonzero AND every element of
%            pose is finite. Both halves are needed: after its first valid frame MotiveData holds the last pose, so
%            a dropout arrives as a FINITE pose with valid = 0; and a NaN pose with valid = 1 must not pass either.
%     t      sample time (s), finite and strictly increasing from call to call (a repeat or a step back means the
%            state was not re-created for a new run: error camera_guard_core:time)
%     cfg    struct, every value passed in by the caller (the core holds no number of its own):
%              dt             sample period (s), > 0. Source: dt_9dof (define_constants.m:101 at 68ce036, 0.05 s).
%              max_wait_s     longest time without a first good frame before the run is stopped (s), >= 0, an
%                             integer number of dt. UNSOURCED (f6core-report.md, limits table).
%              max_dropout_s  longest dropout that is bridged by holding the last good pose (s), >= 0, an integer
%                             number of dt. UNSOURCED (f6core-report.md); must be the same number as
%                             allowed_ranges.m C.cam_dropout_max_s, which run_verdict_core judges mode 4 against.
%              seed           n x 1 finite pose output while no good frame has arrived (e.g. q0_9dof(1:3)). It is a
%                             placeholder so that NaN never leaves this block; while wait = 1 the commands MUST be
%                             gated by the caller (the seed is not a measurement).
%
%   Outputs
%     pose_out  n x 1, always finite. Good frame: pose itself, bit for bit (no arithmetic, so mode 1 / Table 3 are
%               unchanged). Dropout: the last good pose. Before the first good frame: cfg.seed.
%     wait      1 while no good frame has arrived yet and max_wait_s has not run out (double)
%     hold      1 during a dropout of at most max_dropout_s; pose_out is the last good pose (double)
%     stop      1 from the first sample at which the wait or a dropout exceeds its limit; LATCHED for the rest of the
%               run, a later good frame does not clear it (double). The caller must stop the run (wiring note).
%     At most one of wait, hold, stop is 1; all three 0 = a good frame this sample.
%     S         updated state. Counters (all counts are samples):
%                 n_steps       samples processed
%                 n_wait        samples without a good frame before the first one
%                 n_valid       good frames
%                 n_drop        consecutive bad samples in the current dropout (0 when the last sample was good)
%                 n_drop_max    longest dropout so far (samples); max_dropout_s_seen = n_drop_max * dt (s)
%                 n_dropouts    number of dropouts (runs of bad samples after the first good frame)
%                 stopped       true once stop has fired (latched);  reason  0 none | 1 wait ran out | 2 dropout
%                 t_first_valid, t_last_valid, t_stop   (s), NaN until they happen
%
%   Limit rule (counted in samples, not by subtracting float times): n_wait_max = max_wait_s / dt and
%   n_drop_max_ok = max_dropout_s / dt, each required to be an integer within 1e-9 (else camera_guard_core:cfg).
%   A dropout of n_drop_max_ok bad samples is HELD; the next bad sample (n_drop_max_ok + 1) STOPS. The dropout length
%   in seconds is n_drop * dt = time since the last good frame. Same rule for the wait: n_wait_max bad samples wait,
%   the next one stops.
%
%   Errors: camera_guard_core:cfg (bad limit, seed or size), camera_guard_core:time (non-finite or non-increasing t).
%   Test: tests/test_camera_guard_core.m; make-it-fail: tests/manual_makefail_camera_guard_core.m.
if nargin == 1
    n = S;                                          % pose length (count)
    pose_out = struct( ...                          % the initial state, returned as the first output
        'n_steps', 0, 'n_wait', 0, 'n_valid', 0, 'n_drop', 0, 'n_drop_max', 0, 'n_dropouts', 0, ...
        'has_valid', false, 'last_pose', NaN(n, 1), 'stopped', false, 'reason', 0, ...
        'max_dropout_s_seen', 0, 't_first_valid', NaN, 't_last_valid', NaN, 't_stop', NaN, 't_prev', -Inf);
    return
end
[n_wait_max, n_drop_ok] = limits(cfg, numel(S.last_pose));   % allowed wait and dropout (samples)
if numel(pose) ~= numel(S.last_pose)
    error('camera_guard_core:cfg', 'pose has %d elements, the state was made for %d', numel(pose), numel(S.last_pose));
end
if ~(isscalar(t) && isfinite(t) && t > S.t_prev)
    error('camera_guard_core:time', 'sample time %g s is not finite and after the previous one (%g s): state not reset for a new run?', t, S.t_prev);
end
S.t_prev = t;
S.n_steps = S.n_steps + 1;
seed = cfg.seed(:);                                 % placeholder pose before the first good frame (n x 1)
good = isscalar(valid) && isfinite(double(valid)) && double(valid) ~= 0 && all(isfinite(pose(:)));   % good frame (logical)
wait = 0;  hold = 0;  stop = 0;
if S.stopped                                        % latched: nothing after a stop changes the outputs
    stop = 1;
    pose_out = held(S, seed);
    return
end
if good
    S.n_drop = 0;
    S.has_valid = true;
    S.last_pose = pose(:);
    S.n_valid = S.n_valid + 1;
    if isnan(S.t_first_valid), S.t_first_valid = t; end
    S.t_last_valid = t;
    pose_out = pose(:);                             % the frame itself, untouched
elseif ~S.has_valid                                 % no good frame yet: wait, or stop when the wait runs out
    S.n_wait = S.n_wait + 1;
    if S.n_wait > n_wait_max
        S.stopped = true;  S.reason = 1;  S.t_stop = t;  stop = 1;
    else
        wait = 1;
    end
    pose_out = seed;
else                                                % dropout after a good frame: hold, or stop past the limit
    if S.n_drop == 0, S.n_dropouts = S.n_dropouts + 1; end
    S.n_drop = S.n_drop + 1;
    S.n_drop_max = max(S.n_drop_max, S.n_drop);
    S.max_dropout_s_seen = S.n_drop_max * cfg.dt;
    if S.n_drop > n_drop_ok
        S.stopped = true;  S.reason = 2;  S.t_stop = t;  stop = 1;
    else
        hold = 1;
    end
    pose_out = S.last_pose;
end
end

function p = held(S, seed)
% Pose output after a stop: the last good pose, or the seed when none ever arrived (n x 1, finite).
if S.has_valid
    p = S.last_pose;
else
    p = seed;
end
end

function [n_wait_max, n_drop_ok] = limits(cfg, n)
% Checks cfg and converts the two time limits to whole samples (count).
dt = cfg.dt;                                        % sample period (s)
if ~(isscalar(dt) && isfinite(dt) && dt > 0)
    error('camera_guard_core:cfg', 'cfg.dt must be a finite positive scalar (s)');
end
n_wait_max = whole(cfg.max_wait_s, dt, 'max_wait_s');
n_drop_ok  = whole(cfg.max_dropout_s, dt, 'max_dropout_s');
if numel(cfg.seed) ~= n || ~all(isfinite(cfg.seed(:)))
    error('camera_guard_core:cfg', 'cfg.seed must be %d finite values', n);
end
end

function k = whole(x, dt, name)
% x / dt as a whole number of samples; refused unless x is finite, >= 0 and an integer multiple of dt within 1e-9.
if ~(isscalar(x) && isfinite(x) && x >= 0)
    error('camera_guard_core:cfg', 'cfg.%s must be a finite scalar >= 0 (s)', name);
end
k = round(x / dt);                                  % samples
if abs(x / dt - k) > 1e-9
    error('camera_guard_core:cfg', 'cfg.%s = %g s is not a whole number of samples of dt = %g s', name, x, dt);
end
end
