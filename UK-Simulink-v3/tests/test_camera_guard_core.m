function test_camera_guard_core()
%TEST_CAMERA_GUARD_CORE  Offline test of camera_guard_core.m and arm_ready_core.m (focus plan F6, pure core).
%   Pure: no model, no base workspace, no network, no file written. Errors with id test_camera_guard_core:fail and a
%   message 'FAIL: (<check>) ...' on the first failing check; prints 'test_camera_guard_core PASS' at the end.
%   Limits used here are TEST values (dt 0.05 s, both limits 0.25 s = 5 samples), not the proposed lab values.
%   Camera guard checks:
%     (a) start with NaN pose and valid 0 -> wait, output = seed, no stop
%     (b) a NaN pose with valid 1, a finite pose with valid 0, valid NaN, one NaN element with valid 1 -> all wait
%         (a good frame needs valid AND a finite pose)
%     (c) first good frame -> run (no flag), output bit for bit the input (incl. -0 and a subnormal); after a wait
%         t_first_valid is the frame's time
%     (d) short dropout (a finite stale pose with valid 0, then NaN with valid 0) -> hold, output = last good pose,
%         not the input; the next good frame runs again
%     (e) long dropout (limit + 1 bad samples) -> stop, reason 2, output finite = last good pose
%     (f) exact boundary: limit bad samples -> still hold (n_drop = limit, max_dropout_s_seen = limit), the next one
%         stops, t_stop = its time; with max_dropout_s = 0 the first bad sample stops
%     (g) stop is latched: good frames after a dropout stop or a wait stop leave stop = 1 and the output unchanged
%     (h) counters: two dropouts of exactly the limit, separated by one good frame, never stop (the dropout count
%         restarts at each good frame); n_dropouts 2, n_drop_max = limit, n_drop 0, n_valid 3
%     (i) wait timeout: limit bad samples wait, the next one stops with reason 1 and the seed as output;
%         max_wait_s = 0 stops at the first bad sample
%     (j) NaN never reaches the output: 3 x 400 pseudo-random samples (fixed seed, rng restored) mixing good frames,
%         NaN, stale finite poses with valid 0 and partly NaN poses with valid 1, plus one 7-element run: every
%         output finite and the right size, at most one flag, a good frame before any stop passes bit for bit
%     (k) refusals: a limit that is not a whole number of samples, a NaN or wrong-size seed, a wrong-size pose
%         (camera_guard_core:cfg); a repeated, earlier or NaN time (camera_guard_core:time)
%   Arm-ready checks:
%     (l) use_telemetry 0 -> go at the first sample (armReady false, angles = seed)
%     (m) use_telemetry 1: before any line -> waiting; a bare ARM_READY (armReady true, angles still the seed) ->
%         still waiting
%     (n) ARM lines with a non-READY state (angles changed, armReady false) -> waiting; limit samples wait, the next
%         refuses with reason 2
%     (o) a garbled line (armReady true, a NaN angle) -> waiting
%     (p) a real READY line -> go, t_go; go stays latched when armReady drops and when the angles return to the seed;
%         n_lost counts the samples with armReady false
%     (q) no line at all -> refuse after the limit, reason 1, latched against a later READY line; cfg refusals
%   Make-it-fail: tests/manual_makefail_camera_guard_core.m.
root = fileparts(fileparts(mfilename('fullpath')));       % ROME_merged (or its temp mirror in the make-it-fail harness)
p0 = path;  cln = onCleanup(@() path(p0));
addpath(root);
chk('(setup)', startsWith(which('camera_guard_core'), root) && startsWith(which('arm_ready_core'), root), ...
    'code under test not resolved in %s: %s | %s', root, which('camera_guard_core'), which('arm_ready_core'));

dt = 0.05;                                   % sample period of the test (s) (dt_9dof value)
NW = 5;  ND = 5;                             % wait and dropout limits of the test (samples) = 0.25 s / 0.05 s
C = struct('dt', dt, 'max_wait_s', NW * dt, 'max_dropout_s', ND * dt, 'seed', [0.1; 0.2; 0.3]);   % guard cfg
P1 = [1.25; -0.5; -0];                       % a good pose with a negative zero (m, m, rad)
P2 = [1.30; -0.45; realmin / 4];             % a good pose with a subnormal heading (m, m, rad)
STALE = [9; 9; 9];                           % a finite pose that arrives with valid 0 (m, m, rad)

% ---- (a) start NaN -> wait ----
S = camera_guard_core(3);
[po, w, h, s, S] = camera_guard_core(S, NaN(3, 1), 0, 0, C);
chk('(a)', w == 1 && h == 0 && s == 0, 'first NaN sample: wait %g hold %g stop %g, want 1 0 0', w, h, s);
chk('(a)', isequal(po, C.seed) && all(isfinite(po)), 'first NaN sample: output %s, want the seed', mat2str(po));
chk('(a)', S.n_wait == 1 && S.n_valid == 0 && isnan(S.t_first_valid) && ~S.stopped, 'state after one NaN sample wrong');
[po, w, h, s] = camera_guard_core(S, NaN(3, 1), 0, dt, C);
chk('(a)', w == 1 && h == 0 && s == 0 && isequal(po, C.seed), 'second NaN sample: not wait on the seed');
fprintf('(a) PASS  NaN start -> wait on the seed\n');

% ---- (b) a good frame needs valid AND a finite pose ----
cases = {NaN(3, 1), 1; STALE, 0; STALE, NaN; [1; NaN; 2], 1; [Inf; 0; 0], 1};
for c = 1:size(cases, 1)
    S = camera_guard_core(3);
    [po, w, h, s, S] = camera_guard_core(S, cases{c,1}, cases{c,2}, 0, C);
    chk('(b)', w == 1 && h == 0 && s == 0 && isequal(po, C.seed) && S.n_valid == 0, ...
        'case %d (pose %s, valid %g) was accepted: wait %g, output %s', c, mat2str(cases{c,1}), cases{c,2}, w, mat2str(po));
end
fprintf('(b) PASS  NaN with valid 1, stale with valid 0, valid NaN, partial NaN: all wait\n');

% ---- (c) first good frame -> run, bit for bit ----
S = camera_guard_core(3);
[po, w, h, s, S] = camera_guard_core(S, P1, 1, 0, C);
chk('(c)', w == 0 && h == 0 && s == 0, 'first good frame: flags %g %g %g, want 0 0 0', w, h, s);
chk('(c)', bits(po, P1), 'first good frame not passed bit for bit: %s', mat2str(po));
chk('(c)', S.n_valid == 1 && S.t_first_valid == 0 && S.t_last_valid == 0 && S.has_valid, 'state after the first good frame wrong');
[po, w, h, s] = camera_guard_core(S, P2, true, dt, C);
chk('(c)', w == 0 && h == 0 && s == 0 && bits(po, P2), 'second good frame (logical valid) not passed bit for bit');
S = camera_guard_core(3);
for k = 0:2, [~, ~, ~, ~, S] = camera_guard_core(S, NaN(3, 1), 0, k * dt, C); end
[po, w, h, s, S] = camera_guard_core(S, P1, 1, 3 * dt, C);
chk('(c)', w == 0 && h == 0 && s == 0 && bits(po, P1) && S.t_first_valid == 3 * dt && S.n_wait == 3, 'good frame after 3 waits: not run');
fprintf('(c) PASS  first good frame -> run, output bit for bit\n');

% ---- (d) short dropout -> hold the last good pose ----
S = camera_guard_core(3);
[~, ~, ~, ~, S] = camera_guard_core(S, P1, 1, 0, C);
[po, w, h, s, S] = camera_guard_core(S, STALE, 0, dt, C);
chk('(d)', h == 1 && w == 0 && s == 0, 'stale pose with valid 0: hold %g wait %g stop %g, want 1 0 0', h, w, s);
chk('(d)', bits(po, P1), 'stale pose with valid 0: output %s, want the last good pose %s', mat2str(po), mat2str(P1));
[po, w, h, s, S] = camera_guard_core(S, NaN(3, 1), 0, 2 * dt, C);
chk('(d)', h == 1 && w == 0 && s == 0 && bits(po, P1), 'NaN with valid 0 in a dropout: output %s, hold %g', mat2str(po), h);
[po, w, h, s] = camera_guard_core(S, P2, 1, 3 * dt, C);
chk('(d)', w == 0 && h == 0 && s == 0 && bits(po, P2), 'good frame after a short dropout: not run on it');
fprintf('(d) PASS  short dropout -> hold the last good pose, then run\n');

% ---- (e) long dropout -> stop ----
S = camera_guard_core(3);
[~, ~, ~, ~, S] = camera_guard_core(S, P1, 1, 0, C);
for k = 1:ND + 1
    [po, w, h, s, S] = camera_guard_core(S, NaN(3, 1), 0, k * dt, C);
end
chk('(e)', s == 1 && h == 0 && w == 0, 'after %d bad samples: stop %g hold %g wait %g, want 1 0 0', ND + 1, s, h, w);
chk('(e)', S.reason == 2 && S.stopped && bits(po, P1), 'dropout stop: reason %g, output %s', S.reason, mat2str(po));
fprintf('(e) PASS  dropout longer than the limit -> stop, reason 2\n');

% ---- (f) exact boundary ----
S = camera_guard_core(3);
[~, ~, ~, ~, S] = camera_guard_core(S, P1, 1, 0, C);
for k = 1:ND
    [~, w, h, s, S] = camera_guard_core(S, NaN(3, 1), 0, k * dt, C);
    chk('(f)', h == 1 && s == 0 && w == 0, 'bad sample %d of a %d-sample limit: hold %g stop %g, want 1 0', k, ND, h, s);
end
chk('(f)', S.n_drop == ND && abs(S.max_dropout_s_seen - ND * dt) < 1e-12, 'at the limit: n_drop %g, max_dropout_s_seen %g', S.n_drop, S.max_dropout_s_seen);
[~, w, h, s, S] = camera_guard_core(S, NaN(3, 1), 0, (ND + 1) * dt, C);
chk('(f)', s == 1 && h == 0 && w == 0 && S.t_stop == (ND + 1) * dt, 'bad sample %d (limit + 1): stop %g, t_stop %g', ND + 1, s, S.t_stop);
C0 = C;  C0.max_dropout_s = 0;
S = camera_guard_core(3);
[~, ~, ~, ~, S] = camera_guard_core(S, P1, 1, 0, C0);
[~, ~, h, s] = camera_guard_core(S, P1, 0, dt, C0);
chk('(f)', s == 1 && h == 0, 'max_dropout_s = 0: first bad sample did not stop (stop %g hold %g)', s, h);
fprintf('(f) PASS  limit samples hold, limit + 1 stops; limit 0 stops at once\n');

% ---- (g) stop is latched ----
S = camera_guard_core(3);
[~, ~, ~, ~, S] = camera_guard_core(S, P1, 1, 0, C);
for k = 1:ND + 1, [~, ~, ~, ~, S] = camera_guard_core(S, NaN(3, 1), 0, k * dt, C); end
nv = S.n_valid;
for k = ND + 2:ND + 4
    [po, w, h, s, S] = camera_guard_core(S, P2, 1, k * dt, C);
    chk('(g)', s == 1 && w == 0 && h == 0 && bits(po, P1), 'good frame %d after a dropout stop: stop %g, output %s', k, s, mat2str(po));
end
chk('(g)', S.reason == 2 && S.n_valid == nv, 'state changed after the stop: reason %g, n_valid %g (was %g)', S.reason, S.n_valid, nv);
S = camera_guard_core(3);
for k = 0:NW, [~, ~, ~, ~, S] = camera_guard_core(S, NaN(3, 1), 0, k * dt, C); end
[po, w, h, s, S] = camera_guard_core(S, P1, 1, (NW + 1) * dt, C);
chk('(g)', s == 1 && w == 0 && h == 0 && isequal(po, C.seed) && S.reason == 1, 'good frame after a wait stop: stop %g, output %s', s, mat2str(po));
fprintf('(g) PASS  stop latched after a dropout stop and after a wait stop\n');

% ---- (h) counters restart at each good frame ----
S = camera_guard_core(3);
t = 0;
[~, ~, ~, ~, S] = camera_guard_core(S, P1, 1, t, C);
for rep = 1:2
    for k = 1:ND
        t = t + dt;
        [~, ~, ~, s, S] = camera_guard_core(S, NaN(3, 1), 0, t, C);
        chk('(h)', s == 0, 'dropout %d, bad sample %d: stopped although each dropout is exactly the limit', rep, k);
    end
    t = t + dt;
    [~, ~, ~, s, S] = camera_guard_core(S, P2, 1, t, C);
    chk('(h)', s == 0, 'good frame after dropout %d: stop', rep);
end
chk('(h)', S.n_dropouts == 2 && S.n_drop_max == ND && S.n_drop == 0 && S.n_valid == 3, ...
    'counters: n_dropouts %g n_drop_max %g n_drop %g n_valid %g, want 2 %d 0 3', S.n_dropouts, S.n_drop_max, S.n_drop, S.n_valid, ND);
fprintf('(h) PASS  two limit-length dropouts never stop; counters right\n');

% ---- (i) wait timeout ----
S = camera_guard_core(3);
for k = 0:NW - 1
    [~, w, ~, s, S] = camera_guard_core(S, NaN(3, 1), 0, k * dt, C);
    chk('(i)', w == 1 && s == 0, 'wait sample %d of %d: wait %g stop %g', k + 1, NW, w, s);
end
[po, w, h, s, S] = camera_guard_core(S, NaN(3, 1), 0, NW * dt, C);
chk('(i)', s == 1 && w == 0 && h == 0 && S.reason == 1 && isequal(po, C.seed) && S.t_stop == NW * dt, ...
    'wait sample %d (limit + 1): stop %g reason %g output %s', NW + 1, s, S.reason, mat2str(po));
C0 = C;  C0.max_wait_s = 0;
S = camera_guard_core(3);
[~, w, ~, s] = camera_guard_core(S, NaN(3, 1), 0, 0, C0);
chk('(i)', s == 1 && w == 0, 'max_wait_s = 0: first bad sample did not stop');
fprintf('(i) PASS  wait limit: limit samples wait, the next one stops (reason 1)\n');

% ---- (j) NaN never reaches the output ----
rs = rng;  rcl = onCleanup(@() rng(rs));
rng(20261120, 'twister');
cfgs = {C, setfield(C, 'max_dropout_s', 0), setfield(setfield(C, 'max_dropout_s', 20 * dt), 'max_wait_s', 20 * dt)}; %#ok<SFLD>
nGood = 0;
for ci = 1:numel(cfgs)
    nGood = nGood + soak(cfgs{ci}, 3, 400, dt);
end
C7 = C;  C7.seed = (1:7).' / 10;
nGood = nGood + soak(C7, 7, 200, dt);
chk('(j)', nGood > 100, 'the soak passed only %d good frames through (the pattern is not exercising run)', nGood);
fprintf('(j) PASS  1400 mixed samples: every output finite, flags exclusive, %d good frames bit for bit\n', nGood);

% ---- (k) refusals ----
S = camera_guard_core(3);
bad = {setfield(C, 'max_dropout_s', 0.27), setfield(C, 'max_wait_s', 0.12), setfield(C, 'max_dropout_s', -dt), ...
       setfield(C, 'dt', 0), setfield(C, 'seed', [0; NaN; 0]), setfield(C, 'seed', [0; 0])}; %#ok<SFLD>
for b = 1:numel(bad)
    chk('(k)', raises(@() camera_guard_core(S, P1, 1, 0, bad{b}), 'camera_guard_core:cfg'), 'bad cfg %d not refused with camera_guard_core:cfg', b);
end
chk('(k)', raises(@() camera_guard_core(S, [1; 2], 1, 0, C), 'camera_guard_core:cfg'), 'wrong-size pose not refused');
[~, ~, ~, ~, S] = camera_guard_core(S, P1, 1, dt, C);
chk('(k)', raises(@() camera_guard_core(S, P1, 1, dt, C), 'camera_guard_core:time'), 'repeated time not refused');
chk('(k)', raises(@() camera_guard_core(S, P1, 1, 0, C), 'camera_guard_core:time'), 'earlier time not refused');
chk('(k)', raises(@() camera_guard_core(S, P1, 1, NaN, C), 'camera_guard_core:time'), 'NaN time not refused');
fprintf('(k) PASS  bad limits, seed, pose size and time refused\n');

% ================= arm_ready_core =================
home = rad2deg(deg2rad([-3.0; 127.0; -10.0; 0.0; -27.0; 0.0]));   % the seed as ROMECommand computes it (model deg)
A = struct('use_telemetry', 1, 'home_deg', home, 'dt', dt, 'max_wait_s', NW * dt);   % arm-ready cfg
REAL = [-3.4; 125.6; -8.9; 0.1; -24.3; 0.2];                     % a telemetry line, mapped (model deg)

% ---- (l) use_telemetry 0 -> go at once ----
A0 = A;  A0.use_telemetry = 0;
R = arm_ready_core();
[g, wt, rf, R] = arm_ready_core(R, false, home, 0, A0);
chk('(l)', g == 1 && wt == 0 && rf == 0 && R.t_go == 0, 'use_telemetry 0: go %g waiting %g refuse %g', g, wt, rf);
[g, wt, rf] = arm_ready_core(R, false, home, dt, A0);
chk('(l)', g == 1 && wt == 0 && rf == 0, 'use_telemetry 0: go not kept');
fprintf('(l) PASS  no telemetry needed -> go at the first sample\n');

% ---- (m) no line, then a bare ARM_READY -> waiting ----
R = arm_ready_core();
[g, wt, rf, R] = arm_ready_core(R, false, home, 0, A);
chk('(m)', g == 0 && wt == 1 && rf == 0, 'before any line: go %g waiting %g refuse %g', g, wt, rf);
[g, wt, rf, R] = arm_ready_core(R, true, home, dt, A);
chk('(m)', g == 0 && wt == 1 && rf == 0 && ~R.line_seen, 'bare ARM_READY (angles still the seed) started the run: go %g', g);
fprintf('(m) PASS  seed angles never start the run, even with armReady true\n');

% ---- (n) non-READY ARM lines -> waiting, then refuse reason 2 ----
R = arm_ready_core();
for k = 0:NW - 1
    [g, wt, rf, R] = arm_ready_core(R, false, REAL, k * dt, A);
    chk('(n)', g == 0 && wt == 1 && rf == 0, 'non-READY line, sample %d: go %g waiting %g refuse %g', k + 1, g, wt, rf);
end
[g, wt, rf, R] = arm_ready_core(R, false, REAL, NW * dt, A);
chk('(n)', g == 0 && wt == 0 && rf == 1 && R.reason == 2 && R.line_seen && R.t_refuse == NW * dt, ...
    'non-READY lines past the limit: refuse %g reason %g', rf, R.reason);
fprintf('(n) PASS  lines with a non-READY state wait, then refuse (reason 2)\n');

% ---- (o) garbled line -> waiting ----
R = arm_ready_core();
[g, wt, ~, R] = arm_ready_core(R, true, [REAL(1:5); NaN], 0, A);
chk('(o)', g == 0 && wt == 1 && ~R.line_seen, 'a NaN angle with armReady true started the run');
[g, wt] = arm_ready_core(R, true, NaN(6, 1), dt, A);
chk('(o)', g == 0 && wt == 1, 'all-NaN angles with armReady true started the run');
fprintf('(o) PASS  garbled telemetry never starts the run\n');

% ---- (p) a real READY line -> go, latched ----
R = arm_ready_core();
[~, ~, ~, R] = arm_ready_core(R, false, home, 0, A);
[g, wt, rf, R] = arm_ready_core(R, true, REAL, dt, A);
chk('(p)', g == 1 && wt == 0 && rf == 0 && R.t_go == dt, 'first READY line: go %g waiting %g, t_go %g', g, wt, R.t_go);
[g, wt, rf, R] = arm_ready_core(R, false, REAL, 2 * dt, A);
chk('(p)', g == 1 && wt == 0 && rf == 0, 'go not latched when armReady dropped: go %g waiting %g', g, wt);
[g, ~, ~, R] = arm_ready_core(R, false, home, 3 * dt, A);
chk('(p)', g == 1 && R.n_lost == 2, 'go not latched with seed angles, or n_lost %g ~= 2', R.n_lost);
fprintf('(p) PASS  first READY line -> go, latched; n_lost counted\n');

% ---- (q) no line at all -> refuse reason 1, latched; cfg refusals ----
R = arm_ready_core();
for k = 0:NW - 1
    [~, wt, rf, R] = arm_ready_core(R, false, home, k * dt, A);
    chk('(q)', wt == 1 && rf == 0, 'no line, sample %d: waiting %g refuse %g', k + 1, wt, rf);
end
[g, wt, rf, R] = arm_ready_core(R, false, home, NW * dt, A);
chk('(q)', rf == 1 && g == 0 && wt == 0 && R.reason == 1, 'no line past the limit: refuse %g reason %g', rf, R.reason);
[g, ~, rf] = arm_ready_core(R, true, REAL, (NW + 1) * dt, A);
chk('(q)', rf == 1 && g == 0, 'refuse not latched against a later READY line');
R = arm_ready_core();
badA = {setfield(A, 'use_telemetry', 2), setfield(A, 'home_deg', [home(1:5); NaN]), setfield(A, 'home_deg', home(1:5)), ...
        setfield(A, 'max_wait_s', 0.27), setfield(A, 'dt', -1)}; %#ok<SFLD>
for b = 1:numel(badA)
    chk('(q)', raises(@() arm_ready_core(R, true, REAL, 0, badA{b}), 'arm_ready_core:cfg'), 'bad arm cfg %d not refused', b);
end
[~, ~, ~, R] = arm_ready_core(R, false, home, 0, A);
chk('(q)', raises(@() arm_ready_core(R, false, home, 0, A), 'arm_ready_core:time'), 'repeated arm time not refused');
fprintf('(q) PASS  no line -> refuse (reason 1), latched; bad cfg and time refused\n');

fprintf('test_camera_guard_core PASS\n');
end

function nGood = soak(C, n, N, dt)
% N pseudo-random samples through one guard state; checks every output. Returns the good frames passed before a stop.
S = camera_guard_core(n);
nGood = 0;
for k = 0:N - 1
    r = rand;                                % draw that picks the sample kind (-)
    pose = randn(n, 1);                      % a finite pose (m, rad)
    valid = 1;                               % frame flag (-)
    if r < 0.15
        pose = NaN(n, 1);  valid = 0;        % no frame yet / NaN with valid 0
    elseif r < 0.25
        valid = 0;                           % stale finite pose, valid 0
    elseif r < 0.30
        pose(randi(n)) = NaN;                % partly NaN pose with valid 1
    elseif r < 0.33
        pose = NaN(n, 1);                    % all NaN with valid 1
    end
    wasStopped = S.stopped;
    [po, w, h, s, S] = camera_guard_core(S, pose, valid, k * dt, C);
    chk('(j)', isequal(size(po), [n 1]) && all(isfinite(po)), 'sample %d (cfg wait %g drop %g): output %s', k, C.max_wait_s, C.max_dropout_s, mat2str(po));
    chk('(j)', w + h + s <= 1 && all(ismember([w h s], [0 1])), 'sample %d: flags %g %g %g not exclusive', k, w, h, s);
    good = valid ~= 0 && all(isfinite(pose));
    if good && ~wasStopped
        chk('(j)', w + h + s == 0 && bits(po, pose), 'sample %d: good frame not passed bit for bit', k);
        nGood = nGood + 1;
    end
end
end

function tf = bits(a, b)
% True when a and b are the same doubles bit for bit (so -0 ~= +0).
tf = isequal(size(a), size(b(:))) && isequal(typecast(double(a(:)), 'uint64'), typecast(double(b(:)), 'uint64'));
end

function tf = raises(f, id)
% True when f() raises the error id.
tf = false;
try
    f();
catch e
    tf = strcmp(e.identifier, id);
end
end

function chk(name, ok, fmt, varargin)
% Errors test_camera_guard_core:fail with 'FAIL: <name> ...' when ok is false.
if ~ok
    error('test_camera_guard_core:fail', ['FAIL: %s ' fmt], name, varargin{:});
end
end
