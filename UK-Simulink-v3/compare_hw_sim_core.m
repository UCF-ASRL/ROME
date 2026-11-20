function rep = compare_hw_sim_core(hw, sim, P)
%COMPARE_HW_SIM_CORE  Measured (hardware run) vs simulated run of the same start. Re-simulates NOTHING.
%   rep = compare_hw_sim_core(hw, sim, P)    focus plan F3 / Plan 3 Task 3 (pure part), 19 Nov 2026.
%   hw, sim  each either
%            (a) a run file as save_run writes it, loaded: struct with fields out (logged signals) and snap; or
%            (b) a run struct already in the canonical form below (field t present, no field out).
%            sim may be [] (then only the hardware-internal and per-line parts are reported).
%   P        optional struct: tel_gap_s (s, default 0.5: ROMECommand.m L206 drops armReady after 0.5 s without
%            a telemetry line), win_frac (default 0.95: end of the catch-up window, as in the Task 1c method).
%
%   SIGNALS USED from out (To Workspace names of the current ROME_9DOF.slx; wiring and SIDs in
%   docs/verification/2026-11-19_f3core_wiring.log). Each is a Timeseries or a struct with Time and Data.
%     q_cmd (SID 208)        UKDynamics out 1, N x 9 (m, rad): the command / ideal-plant state.
%     q_meas (SID 226)       SensorModel out = SensorModel(StateDelay(q_cmd)). NOT what the controller reads when
%                            EnableMotive = 1 (StateSource SID 218 then takes MuxTracked, which is not logged).
%     q_track (SID 151)      Enabled Subsystem (SID 100) out 1 'tracked_position', the camera-pipeline base
%                            [x y yaw] (m, m, rad).
%     wheel_sent (SID 227)   CommandGuard out 1 = ROMECommand in 1, N x 4 rpm: the wheels sent.
%     joint_sent_rad (228)   ArmFreezeSwitch out = ROMECommand in 2, N x 6 model rad: the joints sent (the frozen
%                            posture when arm_freeze_user = 1). joint_guard_rad (240) is the controller's joint
%                            command before the switch (reported only).
%     wheel_meas (SID 229)   ROMECommand out 1, rpm, GV fields 2-5. Its SIGN is the Mega's echo of the command
%                            (bench_wheel_order.m header, ROME_Mega_Code.ino L283-284), so the measured-vs-
%                            commanded wheel error compares MAGNITUDES; the sign is reported as an echo check.
%     joint_meas_deg (230)   ROMECommand out 2, model DEGREES (ROMECommand.m L162-163 un-maps the firmware degrees
%                            with arm_offset_deg / arm_sign; taken BEFORE ArmDeg2Rad and TelemetryDelay). J4 has
%                            no encoder (bench_joint_signs.m header): its "measurement" is the command.
%     arm_ready (SID 231)    ROMECommand out 3.
%   Both ROMECommand outputs HOLD their last value when telemetry stops (ROMECommand.m L76-82, L211-213); with
%   EnableHardware = 0 they are zeros and constant arm_home (no connection), so for a SIM run (snap.EnableHardware
%   = 0) the "measured" wheels are wheel_sent and the "measured" joints q_meas(:,4:9) (ideal actuators).
%   Base source: EnableMotive = 1 -> q_track, a MEASUREMENT only with motive_source 'optitrack' (mode 3); with
%   'emulator' (mode 2) it is the model's own prediction through the camera pipeline. motive_source absent (model
%   before F7) with EnableMotive 1 is taken as the OptiTrack reader, the only one that model has.
%   EnableMotive = 0 -> q_meas(:,1:3), the model's open-loop state, not a measurement.
%   Optional recorder fields (contract for the telemetry recorder, see the F3 core report):
%     out.telemetry  struct: t (sim s), tw (wall s), line (string): every raw line received from the ESP32.
%                    Lines that ROMECommand.m L146-169 accepts ("GV," >= 9 fields, "ARM," >= 8, "ARM_READY")
%                    are telemetry; anything else is ignored.
%     out.send_log   struct: ts (sim s), tw (wall s), line (string): every line written to the robot, in order
%                    (START_GV, ROME,..., STOP_ALL), the format of the Task 1c record (test_hw_nocam).
%   snap fields used: EnableHardware, EnableMotive, motive_source, arm_freeze_user, q0_9dof, dt_9dof, r, l, alphas.
%
%   CANONICAL RUN STRUCT (form (b); every field optional except t; N rows, SI units):
%     t (N x 1 s); base (N x 3 m, m, rad); base_source (char); base_measured (logical); wheel_cmd, wheel_meas
%     (N x 4 rpm); joint_cmd, joint_meas, joint_guard (N x 6 rad); arm_ready (N x 1); has_tel (logical) and
%     tel_t (telemetry line times, sim s); send (struct: ts, tw (s), wheel (rpm), tw_stop (s or NaN), q0 (3 x 1
%     m, m, rad), dt (s), r (m), l (m), alphas (rad)); q_cmd (N x 9); frozen (flag or NaN).
%
%   REPORT rep (errors are absolute values; "stats" = struct max, rms, start, end, t_max, n):
%     n_overlap, t_overlap      hw samples inside the sim's time span, and that span (s); sim is interpolated
%                               linearly onto the hw times (heading unwrapped first).
%     hw_base_source, hw_base_measured, sim_base_source
%     dropout                   source, n, t_start, t_end (s), t_flag (arm_ready edge times, s, fallback only)
%                               primary: gaps > tel_gap_s between accepted telemetry lines (and before the first /
%                               after the last); fallback without line times: arm_ready falling edges, labelled
%                               "not separable" (an "ARM,...,BUSY" line also clears armReady, L165).
%     n_excluded                hw samples inside a dropout: excluded from every MEASURED comparison (the values
%                               are stale holds there).
%     arm_not_ready             K x 2 intervals (s) where arm_ready is false (information).
%     base.pos, base.yaw        stats of |xy_hw - xy_sim| (m) and |wrap(yaw_hw - yaw_sim)| (rad), all overlap samples.
%     ee.err                    stats of |p_ee(hw) - p_ee(sim)| (m), p_ee = ik9_fk([base; joint_meas]): the end
%                               effector computed through the formulation's kinematics from the MEASURED base and
%                               joints (the cameras measure the base only). MODEL-FRAME end effector: ik9_fk's
%                               DH table is a PLACEHOLDER (ik9_fk.m L15), and the real AR3 geometry differs
%                               (Plan 1 Task 6, docs/verification/2026-11-19_commanding_check.md, ar3_fk.m), so this
%                               is NOT the physical tool tip. Using ik9_fk on BOTH sides is the right choice for
%                               hardware vs simulation: the same kinematics turns the measured and the simulated
%                               configurations into one comparable point, so the difference reflects the base and
%                               joint differences, not a geometry mismatch. The label is also put in rep.notes.
%     wheel.hw_meas_vs_cmd     per-wheel stats of ||meas| - |cmd|| (rpm), hw only, valid samples;
%     wheel.echo_sign_mismatch  per wheel, count of valid samples with |cmd| > 1 rpm whose echoed sign differs;
%     wheel.hw_cmd_vs_sim_cmd, wheel.hw_meas_vs_sim_meas   per-wheel stats (rpm).
%     joint.hw_meas_vs_cmd, joint.hw_cmd_vs_sim_cmd, joint.hw_meas_vs_sim_meas   per-joint stats (rad).
%     send                      per-line send times (if hw carries them): the Task 1c method
%                               (docs/verification/2026-11-19_p2t1c_pacing_deficit.md, tools/p2t1c_pacing_deficit.m):
%                               line k holds h_k = tw(k+1) - tw(k) instead of dt; body velocity = least-squares
%                               inverse of the model's wheel map, rome_wheels (the block's function, called through
%                               mass_at's handle P.wheelmap with snap's r, l, alphas); explicit Euler from q0,
%                               heading at the start of each hold, nominal (dt) and actual (h_k); deficit =
%                               |actual - nominal|. Fields: n_lines, wheel_source, lag (s), lag_min, t_lag_min,
%                               t_window_end, dxy_window_mm, dxy_end_mm, dxy_max_mm, t_dxy_max, dyaw_window_deg,
%                               dyaw_end_deg, dyaw_maxabs_deg, path_m, fit_res_rpm, hold_median/min/max_s,
%                               last_hold_s (until STOP_ALL), sanity_qcmd_mm (nominal path vs q_cmd), pN, pA.
%     notes                     cellstr of caveats that apply to this pair of runs.
%   Limits: the 1c hold is the SEND-to-send interval; the robot holds from arrival to arrival, so network jitter
%   is not in it. Open loop, ideal wheels (no motor lag, no slip), as in the 1c document.
if nargin < 2, sim = []; end
if nargin < 3 || isempty(P), P = struct(); end
if ~isfield(P, 'tel_gap_s'), P.tel_gap_s = 0.5; end
if ~isfield(P, 'win_frac'), P.win_frac = 0.95; end
H = canon_run(hw);
S = canon_run(sim);
rep = struct();
rep.notes = {};
rep.hw_base_source = H.base_source;
rep.hw_base_measured = H.base_measured;
rep.frozen = H.frozen;
if ~isempty(S), rep.sim_base_source = S.base_source; else, rep.sim_base_source = 'none'; end
if ~H.base_measured
    rep.notes{end+1} = 'hw base is not a measurement (no camera, or the emulator): the base comparison shows the model against itself; the per-line deficit is the base estimate';
end
if isequal(H.frozen, 1)
    rep.notes{end+1} = 'arm frozen: joints sent = q0_9dof(4:9), not the controller command joint_guard_rad';
end
if ~isempty(H.joint_meas)
    rep.notes{end+1} = 'J4 has no encoder: its measured value is the command';
end

% ---------- dropout (telemetry), on the hw time base
nH = numel(H.t);
if nH > 0
    [dro, droSrc] = detect_dropouts(H, P);
else
    dro = zeros(0, 2);  droSrc = 'no logged signals';
end
rep.dropout = struct('source', droSrc, 'n', size(dro, 1), 't_start', dro(:, 1), 't_end', dro(:, 2), ...
                     't_flag', NaN(size(dro, 1), 1));
if startsWith(droSrc, 'arm_ready'), rep.dropout.t_flag = dro(:, 1) + P.tel_gap_s; end
valid = true(nH, 1);                            % hw samples with fresh measurements (logical)
for k = 1:size(dro, 1), valid(H.t > dro(k, 1) & H.t <= dro(k, 2)) = false; end
rep.n_excluded = nnz(~valid);
rep.arm_not_ready = false_intervals(H.t, H.arm_ready);

% ---------- hardware-internal: measured vs commanded
if ~isempty(H.wheel_meas) && ~isempty(H.wheel_cmd)
    em = abs(abs(H.wheel_meas(valid, :)) - abs(H.wheel_cmd(valid, :)));
    rep.wheel.hw_meas_vs_cmd = stats(em, H.t(valid));
    big = abs(H.wheel_cmd(valid, :)) > 1;       % commanded clearly non-zero (rpm)
    rep.wheel.echo_sign_mismatch = sum(big & sign(H.wheel_meas(valid, :)) ~= sign(H.wheel_cmd(valid, :)), 1);
else
    rep.wheel.hw_meas_vs_cmd = stats([], []);  rep.wheel.echo_sign_mismatch = NaN;
end
if ~isempty(H.joint_meas) && ~isempty(H.joint_cmd)
    rep.joint.hw_meas_vs_cmd = stats(abs(H.joint_meas(valid, :) - H.joint_cmd(valid, :)), H.t(valid));
else
    rep.joint.hw_meas_vs_cmd = stats([], []);
end

% ---------- hw vs sim, on the hw time base
rep.n_overlap = 0;  rep.t_overlap = [NaN NaN];
rep.base.pos = stats([], []);  rep.base.yaw = stats([], []);  rep.ee.err = stats([], []);
rep.wheel.hw_cmd_vs_sim_cmd = stats([], []);  rep.wheel.hw_meas_vs_sim_meas = stats([], []);
rep.joint.hw_cmd_vs_sim_cmd = stats([], []);  rep.joint.hw_meas_vs_sim_meas = stats([], []);
if ~isempty(S) && nH > 0 && numel(S.t) > 1
    tolT = 1e-9;                                % time tolerance at the span ends (s)
    ov = H.t >= S.t(1) - tolT & H.t <= S.t(end) + tolT;   % hw samples inside the sim span (logical)
    tq = min(max(H.t(ov), S.t(1)), S.t(end));   % query times clamped to the span (s)
    rep.n_overlap = nnz(ov);
    if rep.n_overlap > 0, rep.t_overlap = [tq(1) tq(end)]; end
    it = @(X) interp1(S.t, X, tq, 'linear');    % sim signal at the hw times
    vo = valid(ov);                             % fresh-measurement mask inside the overlap (logical)
    to = H.t(ov);                               % overlap times (s)
    if ~isempty(H.base) && ~isempty(S.base)
        Sb = it([S.base(:, 1:2), unwrap(S.base(:, 3))]);
        Hb = H.base(ov, :);
        rep.base.pos = stats(vecnorm(Hb(:, 1:2) - Sb(:, 1:2), 2, 2), to);
        dyaw = Hb(:, 3) - Sb(:, 3);             % heading difference (rad)
        yaw_e = atan2(sin(dyaw), cos(dyaw));    % wrapped to (-pi, pi] (rad)
        rep.base.yaw = stats(abs(yaw_e), to);
        if ~isempty(H.joint_meas) && ~isempty(S.joint_meas)
            Sj = it(S.joint_meas);  Hj = H.joint_meas(ov, :);
            eh = ee_rows([Hb(vo, :), Hj(vo, :)]);
            es = ee_rows([Sb(vo, :), Sj(vo, :)]);
            rep.ee.err = stats(vecnorm(eh - es, 2, 2), to(vo));
            rep.ee.frame = 'model';             % ik9_fk placeholder DH, not the physical AR3 tool tip
            rep.notes{end+1} = ['ee.err is the MODEL-FRAME end effector (ik9_fk, PLACEHOLDER DH table), not the ' ...
                                'physical AR3 tool tip (analysis/commanding_check.m); same kinematics on both ' ...
                                'sides, so hw vs sim stays comparable'];
        end
    end
    if ~isempty(H.wheel_cmd) && ~isempty(S.wheel_cmd)
        rep.wheel.hw_cmd_vs_sim_cmd = stats(abs(H.wheel_cmd(ov, :) - it(S.wheel_cmd)), to);
    end
    if ~isempty(H.wheel_meas) && ~isempty(S.wheel_meas)
        Hw = H.wheel_meas(ov, :);  Sw = it(S.wheel_meas);
        rep.wheel.hw_meas_vs_sim_meas = stats(abs(abs(Hw(vo, :)) - abs(Sw(vo, :))), to(vo));
    end
    if ~isempty(H.joint_cmd) && ~isempty(S.joint_cmd)
        rep.joint.hw_cmd_vs_sim_cmd = stats(abs(H.joint_cmd(ov, :) - it(S.joint_cmd)), to);
    end
    if ~isempty(H.joint_meas) && ~isempty(S.joint_meas)
        Hj = H.joint_meas(ov, :);  Sj = it(S.joint_meas);
        rep.joint.hw_meas_vs_sim_meas = stats(abs(Hj(vo, :) - Sj(vo, :)), to(vo));
    end
end

% ---------- per-line send times: the Task 1c open-loop deficit
if ~isempty(H.send)
    rep.send = send_deficit(H.send, P, H.q_cmd, H.t);
else
    rep.send = struct('n_lines', 0, 'note', 'no per-line send times in this run');
end
end

% =====================================================================================================
function R = canon_run(x)
% Canonical run struct from a save_run file struct (fields out, snap) or a canonical struct (field t).
R = [];
if isempty(x), return; end
if isstruct(x) && isfield(x, 'out')
    snap = struct();
    if isfield(x, 'snap'), snap = x.snap; end
    R = out_to_run(x.out, snap);
elseif isstruct(x) && isfield(x, 't')
    R = x;
else
    error('compare_hw_sim_core:input', 'a run must be a save_run file struct (out, snap) or a canonical struct (t, ...)');
end
dflt = {'base', []; 'base_source', 'not given'; 'base_measured', false; 'wheel_cmd', []; 'wheel_meas', []; ...
        'joint_cmd', []; 'joint_meas', []; 'joint_guard', []; 'arm_ready', []; 'has_tel', false; 'tel_t', []; ...
        'send', []; 'q_cmd', []; 'frozen', NaN};
for k = 1:size(dflt, 1)
    if ~isfield(R, dflt{k, 1}), R.(dflt{k, 1}) = dflt{k, 2}; end
end
R.t = R.t(:);
end

function R = out_to_run(o, snap)
% Logged signals (To Workspace names) + snapshot -> canonical run struct. See the header for the mapping.
R = struct();
[R.t, R.q_cmd] = sig(o, 'q_cmd');
hwMode = isequal(getf(snap, 'EnableHardware', 0), 1);
mot = isequal(getf(snap, 'EnableMotive', 0), 1);
src = char(string(getf(snap, 'motive_source', 'absent')));
R.frozen = getf(snap, 'arm_freeze_user', NaN);
[~, qm] = sig(o, 'q_meas');
R.base_measured = false;
if mot && has(o, 'q_track')
    [~, qt] = sig(o, 'q_track');
    R.base = qt(:, 1:3);
    if strcmpi(src, 'emulator')
        R.base_source = 'camera pipeline fed by the EMULATOR (model prediction, mode 2): not a measurement';
    else
        R.base_source = sprintf('camera (q_track, EnableMotive 1, motive_source %s): measured', src);
        R.base_measured = true;
    end
elseif ~isempty(qm)
    R.base = qm(:, 1:3);
    R.base_source = 'q_meas = SensorModel of q_cmd one step late (EnableMotive 0): model state, not a measurement';
else
    R.base = [];  R.base_source = 'none logged';
end
[~, R.wheel_cmd] = sig(o, 'wheel_sent');
[~, R.joint_cmd] = sig(o, 'joint_sent_rad');
[~, R.joint_guard] = sig(o, 'joint_guard_rad');
if hwMode
    [~, R.wheel_meas] = sig(o, 'wheel_meas');
    [~, jd] = sig(o, 'joint_meas_deg');
    R.joint_meas = deg2rad(jd);
    [~, R.arm_ready] = sig(o, 'arm_ready');
else
    R.wheel_meas = R.wheel_cmd;                 % ideal actuators
    if ~isempty(qm), R.joint_meas = qm(:, 4:9); else, R.joint_meas = []; end
    R.arm_ready = [];
end
R.has_tel = has(o, 'telemetry');
R.tel_t = [];
if R.has_tel
    T = get_field(o, 'telemetry');
    ln = strtrim(string(T.line(:)));
    nf = zeros(numel(ln), 1);
    for k = 1:numel(ln), nf(k) = numel(split(ln(k), ",")); end
    ok = (startsWith(ln, "GV,") & nf >= 9) | (startsWith(ln, "ARM,") & nf >= 8) | ln == "ARM_READY";
    R.tel_t = T.t(ok);  R.tel_t = R.tel_t(:);
end
R.send = [];
if has(o, 'send_log')
    L = get_field(o, 'send_log');
    ln = strtrim(string(L.line(:)));
    isR = startsWith(ln, "ROME,");
    S = struct();
    S.ts = L.ts(isR);  S.ts = S.ts(:);  S.tw = L.tw(isR);  S.tw = S.tw(:);
    iStop = find(ln == "STOP_ALL");
    S.tw_stop = NaN;
    if isscalar(iStop), S.tw_stop = L.tw(iStop); end
    S.wheel = NaN(nnz(isR), 4);
    S.wheel_source = 'ROME line text (%.3f)';
    idx = find(isR);
    for k = 1:numel(idx)
        p = split(ln(idx(k)), ",");
        S.wheel(k, :) = str2double(p(2:5)).';
    end
    if ~isempty(R.wheel_cmd) && ~isempty(R.t)
        [hit, loc] = ismembertol(S.ts, R.t, 1e-9, 'DataScale', 1);   % each line at a logged step?
        if all(hit), S.wheel = R.wheel_cmd(loc, :);  S.wheel_source = 'logged wheel_sent'; end
    end
    q0 = getf(snap, 'q0_9dof', NaN(3, 1));
    S.q0 = q0(1:3);  S.q0 = S.q0(:);
    S.dt = getf(snap, 'dt_9dof', NaN);  S.r = getf(snap, 'r', NaN);  S.l = getf(snap, 'l', NaN);
    S.alphas = getf(snap, 'alphas', NaN(1, 4));
    R.send = S;
end
end

function [dro, src] = detect_dropouts(H, P)
% Telemetry dropout intervals [start end] (s). Primary: accepted-line times; fallback: arm_ready falling edges.
dro = zeros(0, 2);  gap = P.tel_gap_s;  t0 = H.t(1);  t1 = H.t(end);
if H.has_tel
    src = 'telemetry line times (gap > tel_gap_s between accepted lines)';
    tt = sort(H.tel_t(:));
    if isempty(tt), dro = [t0 t1]; return; end
    if tt(1) - t0 > gap, dro(end+1, :) = [t0 tt(1)]; end
    k = find(diff(tt) > gap);
    dro = [dro; tt(k) tt(k+1)];
    if t1 - tt(end) > gap, dro(end+1, :) = [tt(end) t1]; end
elseif ~isempty(H.arm_ready)
    src = ['arm_ready falling edge: dropout OR arm not READY (an ARM,...,BUSY line), not separable from the ' ...
           'logged signals; start = edge - tel_gap_s'];
    ar = H.arm_ready(:) > 0.5;
    fe = find(~ar(2:end) & ar(1:end-1)) + 1;    % falling edges (index)
    for f = fe.'
        re = find(ar(f:end), 1);                % next rising edge (offset)
        if isempty(re), te = t1; else, te = H.t(f + re - 1); end
        dro(end+1, :) = [H.t(f) - gap, te];     %#ok<AGROW>
    end
else
    src = 'none available (no telemetry line times, no arm_ready)';
end
end

function I = false_intervals(t, a)
% K x 2 [start end] (s) of the runs where logical a is false.
I = zeros(0, 2);
if isempty(a), return; end
a = a(:) > 0.5;  d = diff([true; a; true]);
s = find(d == -1);  e = find(d == 1) - 1;
I = [t(s), t(e)];
end

function D = send_deficit(S, P, q_cmd, t_q)
% Task 1c method on the per-line send times (see the header). Units: m, rad, s; deficits reported in mm / deg.
% rome_wheels exists only as a LOCAL function (ukd_current.m L817, ukd_fixed.m, rome_uk_block.m, mass_at.m L587; no
% standalone file), so it is reached through mass_at's handle; mass_at's copy is byte-identical to ukd_current's
% (diff in the F3 core report, f3core-report.md). The full mass_at evaluation at q = 0 is the price of that handle.
[~, ~, ~, ~, ~, ~, ~, PB] = mass_at(zeros(9, 1), zeros(9, 1));
wm = PB.wheelmap;                              % rome_wheels(q, qd, r, l, alpha) of the block (rad/s)
Mw = zeros(4, 3);                               % body velocity [vx_B vy_B thetadot] -> wheel rad/s, at yaw 0
for c = 1:3
    ec = zeros(3, 1);  ec(c) = 1;
    Mw(:, c) = wm([0; 0; 0], ec, S.r, S.l, S.alphas(:));
end
N = numel(S.ts);
D = struct('n_lines', N, 'wheel_source', 'given');
if isfield(S, 'wheel_source'), D.wheel_source = S.wheel_source; end
if N < 2, D.note = 'fewer than 2 ROME lines'; return; end
wr = S.wheel * (2*pi/60);                       % N x 4 wheel speeds (rad/s)
vb = (pinv(Mw) * wr.').';                       % N x 3 least-squares body velocity (m/s, m/s, rad/s)
D.fit_res_rpm = max(abs(Mw * vb.' - wr.'), [], 'all') * 60/(2*pi);
h = diff(S.tw(:));                              % N-1 actual holds (s)
pN = zeros(N, 3);  pA = zeros(N, 3);            % nominal / actual base pose at the end of hold k-1
pN(1, :) = S.q0(1:3).';  pA(1, :) = pN(1, :);
for k = 1:N-1
    for c = 1:2                                 % 1 nominal (dt), 2 actual (h_k)
        if c == 1, p = pN(k, :); hk = S.dt; else, p = pA(k, :); hk = h(k); end
        th = p(3);                              % heading at the start of the hold (rad)
        vw = [cos(th) -sin(th); sin(th) cos(th)] * vb(k, 1:2).';   % world velocity (m/s)
        p = p + [vw.' vb(k, 3)] * hk;
        if c == 1, pN(k+1, :) = p; else, pA(k+1, :) = p; end
    end
end
dxy = vecnorm(pA(:, 1:2) - pN(:, 1:2), 2, 2) * 1e3;   % position deficit (mm)
dyaw = rad2deg(pA(:, 3) - pN(:, 3));            % heading deficit (deg)
ts = S.ts(:);  tw = S.tw(:);
lag = (tw - tw(1)) - (ts - ts(1));              % wall minus simulated elapsed time (s)
[lagMin, kL] = min(lag);
kS = find(lag <= P.win_frac * lagMin, 1);       % end of the catch-up window (index)
[dmax, kM] = max(dxy);
D.lag = lag;  D.lag_min = lagMin;  D.t_lag_min = ts(kL);  D.t_window_end = ts(kS);
D.dxy_window_mm = dxy(kS);  D.dxy_end_mm = dxy(N);  D.dxy_max_mm = dmax;  D.t_dxy_max = ts(kM);
D.dyaw_window_deg = dyaw(kS);  D.dyaw_end_deg = dyaw(N);  D.dyaw_maxabs_deg = max(abs(dyaw));
D.path_m = sum(vecnorm(diff(pN(:, 1:2)), 2, 2));
D.hold_median_s = median(h);  D.hold_min_s = min(h);  D.hold_max_s = max(h);
D.last_hold_s = S.tw_stop - tw(N);              % NaN without a STOP_ALL record
D.sanity_qcmd_mm = NaN;                         % nominal path vs the model's own q_cmd (1c sanity check)
if ~isempty(q_cmd) && ~isempty(t_q)
    [hit, loc] = ismembertol(ts, t_q, 1e-9, 'DataScale', 1);
    if all(hit)
        Q = q_cmd(loc, 1:2);                    % q_cmd at line k is the state for t_{k+1}
        D.sanity_qcmd_mm = max(vecnorm(pN(2:end, 1:2) - Q(1:end-1, :), 2, 2)) * 1e3;
    end
end
D.pN = pN;  D.pA = pA;  D.dxy_mm = dxy;  D.dyaw_deg = dyaw;
end

function s = stats(e, t)
% Column-wise stats of absolute errors e (n x k) at times t (n x 1).
if isempty(e)
    s = struct('max', NaN, 'rms', NaN, 'start', NaN, 'end', NaN, 't_max', NaN, 'n', 0);
    return;
end
[mx, i] = max(e, [], 1);
s = struct('max', mx, 'rms', sqrt(mean(e.^2, 1)), 'start', e(1, :), 'end', e(end, :), ...
           't_max', reshape(t(i), 1, []), 'n', size(e, 1));
end

function P = ee_rows(Q)
% End-effector position (m) of every row of Q (n x 9) through the formulation's kinematics, ik9_fk.
P = zeros(size(Q, 1), 3);
for k = 1:size(Q, 1), P(k, :) = ik9_fk(Q(k, :).').'; end
end

function [t, X] = sig(o, n)
% Time (N x 1 s) and data (N x k) of a logged signal; [] when absent. Handles N x k and k x 1 x N layouts.
t = [];  X = [];
if ~has(o, n), return; end
x = get_field(o, n);
t = x.Time(:);  X = double(x.Data);  N = numel(t);
if ndims(X) == 3 && size(X, 3) == N
    X = reshape(permute(X, [3 1 2]), N, []);
elseif size(X, 1) ~= N && size(X, 2) == N
    X = X.';
end
end

function tf = has(o, n)
% Field test for structs and for Simulink.SimulationOutput (NOT VERIFIED on a real SimulationOutput yet).
if isstruct(o), tf = isfield(o, n);
elseif isa(o, 'Simulink.SimulationOutput'), tf = any(strcmp(o.who, n));
else, tf = isprop(o, n);
end
end

function v = get_field(o, n)
if isa(o, 'Simulink.SimulationOutput'), v = o.get(n); else, v = o.(n); end
end

function v = getf(s, n, d)
% Field n of struct s, or default d.
if isstruct(s) && isfield(s, n) && ~(ischar(s.(n)) && strcmp(s.(n), 'MISSING')), v = s.(n); else, v = d; end
end
