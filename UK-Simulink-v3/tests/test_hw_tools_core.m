function test_hw_tools_core()
%TEST_HW_TOOLS_CORE  Offline tests of the F3 core (focus plan F3, Plan 3 Task 3 pure part, 19 Nov 2026):
%   save_run.m (run recorder) and compare_hw_sim_core.m (measured vs simulated comparison).
%   PURE: no model, no define_constants, no sim(), no network. Data: ROME_merged/runs/case_1.mat is LOADED only
%   (t, q = q_cmd, w = wheel_speeds_rpm, QM = q_meas of the scenario 1 sim run); everything else is synthetic.
%   Files are written only under tempname and deleted at the end.
%   A "fake out" below is a struct whose fields mimic the model's To Workspace Timeseries (Time, Data), with
%   the names of the current slx (docs/verification/2026-11-19_f3core_wiring.log). One field (wheel_meas of the
%   hardware run) is stored k x 1 x N, as a Timeseries of a column signal can be, to exercise that layout.
%   Cases
%     (i)   a "hardware" run made from the sim run (EnableHardware 1, wheel_meas = wheel_sent, joint_meas_deg =
%           q_meas joints in degrees) against the sim run itself (EnableHardware 0, whose wheel_meas is zeros and
%           joint_meas_deg is constant, as ROMECommand outputs without a connection): every hw-vs-sim difference
%           < 1e-9, directly and after a save_run / load round trip; no dropout.
%     (ii)  save_run twice with the same run stamp (same second) gives two files, the second with suffix _n2,
%           the first file's content unchanged; two calls on the real clock also give two files.
%     (iii) telemetry stops halfway (lines only up to t_half, measured values held after it, arm_ready false
%           0.5 s later): the run is still saved; compare reports ONE dropout starting at t_half (from the line
%           times), excludes the stale samples from the measured-vs-commanded stats; without line times it falls
%           back to the arm_ready edge and says the cause is not separable; lines that keep arriving while
%           arm_ready is false (arm BUSY) are NOT a dropout.
%     (iv)  camera base perturbed by [0.03 0.04] m (5 cm): base position error 0.05 m at the start and
%           everywhere, heading error 0, end-effector error 0.05 m (base translation); a 0.02 rad heading
%           perturbation gives heading error 0.02 rad and position error 0; mode 2 (emulator) is labelled as
%           not a measurement; ee.err is labelled model-frame (ik9_fk placeholder DH) in rep.notes; a heading
%           that crosses +-pi with hw 0.02 rad ahead gives a 0.02 rad error, not 2 pi.
%     (ii)  also: an output folder whose path contains "OneDrive" raises warning save_run:onedrive, a local one
%           does not.
%     (v)   per-line send times. (a) known profile: pure body translation 0.2 m/s, first 90 holds 7 % short:
%           lag min -0.315 s, deficit 63 mm at the end and 60.2 mm at the 95 % window end, nominal path ends at
%           +1.99 m in x. Wheels from a formula written HERE (+sin map, define_constants alphas 315/225/135/45 deg,
%           r 0.0762 m, l 0.35 m, define_constants.m L173-184), not from the code under test. (b) a rotating
%           profile written as a Task 1c record and run through tools/p2t1c_pacing_deficit.m (the 1c method
%           itself): the printed lag, deficits and heading deficits equal compare_hw_sim_core's to print precision.
%   Errors with test_hw_tools_core:fail and a message starting 'FAIL:' listing every failed check.
%   Make-it-fail (source level): tests/manual_makefail_hw_tools_core.m.
here = fileparts(mfilename('fullpath'));                  % tests folder (char)
root = fileparts(here);                                    % ROME_merged (char)
addpath(root, fullfile(root, 'tools'));
fprintf('test_hw_tools_core: save_run = %s\n  compare_hw_sim_core = %s\n', which('save_run'), which('compare_hw_sim_core'));
D = load(fullfile(root, 'runs', 'case_1.mat'), 't', 'q', 'w', 'QM');
t = D.t(:);  N = numel(t);                                 % sample times (s), count
tmp = tempname;  mkdir(tmp);                               % scratch folder for run files (char)
cln = onCleanup(@() rmdir(tmp, 's'));
F = {};                                                    % failed checks (cellstr)
TOL = 1e-9;                                                % "same run" tolerance (m, rad, rpm)

snapS = mk_snap(0, 0, 'absent');                           % sim run snapshot
snapH = mk_snap(1, 0, 'absent');                           % hardware, no camera
outS = fake_out(D, false);                                 % sim run's logged signals
outH = fake_out(D, true);                                  % "hardware" run made from the sim run
runS = struct('out', outS, 'snap', snapS);
runH = struct('out', outH, 'snap', snapH);

% ---------- (i) hardware-from-sim vs sim
rep = compare_hw_sim_core(runH, runS, []);
F = ck(F, rep.n_overlap == N, sprintf('(i) overlap %d samples of %d', rep.n_overlap, N));
d = [rep.base.pos.max, rep.base.yaw.max, rep.ee.err.max, max(rep.wheel.hw_cmd_vs_sim_cmd.max), ...
     max(rep.wheel.hw_meas_vs_sim_meas.max), max(rep.joint.hw_cmd_vs_sim_cmd.max), max(rep.joint.hw_meas_vs_sim_meas.max)];
F = ck(F, all(isfinite(d)) && all(d < TOL), sprintf('(i) hw-vs-sim differences < 1e-9: %s', mat2str(d, 3)));
F = ck(F, rep.dropout.n == 0, sprintf('(i) no dropout reported (n = %d)', rep.dropout.n));
f1 = save_run(outH, snapH, 'selfcmp', tmp);
L = load(f1);
rep2 = compare_hw_sim_core(L, runS, []);
d2 = [rep2.base.pos.max, rep2.base.yaw.max, rep2.ee.err.max, max(rep2.joint.hw_meas_vs_sim_meas.max), max(rep2.wheel.hw_meas_vs_sim_meas.max)];
F = ck(F, rep2.n_overlap == N && all(d2 < TOL), sprintf('(i) after save_run/load round trip < 1e-9: %s', mat2str(d2, 3)));
F = ck(F, isfield(L, 'snap') && isequal(L.snap.dc_sha_at_run, snapH.dc_sha_at_run) && isequal(L.snap.user, snapH.user), ...
       '(i) snapshot saved with the run (dc_sha_at_run, user block)');

% ---------- (ii) no overwrite in the same second
snap2 = snapH;  snap2.run_stamp = '2026-11-19_120000';
oA = struct('marker', 1);  oB = struct('marker', 2);
fa = save_run(oA, snap2, 'dup', tmp);
fb = save_run(oB, snap2, 'dup', tmp);
[~, na] = fileparts(fa);  [~, nb] = fileparts(fb);
F = ck(F, strcmp(na, '2026-11-19_120000_s1_dup') && strcmp(nb, '2026-11-19_120000_s1_dup_n2'), ...
       sprintf('(ii) names %s, %s', na, nb));
La = load(fa);  Lb = load(fb);
F = ck(F, La.out.marker == 1 && Lb.out.marker == 2, sprintf('(ii) first file kept its content (markers %d, %d)', La.out.marker, Lb.out.marker));
fc = save_run(oA, snapH, 'clock', tmp);  fd = save_run(oB, snapH, 'clock', tmp);   % real clock, back to back
Lc = load(fc);  Ld = load(fd);
F = ck(F, ~strcmp(fc, fd) && isfile(fc) && isfile(fd) && Lc.out.marker == 1 && Ld.out.marker == 2, ...
       '(ii) two real-clock calls give two files');
F = ck(F, ~isempty(regexp(fc, '\d{4}-\d{2}-\d{2}_\d{6}_s1_clock(_n\d+)?\.mat$', 'once')), sprintf('(ii) name pattern: %s', fc));
lastwarn('');  fo = save_run(oA, snapH, 'od', fullfile(tmp, 'OneDrive_fake'));  [~, idOD] = lastwarn;   % synced-folder warning
lastwarn('');  fn = save_run(oA, snapH, 'local', tmp);  [~, idL] = lastwarn;                         % local folder: none
F = ck(F, strcmp(idOD, 'save_run:onedrive') && isempty(idL) && isfile(fo) && isfile(fn), ...
       sprintf('(ii) OneDrive warning: "%s" for a OneDrive path, "%s" for a local one', idOD, idL));

% ---------- (iii) telemetry stops halfway
h = floor(N/2);  tHalf = t(h);                             % last sample with telemetry (index, s)
oD = fake_out(D, true);
wm = squeeze(oD.wheel_meas.Data).';  wm(h+1:end, :) = repmat(wm(h, :), N-h, 1);   % held (ROMECommand L76-82)
oD.wheel_meas.Data = wm;
jm = oD.joint_meas_deg.Data;  jm(h+1:end, :) = repmat(jm(h, :), N-h, 1);
oD.joint_meas_deg.Data = jm;
oD.arm_ready.Data = t <= tHalf + 0.5;                      % ROMECommand: false after 0.5 s without a line (L206-208)
oD.telemetry = tel_lines(t(1:h), D.w(1:h, :), rad2deg(D.QM(1:h, 4:9)));
oD.telemetry.t(end+1, 1) = t(h+3);  oD.telemetry.tw(end+1, 1) = t(h+3);  oD.telemetry.line(end+1, 1) = "GV,1,2,3";   % malformed: not telemetry
fD = save_run(oD, snapH, 'dropout', tmp);
F = ck(F, isfile(fD), '(iii) dropout run saved');
LD = load(fD);
F = ck(F, numel(LD.out.telemetry.line) == 2*h + 1, sprintf('(iii) saved telemetry lines %d (expected %d)', numel(LD.out.telemetry.line), 2*h + 1));
rD = compare_hw_sim_core(LD, runS, []);
F = ck(F, rD.dropout.n == 1 && abs(rD.dropout.t_start(1) - tHalf) < TOL && abs(rD.dropout.t_end(1) - t(end)) < TOL, ...
       sprintf('(iii) one dropout from t_half = %.3f s: n %d, start %s', tHalf, rD.dropout.n, mat2str(rD.dropout.t_start, 6)));
F = ck(F, contains(rD.dropout.source, 'telemetry line'), ['(iii) source: ' rD.dropout.source]);
F = ck(F, rD.n_excluded == N - h && max(rD.wheel.hw_meas_vs_cmd.max) < TOL, ...
       sprintf('(iii) stale samples excluded: %d excluded, wheel |meas|-|cmd| max %.3g rpm', rD.n_excluded, max(rD.wheel.hw_meas_vs_cmd.max)));
oE = rmfield(oD, 'telemetry');                             % no line times: fall back to arm_ready
rE = compare_hw_sim_core(struct('out', oE, 'snap', snapH), runS, []);
kF = find(t > tHalf + 0.5, 1);                             % first sample with arm_ready false (index)
F = ck(F, rE.dropout.n == 1 && abs(rE.dropout.t_flag(1) - t(kF)) < TOL && contains(rE.dropout.source, 'not separable'), ...
       sprintf('(iii) arm_ready fallback: n %d, flag %s (expected %.3f)', rE.dropout.n, mat2str(rE.dropout.t_flag, 6), t(kF)));
oB2 = fake_out(D, true);                                   % lines keep coming, arm BUSY 5-7 s
oB2.telemetry = tel_lines(t, D.w, rad2deg(D.QM(:, 4:9)));
oB2.arm_ready.Data = ~(t >= 5 & t < 7);
rB = compare_hw_sim_core(struct('out', oB2, 'snap', snapH), runS, []);
F = ck(F, rB.dropout.n == 0 && size(rB.arm_not_ready, 1) == 1, sprintf('(iii) BUSY is not a dropout: n %d, not-ready intervals %d', rB.dropout.n, size(rB.arm_not_ready, 1)));

% ---------- (iv) perturbed start, camera base
snapC = mk_snap(1, 1, 'optitrack');
oC = fake_out(D, true);
oC.q_track = struct('Time', t, 'Data', D.QM(:, 1:3) + [0.03 0.04 0]);   % camera base 5 cm off, q_meas untouched
rC = compare_hw_sim_core(struct('out', oC, 'snap', snapC), runS, []);
F = ck(F, abs(rC.base.pos.start - 0.05) < TOL && abs(rC.base.pos.max - 0.05) < TOL && abs(rC.base.pos.rms - 0.05) < TOL, ...
       sprintf('(iv) base position error 0.05 m: start %.12f, max %.12f', rC.base.pos.start, rC.base.pos.max));
F = ck(F, rC.base.yaw.max < TOL, sprintf('(iv) heading error 0 for a pure translation: %.3g', rC.base.yaw.max));
F = ck(F, abs(rC.ee.err.max - 0.05) < TOL && abs(rC.ee.err.start - 0.05) < TOL, sprintf('(iv) end-effector error 0.05 m: %.12f', rC.ee.err.max));
F = ck(F, rC.hw_base_measured, ['(iv) camera base reported as measured: ' rC.hw_base_source]);
oY = fake_out(D, true);
oY.q_track = struct('Time', t, 'Data', D.QM(:, 1:3) + [0 0 0.02]);
rY = compare_hw_sim_core(struct('out', oY, 'snap', snapC), runS, []);
F = ck(F, abs(rY.base.yaw.max - 0.02) < TOL && abs(rY.base.yaw.start - 0.02) < TOL && rY.base.pos.max < TOL && rY.ee.err.max > 1e-3, ...
       sprintf('(iv) heading 0.02 rad: yaw %.12f, pos %.3g, ee %.4f', rY.base.yaw.max, rY.base.pos.max, rY.ee.err.max));
rM = compare_hw_sim_core(struct('out', oY, 'snap', mk_snap(1, 1, 'emulator')), runS, []);
F = ck(F, ~rM.hw_base_measured && contains(rM.hw_base_source, 'EMULATOR'), ['(iv) mode 2 base not a measurement: ' rM.hw_base_source]);
F = ck(F, isfield(rC.ee, 'frame') && strcmp(rC.ee.frame, 'model') && any(contains(rC.notes, 'MODEL-FRAME end effector')) ...
       && any(contains(rC.notes, 'PLACEHOLDER')), '(iv) ee.err labelled model-frame (ik9_fk placeholder DH) in rep.notes');
% heading wrap at +-pi: the sim heading crosses +pi (stored wrapped to (-pi, pi]), hw is 0.02 rad ahead
tw_ = (0:0.05:2).';  yu = pi - 0.1 + 0.1*tw_;              % unwrapped sim heading (rad), crosses pi at t = 1 s
wrapf = @(a) atan2(sin(a), cos(a));                        % wrap to (-pi, pi]
simW = struct('t', tw_, 'base', [zeros(numel(tw_), 2), wrapf(yu)]);
hwW = struct('t', tw_, 'base', [zeros(numel(tw_), 2), wrapf(yu + 0.02)]);
rW = compare_hw_sim_core(hwW, simW, []);
F = ck(F, any(abs(diff(hwW.base(:, 3))) > pi) && abs(rW.base.yaw.max - 0.02) < TOL && abs(rW.base.yaw.end - 0.02) < TOL, ...
       sprintf('(iv) heading wrap at +-pi: yaw error max %.12f rad (0.02 expected, not 2 pi)', rW.base.yaw.max));

% ---------- (v) per-line send times and the 1c deficit
dt = 0.05;  r = 0.0762;  l = 0.35;  al = deg2rad([315 225 135 45]);   % define_constants.m L189, L173, L174, L184
wheels = @(vb) ((sin(al(:))*vb(1) + cos(al(:))*vb(2) + l*vb(3)) / r * 60/(2*pi)).';   % +sin map (rpm, 1x4)
% (a) pure translation, analytic
Na = 200;  ha = dt*ones(Na-1, 1);  ha(1:90) = 0.93*dt;     % holds (s); 90 short holds: the 95 % point (line 87) is not a tie
Sa = struct('ts', (0:Na-1).'*dt, 'tw', [0; cumsum(ha)], 'wheel', repmat(wheels([0.2 0 0]), Na, 1), ...
            'tw_stop', NaN, 'q0', [0.1; -0.2; 0], 'dt', dt, 'r', r, 'l', l, 'alphas', al);
ra = compare_hw_sim_core(struct('t', zeros(0, 1), 'send', Sa), [], []);
s = ra.send;
% expected: lag(k) = -0.07*dt*min(k-1, 90); deficit(k) = 0.2 m/s * 0.07*dt*min(k-1, 90); 95 % of -0.315 s first at k = 87
F = ck(F, abs(s.lag_min + 0.315) < TOL, sprintf('(v-a) lag min %.12f s (expected -0.315)', s.lag_min));
F = ck(F, abs(s.dxy_end_mm - 63) < 1e-6 && abs(s.dxy_max_mm - 63) < 1e-6 && abs(s.dxy_window_mm - 60.2) < 1e-6, ...
       sprintf('(v-a) deficit end %.9f, max %.9f, window %.9f mm (63, 63, 60.2)', s.dxy_end_mm, s.dxy_max_mm, s.dxy_window_mm));
F = ck(F, norm(s.pN(end, 1:2) - [0.1 + 0.2*dt*(Na-1), -0.2]) < 1e-9, sprintf('(v-a) nominal end %s (expected [%.3f -0.2])', mat2str(s.pN(end, 1:2), 9), 0.1 + 0.2*dt*(Na-1)));
% (b) rotating profile, against tools/p2t1c_pacing_deficit.m
Nb = 300;  vb = [0.15 -0.05 0.2];                          % body velocity (m/s, m/s, rad/s)
hb = dt*ones(Nb-1, 1);  kk = (1:120).';  hb(kk) = dt*(0.93 + 0.01*sin(kk));
tsb = (0:Nb-1).'*dt;  twb = 100 + [0; cumsum(hb)];         % sim times, wall stamps (s)
Wb = repmat(wheels(vb), Nb, 1);
q0 = [0.1; -0.2; 0.3];  Q = zeros(Nb, 9);  p = q0.';         % nominal path, by the test's own Euler step
for k = 1:Nb
    c = cos(p(3));  sn = sin(p(3));
    p = p + [c*vb(1) - sn*vb(2), sn*vb(1) + c*vb(2), vb(3)]*dt;
    Q(k, 1:3) = p;                                          % state at t_{k+1} (1c convention)
end
lines = ["START_GV" + newline; compose("ROME,%.3f,%.3f,%.3f,%.3f,0,0,0,0,0,0", Wb) + newline; "STOP_ALL" + newline];
twS = twb(end) + 0.0076;
send_log = struct('ts', [0; tsb; tsb(end)], 'tw', [twb(1) - 0.5; twb; twS], 'line', lines);
ob = struct('q_cmd', struct('Time', tsb, 'Data', Q), 'wheel_sent', struct('Time', tsb, 'Data', Wb), 'send_log', send_log);
snapB = mk_snap(1, 0, 'absent');  snapB.q0_9dof = [q0; zeros(6, 1)];  snapB.dt_9dof = dt;  snapB.r = r;  snapB.l = l;  snapB.alphas = al;
rb = compare_hw_sim_core(struct('out', ob, 'snap', snapB), [], []);
sb = rb.send;
F = ck(F, strcmp(sb.wheel_source, 'logged wheel_sent') && sb.n_lines == Nb && sb.sanity_qcmd_mm < 1e-6, ...
       sprintf('(v-b) %d lines, wheels from %s, nominal vs q_cmd %.3g mm', sb.n_lines, sb.wheel_source, sb.sanity_qcmd_mm));
R1 = struct('K', {repmat({'W'}, numel(lines), 1)}, 'TS', send_log.ts, 'TW', send_log.tw, 'TX', {cellstr(lines)}, ...
            't', tsb, 'W', Wb, 'Q', Q, 'q0_9dof', [q0; zeros(6, 1)], 'dt_9dof', dt, 'r', r, 'l', l, 'alphas', al);
mf = fullfile(tmp, 'p2t1c_record.mat');  save(mf, '-struct', 'R1');
txt = evalc('p2t1c_pacing_deficit(mf)');
fprintf('%s', txt);
tl = regexp(txt, 'min (\S+) s at t', 'tokens', 'once');
td = regexp(txt, 'catch-up window \(t = \S+ s\) (\S+) mm; at the end of the lap \(t = \S+ s\) (\S+) mm; max (\S+) mm', 'tokens', 'once');
th = regexp(txt, 'end of window (\S+) deg, end of lap (\S+) deg, max \|\.\| (\S+) deg', 'tokens', 'once');
F = ck(F, numel(tl) == 1 && numel(td) == 3 && numel(th) == 3, '(v-b) p2t1c printout parsed');
if numel(tl) == 1 && numel(td) == 3 && numel(th) == 3
    v1c = str2double([tl, td, th]);                        % 1c numbers: lag (s), deficits (mm), heading (deg)
    vme = [sb.lag_min, sb.dxy_window_mm, sb.dxy_end_mm, sb.dxy_max_mm, sb.dyaw_window_deg, sb.dyaw_end_deg, sb.dyaw_maxabs_deg];
    tol = [5e-5, 5e-3, 5e-3, 5e-3, 5e-5, 5e-5, 5e-5] + 1e-9;   % half a unit of each printed decimal
    F = ck(F, all(abs(v1c - vme) <= tol) && sb.dxy_max_mm > 1, sprintf('(v-b) 1c %s vs core %s', mat2str(v1c, 7), mat2str(vme, 7)));
end
F = ck(F, abs(sb.last_hold_s - 0.0076) < TOL, sprintf('(v-b) last line held %.6f s before STOP_ALL', sb.last_hold_s));

if ~isempty(F)
    error('test_hw_tools_core:fail', 'FAIL: %d check(s): %s', numel(F), strjoin(F, ' | '));
end
fprintf('test_hw_tools_core: PASS (all checks)\n');
end

function F = ck(F, ok, msg)
% Records one check: prints PASS/FAIL and appends a failure to F.
if ok, fprintf('  PASS %s\n', msg);
else, fprintf('  FAIL %s\n', msg);  F{end+1} = msg; end
end

function s = mk_snap(hw, mot, src)
% A run snapshot as collect_run_snapshot would build it (values synthetic, fields real).
s = struct('scenario', 1, 'user', struct('scenario', 1, 'EnableHardware', hw, 'EnableMotive', mot, 'max_rpm', 80), ...
           'dc_sha_at_run', 'sha-test', 'repo_commit', 'commit-test', 'matlab_release', version, ...
           'EnableHardware', hw, 'EnableMotive', mot, 'motive_source', src, 'arm_freeze_user', 0, ...
           'q0_9dof', zeros(9, 1), 'dt_9dof', 0.05, 'r', 0.0762, 'l', 0.35, 'alphas', deg2rad([315 225 135 45]));
end

function o = fake_out(D, hwMode)
% To Workspace-like struct of the case_1 sim run. hwMode true: the measured channels carry the sim's values
% (a "perfect robot"); false: what ROMECommand outputs with no connection (zeros, constant joints, not ready).
t = D.t(:);  N = numel(t);
ts = @(X) struct('Time', t, 'Data', X);
o = struct();
o.q_cmd = ts(D.q);  o.q_meas = ts(D.QM);  o.wheel_speeds_rpm = ts(D.w);  o.wheel_sent = ts(D.w);
o.joint_guard_rad = ts(D.q(:, 4:9));  o.joint_sent_rad = ts(D.q(:, 4:9));
if hwMode
    o.wheel_meas = ts(reshape(D.w.', 4, 1, N));          % k x 1 x N layout
    o.joint_meas_deg = ts(rad2deg(D.QM(:, 4:9)));
    o.arm_ready = ts(true(N, 1));
else
    o.wheel_meas = ts(zeros(N, 4));
    o.joint_meas_deg = ts(repmat(rad2deg(D.q(1, 4:9)), N, 1));
    o.arm_ready = ts(false(N, 1));
end
end

function T = tel_lines(tt, W, J)
% Raw telemetry as the recorder would save it: one GV and one ARM line per sample time (sim s, wall s, text).
n = numel(tt);
gv = compose("GV,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", [W, W]);
arm = compose("ARM,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,READY", J);
line = reshape([gv, arm].', [], 1);
tl = reshape([tt(:), tt(:)].', [], 1);
T = struct('t', tl, 'tw', tl, 'line', line);
assert(numel(line) == 2*n);
end
