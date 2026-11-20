function p2t1c_pacing_deficit(matf)
%P2T1C_PACING_DEFICIT  Base distance deficit caused by the faster-than-real-time command stream (Plan 2 Task 1c,
%   fix round 1; the author asked for this number). Open-loop arithmetic on a recorded paced hardware run, no
%   simulation: matf is the file test_hw_nocam saves for its hw run (<tempdir>/test_hw_nocam_hw_record.mat):
%   record kinds K, simulation times TS (s), wall stamps TW (s), line bytes TX, logged t (s), wheel_sent W (rpm),
%   q_cmd Q (m, rad), q0_9dof, dt_9dof (s), r (m), l (m), alphas (rad) from define_constants.
%   Method:
%     1. Line k of the ROME stream is the robot's wheel command from its arrival until line k+1 arrives, i.e.
%        for h_k = TW(k+1) - TW(k) of wall time, instead of the dt the model assumed.
%     2. The commanded body velocity of line k is the least-squares inverse of the model's wheel map
%        (ukd_current.m rome_wheels: w_i = (sin a_i vx_B + cos a_i vy_B + l thetadot)/r) applied to wheel_sent.
%     3. Both streams are integrated from q0_9dof(1:3) (explicit Euler, heading at the start of each hold):
%        nominal with h = dt, actual with the recorded h_k. Deficit = actual - nominal base pose.
%   Lines 1..N-1 are compared (the last line is followed by STOP_ALL; its hold is reported separately).
%   Prints the numbers; docs/verification/2026-11-19_p2t1c_pacing_deficit.md quotes them.
S = load(matf);
lf = char(10);                                % line feed (char)
isW = strcmp(S.K, 'W');                       % write records (logical)
ln = S.TX(isW);  tw = S.TW(isW);  ts = S.TS(isW);   % lines (char), wall stamps (s), sim times (s)
isR = startsWith(ln, 'ROME,');                % ROME lines (logical)
twR = tw(isR);  tsR = ts(isR);                % their wall stamps and sim times (s)
N = numel(twR);                               % ROME lines (count)
assert(N == numel(S.t) && max(abs(tsR - S.t)) < 1e-9, 'p2t1c_pacing_deficit:align', 'lines and log do not align');
twStop = tw(strcmp(ln, ['STOP_ALL' lf]));     % STOP_ALL wall stamp (s)
assert(numel(twStop) == 1, 'p2t1c_pacing_deficit:stop', 'STOP_ALL not recorded exactly once');
dt = S.dt_9dof;                               % model step (s)
M = [sin(S.alphas(:)), cos(S.alphas(:)), S.l * ones(4, 1)] / S.r;   % 4x3 body velocity -> wheel rad/s
wr = S.W * (2*pi/60);                         % N x 4 wheel_sent (rad/s)
vb = (pinv(M) * wr.').';                      % N x 3 least-squares body velocity [vx_B vy_B (m/s), thetadot (rad/s)]
fitRes = max(abs(M * vb.' - wr.'), [], 'all') * 60/(2*pi);   % worst wheel residual of the fit (rpm)
h = diff(twR);                                % N-1 actual holds (s)
pN = zeros(N, 3);  pA = zeros(N, 3);          % nominal / actual base pose at the end of hold k-1 (m, m, rad)
pN(1,:) = S.q0_9dof(1:3).';  pA(1,:) = pN(1,:);
for k = 1:N-1
    for c = 1:2                               % c = 1 nominal (dt), 2 actual (h_k)
        if c == 1, p = pN(k,:); hk = dt; else, p = pA(k,:); hk = h(k); end
        th = p(3);                            % heading at the start of the hold (rad)
        vw = [cos(th) -sin(th); sin(th) cos(th)] * vb(k,1:2).';   % world velocity (m/s)
        p = p + [vw.' vb(k,3)] * hk;
        if c == 1, pN(k+1,:) = p; else, pA(k+1,:) = p; end
    end
end
dxy = vecnorm(pA(:,1:2) - pN(:,1:2), 2, 2) * 1e3;   % N x 1 position deficit (mm)
dyaw = rad2deg(pA(:,3) - pN(:,3));            % N x 1 heading deficit (deg)
pathN = sum(vecnorm(diff(pN(:,1:2)), 2, 2));  % nominal base path length (m)
lag = (twR - twR(1)) - (tsR - tsR(1));        % wall minus simulated elapsed time (s)
[lagMin, kL] = min(lag);                      % most ahead and where (s, index)
kS = find(lag <= 0.95 * lagMin, 1);           % first line at 95 % of the final deficit: end of the catch-up window (index)
% sanity: the nominal integration against the model's own base trajectory (q_cmd at k is the state for t_{k+1})
sanity = max(vecnorm(pN(2:end,1:2) - S.Q(1:end-1,1:2), 2, 2)) * 1e3;   % mm
fprintf('p2t1c_pacing_deficit: %s\n', matf);
fprintf('  %d ROME lines, sim %.3f s, wall %.4f s (first to last line); hold median %.4f s, min %.4f s, max %.4f s (dt %.3f s)\n', ...
        N, tsR(end) - tsR(1), twR(end) - twR(1), median(h), min(h), max(h), dt);
fprintf('  wall minus simulated: min %.4f s at t = %.2f s (line %d); 95 %% of it first reached at t = %.2f s (line %d)\n', lagMin, tsR(kL), kL, tsR(kS), kS);
fprintf('  wheel-map least-squares residual max %.3g rpm; nominal integration vs q_cmd base path max %.3f mm; nominal path length %.4f m\n', fitRes, sanity, pathN);
fprintf('  position deficit |actual - nominal|: at the end of the catch-up window (t = %.2f s) %.2f mm; at the end of the lap (t = %.2f s) %.2f mm; max %.2f mm at t = %.2f s\n', ...
        tsR(kS), dxy(kS), tsR(N), dxy(N), max(dxy), tsR(find(dxy == max(dxy), 1)));
fprintf('  heading deficit: end of window %.4f deg, end of lap %.4f deg, max |.| %.4f deg\n', dyaw(kS), dyaw(N), max(abs(dyaw)));
fprintf('  deficit at the end of the lap / nominal path length: %.3f %%\n', dxy(N) / (pathN * 1e3) * 100);
fprintf('  profile (t s: wall-minus-simulated s | position deficit mm | nominal speed mm/s):\n');
for tq = [0.5 1 2 3 4 5 6 6.7 8 10 15 20 25 31]
    [~, k] = min(abs(tsR - tq));              % line nearest tq (index)
    spd = 0;  if k < N, spd = norm(pN(k+1,1:2) - pN(k,1:2)) / dt * 1e3; end   % nominal base speed over line k (mm/s)
    fprintf('    t = %5.2f: %8.4f | %7.2f | %7.2f\n', tsR(k), lag(k), dxy(k), spd);
end
fprintf('  last ROME line (t = %.2f s, wheels %s rpm) held %.4f s wall until STOP_ALL; at that speed the base moves %.2f mm in that time\n', ...
        tsR(N), mat2str(S.W(N,:), 5), twStop - twR(N), norm(vb(N,1:2)) * (twStop - twR(N)) * 1e3);
end
