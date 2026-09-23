function derivation_checks()
%DERIVATION_CHECKS  Numerical checks of Appendix A and of the proposed
%corrections, section by section. Raises an error on the first failure.
%
%   Sections A-L check identities at the recorded start configurations.
%   Section M re-runs the failed cases with the corrections applied, using
%   the prototypes ukd_fixed.m and scenario_seeds_screened.m. ROME_9DOF.slx
%   is not modified.

here = fileparts(mfilename('fullpath'));  cd(here);  addpath(here);
dataDir = 'C:\Users\jbelmonte\OneDrive\Joao M. R. Belmonte\UCF\Research';
evalc('define_constants');                     % r, l, alphas, dt_9dof, arm_home, z_work
R1 = load(fullfile(dataDir, 'scaled_factor_0.1_scenario1.mat'));
R2 = load(fullfile(dataDir, 'scaled_factor_1.0_scenario1_semimajor0.5.mat'));
R3 = load(fullfile(dataDir, 'scaled_factor_1.0_scenario1_semimajor10.mat'));
rng(1);
tol = 1e-9;

[parV, DV, SV] = scenario_defaults(2);
[p0, u0] = EndEffectorTrajectory(0, 2, elements, mu, SV, DV, z_work, parV);
qV = ik9_warm_start([0; 0; 0.10; arm_home(:)], p0, u0);      % V-bar default start
starts = {'run 3', R3.q0_9dof; 'V-bar default', qV; 'run 2', R2.q0_9dof};

%% A. Eq. (9) equals Eq. (7); KKT gives the same result (A4-A6)
fprintf('\nA. Fundamental equation, three forms\n');
for i = 1:size(starts,1)
    [M, J] = mass_at(starts{i,2});
    a = randn(9,1);  b = randn(6,1);  c = b - J*a;
    Mh = sqrtm(M);  B = J/Mh;
    q7 = a + Mh\(pinv(B)*c);                          % Eq. (7)
    Sm = J*(M\J.');
    q9 = a + (M\J.')*(Sm\c);                          % Eq. (9)
    K  = [M, -J.'; J, zeros(6)];
    z  = K \ [M*a; b];  qk = z(1:9);                  % KKT
    s_here = min(svd(B));                             % conditioning at this start
    e79 = norm(q7 - q9)/norm(q9);  e9k = norm(q9 - qk)/norm(q9);
    fprintf('   %-14s |Eq7-Eq9|/|Eq9| = %.1e   |Eq9-KKT|/|Eq9| = %.1e   |J*qdd-b| = %.1e\n', ...
            starts{i,1}, e79, e9k, norm(J*q9 - b));
    % checked for EVERY start, not just whichever is last in the list
    check(norm(J*q9 - b) < 1e-8*max(1, norm(b)), ...
          sprintf('A: constraint not met at %s', starts{i,1}));
    if s_here > 0.30
        check(e79 < 1e-6 && e9k < 1e-6, ...
              sprintf('A: the three forms disagree at %s', starts{i,1}));
    end
end

%% B. Projector properties (A7) and posture invariance (A13)
fprintf('\nB. Projector\n');
[M, J] = mass_at(R2.q0_9dof);
Sm = J*(M\J.');  P = eye(9) - (M\J.')*(Sm\J);
dQ = randn(9,1);  dqdd = P*(M\dQ);
fprintf('   |J P| = %.1e   |P^2-P| = %.1e   |MP-(MP)''| = %.1e   |J dqdd(posture)| = %.1e\n', ...
        norm(J*P), norm(P*P - P), norm(M*P - (M*P).'), norm(J*dqdd));
check(norm(J*P) < tol && norm(P*P-P) < tol && norm(J*dqdd) < tol, 'B: projector');

%% C. Square A collapses the solve (A9)
fprintf('\nC. Square A\n');
evalc('check_square_A');                       % errors on failure
Ms = randn(9); Ms = Ms*Ms.' + 9*eye(9);  As = randn(9);  as = randn(9,1);  bs = randn(9,1);
qs = as + (Ms\As.')*((As*(Ms\As.'))\(bs - As*as));
fprintf('   9x9 random A, M: |qdd - A^-1 b| / |A^-1 b| = %.1e\n', norm(qs - As\bs)/norm(As\bs));
check(norm(qs - As\bs)/norm(As\bs) < 1e-8, 'C: square A');

%% D. Spectrum and gain bound (A8)
fprintf('\nD. lambda(S) = sigma(B)^2 and gain bound\n');
for i = 1:size(starts,1)
    [M, J] = mass_at(starts{i,2});
    Mh = sqrtm(M);  B = J/Mh;  Sm = J*(M\J.');
    sv = svd(B);  ev = sort(eig((Sm+Sm.')/2), 'descend');
    c = randn(6,1);  g = norm((M\J.')*(Sm\c));
    bound = norm(inv(Mh))*norm(c)/sv(end);
    fprintf('   %-14s max|lam-sig^2|/lam = %.1e   s = %.4f   |qdd-a| = %.3g <= %.3g\n', ...
            starts{i,1}, max(abs(ev - sv.^2)./ev), sv(end), g, bound);
    check(g <= bound*(1+1e-9), 'D: gain bound');
end

%% E. Why w misses the singularity (A16)
fprintf('\nE. w versus s\n');
fprintf('   %-14s %8s %8s %10s %10s\n', 'start', 'w', 's', 'kappa(J)^2', 'kappa(S)');
for i = 1:size(starts,1)
    [M, J] = mass_at(starts{i,2});
    Sm = J*(M\J.');
    fprintf('   %-14s %8.3f %8.3f %10.3g %10.3g\n', starts{i,1}, sqrt(det(J*J.')), ...
            sqrt(min(eig(Sm))), cond(J)^2, cond(Sm));
end
[M3, ~] = mass_at(R3.q0_9dof);
fprintf('   diag(M^-1) at run 3: %s\n', mat2str(diag(inv(M3)).', 3));

%% F. Spherical wrist: det[z3 z4 z5] = +/- sin q5 (A17)
fprintf('\nF. Wrist determinant\n');
q = R2.q0_9dof;  q5 = linspace(-pi, pi, 25);  err = 0;
for k = 1:numel(q5)
    q(8) = q5(k);  J = ik9_jac(q);
    err = max(err, abs(abs(det(J(4:6,7:9))) - abs(sin(q5(k)))));
end
Ja = ik9_jac(R3.q0_9dof);
fprintf('   max ||det| - |sin q5|| over 25 angles = %.1e;  run 3: det = %.1e, rank J_arm = %d\n', ...
        err, det(Ja(4:6,7:9)), rank(Ja(:,4:9), 1e-6));
check(err < 1e-9, 'F: wrist determinant');

%% G. Damped least squares (A14, A15)
fprintf('\nG. Damped least squares\n');
[M, J] = mass_at(R3.q0_9dof);
Mh = sqrtm(M);  B = J/Mh;  Sm = J*(M\J.');  c = randn(6,1);  lam = 0.0302;
r1 = (B.'*B + lam^2*eye(9)) \ (B.'*c);
r2 = B.' * ((Sm + lam^2*eye(6)) \ c);
s  = min(svd(B));
fprintf('   |r(form 1) - r(form 2)| = %.1e;  run 3: 1/s = %.1f, 1/(2 lambda) = %.1f\n', ...
        norm(r1 - r2), 1/s, 1/(2*lam));
check(norm(r1 - r2) < 1e-8*norm(r2), 'G: DLS forms');

%% H. Wheel map consistency (A18)
fprintf('\nH. Wheel map\n');
al = alphas(:);  W = [sin(al) cos(al) l*ones(4,1)];  n = [-1; 1; -1; 1];
w2 = squeeze(R2.out.wheel_speeds_rpm.Data);  if size(w2,1) ~= 4, w2 = w2.'; end
ws = [zeros(1, size(w2,2)); w2(1:3,:)];
fprintf('   |n''W| = %.1e;  run 2: max |n''N| sent = %.1e rpm, median |n''N| if shifted = %.1f rpm\n', ...
        norm(n.'*W), max(abs(n.'*w2)), median(abs(n.'*ws)));
check(norm(n.'*W) < tol, 'H: consistency vector');

%% I. CW control of each reference (A19)
fprintf('\nI. Clohessy-Wiltshire control\n');
nc = 1.078e-3;  t = 37.0;
refs = {2, [100 0.5 0 0], @(x,y,xd,yd) [2*nc*0.5; 0];
        3, [50 0.25 0 0], @(x,y,xd,yd) [-3*nc^2*x; -2*nc*0.25];
        4, [50 nc 0 0],   @(x,y,xd,yd) [0; 0]};
for i = 1:3
    [p, ~, V, Ad] = EndEffectorTrajectory(t, refs{i,1}, elements, mu, 1, 1, z_work, refs{i,2});
    x = p(1); y = p(2); xd = V(1); yd = V(2); xdd = Ad(1); ydd = Ad(2);
    u  = [xdd - 2*nc*yd - 3*nc^2*x; ydd + 2*nc*xd];
    ue = refs{i,3}(x, y, xd, yd);
    fprintf('   scenario %d: u = [%+.3e %+.3e], expected [%+.3e %+.3e]\n', refs{i,1}, u, ue);
    check(norm(u - ue) < 1e-12, 'I: CW control');
end

%% J. Yaw acceleration, Eq. (32) (A22)
fprintf('\nJ. Pointing law\n');
h = 1e-5;  emax = 0;
for t = [1.3 5.7 11.2 19.9]
    [p, ~, V, Ad] = EndEffectorTrajectory(t, 1, elements, mu, 0.40, 1, z_work, [0 0 0 0]);
    [~, ~, Vp] = EndEffectorTrajectory(t + h, 1, elements, mu, 0.40, 1, z_work, [0 0 0 0]);
    [~, ~, Vm] = EndEffectorTrajectory(t - h, 1, elements, mu, 0.40, 1, z_work, [0 0 0 0]);
    x = p(1); y = p(2); xd = V(1); yd = V(2); xdd = Ad(1); ydd = Ad(2); rho2 = x^2 + y^2;
    psidd = (x*ydd - y*xdd)/rho2 - 2*(x*xd + y*yd)*(x*yd - y*xd)/rho2^2;
    emax = max(emax, abs(psidd - (Vp(6) - Vm(6))/(2*h)));
end
fprintf('   max |Eq.(32) - central difference| = %.1e rad/s^2\n', emax);
check(emax < 1e-4, 'J: yaw acceleration');

%% K. Scenario 1 scaling (A21)
fprintf('\nK. Orbit scaling, D = 0.1\n');
el = [1.4 0.5 0 0 0 0];  D = 0.1;  St = 0.40;  mu0 = 1.0;
T1   = 2*pi*sqrt(el(1)^3/mu0)/St;              % lap at D = 1
Tnow = 2*pi*sqrt((D*el(1))^3/mu0)/St;          % lap if mu were NOT scaled
Tfix = 2*pi*sqrt((D*el(1))^3/(mu0*D^3))/St;    % lap as the block now flies it
% mu is passed unscaled: EndEffectorTrajectory applies mu*dist_scale^3 itself.
pa = EndEffectorTrajectory(3.0, 1, el, mu0, St, D, z_work, [0 0 0 0]);
pb = EndEffectorTrajectory(3.0 + Tfix, 1, el, mu0, St, D, z_work, [0 0 0 0]);
pc = EndEffectorTrajectory(3.0, 1, el, mu0, St, 1, z_work, [0 0 0 0]);
fprintf('   lap: D = 1 %.2f s | D = 0.1, mu fixed %.2f s | D = 0.1, mu*D^3 %.2f s\n', T1, Tnow, Tfix);
fprintf('   |p(t+T) - p(t)| = %.1e m;  |p_D(t) - D p_1(t)| = %.1e m\n', ...
        norm(pb(1:2) - pa(1:2)), norm(pa(1:2) - D*pc(1:2)));
check(abs(Tfix - T1) < 1e-12 && norm(pa(1:2) - D*pc(1:2)) < 1e-9, 'K: scaling');

%% L. Seed screening (Step 3)
fprintf('\nL. Screened starts, s_min = 0.3\n');
[q3s, qd3s, s3, g3] = scenario_seeds_screened(1, R3.elements, R3.mu, R3.time_scale, R3.dist_scale, R3.z_work, R3.arm_home, 0.3);
[qVs, qdVs, sV, gV] = scenario_seeds_screened(2, elements, mu, SV, DV, z_work, arm_home, 0.3);
fprintf('   run 3 : guess %d, q5 = %6.2f deg, s = %.3f\n', g3, rad2deg(q3s(8)), s3);
fprintf('   V-bar : guess %d, q5 = %6.2f deg, s = %.3f\n', gV, rad2deg(qVs(8)), sV);

%% M. Closed loop, before and after
fprintf('\nM. Closed loop (EnableMotive = 0 equivalent)\n');
fprintf('   %-12s %-22s %8s %10s %10s %10s\n', 'case', 'variant', 'steps', 'peak rpm', 'max resid', 'EE rmse m');
[pV, ~, SV2, tfV] = scenario_defaults(2);
C3 = struct('sc',1,'el',R3.elements,'mu',R3.mu,'S',R3.time_scale,'D',R3.dist_scale,'par',[0 0 0 0],'tf',R3.T_scaled);
CV = struct('sc',2,'el',elements,'mu',mu,'S',SV2,'D',DV,'par',pV,'tf',tfV);
C1 = struct('sc',1,'el',R1.elements,'mu',R1.mu,'S',R1.time_scale,'D',R1.dist_scale,'par',[0 0 0 0],'tf',R1.T_scaled);
C1f = C1;  C1f.mu = R1.mu*R1.dist_scale^3;
qdV0 = pinv(ik9_jac(qV)) * EV0(CV, z_work);
rows = {
  'run 3',  'as recorded',            C3,  R3.q0_9dof, R3.qd0_9dof, 0
  'run 3',  'damping fix only',       C3,  R3.q0_9dof, R3.qd0_9dof, 1
  'run 3',  'screened start',         C3,  q3s,        qd3s,        0
  'run 3',  'screened + damping',     C3,  q3s,        qd3s,        1
  'V-bar',  'default start',          CV,  qV,         qdV0,        0
  'V-bar',  'damping fix only',       CV,  qV,         qdV0,        1
  'V-bar',  'screened start',         CV,  qVs,        qdVs,        0
  'run 1',  'as recorded',            C1,  R1.q0_9dof, R1.qd0_9dof, 0
  'run 1',  'mu*D^3 + screened',      C1f, [],         [],          1 };
res = zeros(size(rows,1), 4);
for i = 1:size(rows,1)
    C = rows{i,3};  q0 = rows{i,4};  qd0 = rows{i,5};
    if isempty(q0)
        [q0, qd0] = scenario_seeds_screened(C.sc, C.el, C.mu, C.S, C.D, z_work, arm_home, 0.3);
    end
    Rr = sim_case(C, q0, qd0, rows{i,6}, r, l, alphas, dt_9dof, z_work);
    res(i,:) = [Rr.steps Rr.peak Rr.res Rr.rmse];
    fprintf('   %-12s %-22s %8s %10.3g %10.2g %10.3g\n', rows{i,1}, rows{i,2}, Rr.stepstr, Rr.peak, Rr.res, Rr.rmse);
end
check(res(3,1) > 0 && res(3,2) < 120 && res(4,1) > 0 && res(7,1) > 0 && res(9,1) > 0 && res(9,2) < 120, ...
      'M: a corrected case did not complete within 120 rpm');
fprintf('\nAll checks passed.\n');
end

% -------------------------------------------------------------------------
function V0 = EV0(C, z_work)
[~, ~, V0] = EndEffectorTrajectory(0, C.sc, C.el, C.mu, C.S, C.D, z_work, C.par);
end

function R = sim_case(C, q, qd, fixed, r, l, alphas, dt, z_work)
clear ukd_current ukd_fixed
N = round(C.tf/dt);  peak = 0;  res = 0;  e2 = 0;  k = 0;  ok = true;
for k = 1:N
    t = (k-1)*dt;
    [p, u, V, Ad] = EndEffectorTrajectory(t, C.sc, C.el, C.mu, C.S, C.D, z_work, C.par);
    if fixed
        [qc, qdc, w, ~, ~, rs] = ukd_fixed(q, qd, p, u, V, Ad, dt, double(k==1), r, l, alphas, 0.3, 0.3);
    else
        [qc, qdc, w, ~, ~, rs] = ukd_current(q, qd, p, u, V, Ad, dt, double(k==1), r, l, alphas);
    end
    if ~all(isfinite([qc; w])) || max(abs(w)) > 1e4
        ok = false;  break
    end
    q = qc;  qd = qdc;
    peak = max(peak, max(abs(w)));  res = max(res, rs);
    e2 = e2 + norm(ik9_fk(q) - p)^2;
end
if ok
    R.steps = N;  R.stepstr = sprintf('%d/%d', N, N);
else
    R.steps = -k; R.stepstr = sprintf('div@%d', k);
end
R.peak = peak;  R.res = res;  R.rmse = sqrt(e2/max(k - ~ok, 1));
end

function check(cond, msg)
if ~cond, error('derivation_checks:fail', '%s', msg); end
end
