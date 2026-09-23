function T = sweep_formulation()
%SWEEP_FORMULATION  Does the formulation stay bounded in every scenario?
%
%   Runs each case study to completion outside Simulink, in the same loop the
%   model runs (command fed back as measurement, EnableMotive = 0), over the
%   five scenarios, the three recorded hardware geometries, a range of orbit
%   scales and clocks, two starting-configuration methods, and displaced
%   starts.
%
%   Solve
%     block      ukd_current, regenerated from the UKDynamics chart, so this
%                is the code the model runs. A second copy of the same
%                equation would prove nothing: the independent cross-check of
%                the solve is derivation_checks section A, which compares the
%                pseudoinverse form, the square-root-free form and the KKT
%                solve, three different algebras, at every start.
%
%   Starts
%     seeds      scenario_seeds, which screens on s and then trial-runs
%     screened   scenario_seeds_screened, which screens on s alone
%     displaced  the accepted start moved by 1 cm/1 deg and 3 cm/2 deg, since
%                the robot will not be placed exactly at q0
%
%   Reported per run: steps completed of steps attempted, peak wheel command,
%   worst constraint residual, end-effector RMSE, and the smallest s seen,
%   where s = sigma_min(Jc M^-1/2).

here = fileparts(mfilename('fullpath'));  cd(here);  addpath(here);
evalc('define_constants');
dataDir = 'C:\Users\jbelmonte\OneDrive\Joao M. R. Belmonte\UCF\Research';
SMIN = 0.3;      % screening threshold, bracketed by metric_check

cases = {};
for sc = 1:5
    [par, D, S, tf, nm] = scenario_defaults(sc);
    if sc == 1, D = dist_scale; S = time_scale; tf = T_scaled; end
    cases(end+1,:) = {nm, sc, elements, mu, S, D, par, tf}; %#ok<AGROW>
end
% the three recorded hardware runs, as flown
for f = {'scaled_factor_0.1_scenario1.mat','run1 D=0.1'; ...
         'scaled_factor_1.0_scenario1_semimajor0.5.mat','run2 a=0.5'; ...
         'scaled_factor_1.0_scenario1_semimajor10.mat','run3 a=10'}'
    R = load(fullfile(dataDir, f{1}));
    cases(end+1,:) = {f{2}, 1, R.elements, R.mu, R.time_scale, R.dist_scale, R.par, R.T_scaled}; %#ok<AGROW>
end
% scenario 1 over a range of orbit scales, now that dist_scale is a pure
% similarity (EndEffectorTrajectory scales mu by dist_scale^3)
for D = [0.1 0.25 0.5 2.0]
    cases(end+1,:) = {sprintf('ellipse D=%.2g', D), 1, elements, mu, time_scale, D, ...
                      [0 0 0 0], T_scaled}; %#ok<AGROW>
end
% and over the clock, faster and slower than the nominal 0.40
for Sf = [0.25 0.60 0.80]
    cases(end+1,:) = {sprintf('ellipse S=%.2g', Sf), 1, elements, mu, Sf, dist_scale, ...
                      [0 0 0 0], 2*pi*sqrt(elements(1)^3/mu)/Sf}; %#ok<AGROW>
end

% The solve is the block's own code, regenerated from the model.
variants = {'block', 0};
starts   = {'seeds', 'screened'};
% Placement error: the robot will not start exactly at q0. Each start is also
% run displaced by these amounts (m on the base, rad on yaw and every joint).
perturb  = [0, 0.01, 0.03];

fprintf('\n%-14s %-9s %-9s %10s %9s %9s %10s %8s\n', 'case', 'start', 'solve', ...
        'steps', 'peak rpm', 'max res', 'EE rmse', 'min s');
T = struct('case', {}, 'start', {}, 'solve', {}, 'ok', {}, 'peak', {}, ...
           'res', {}, 'rmse', {}, 'smin', {});
for i = 1:size(cases,1)
    C = cell2struct(cases(i,:), {'name','sc','el','mu','S','D','par','tf'}, 2);
    for st = 1:numel(starts)
        try
            if strcmp(starts{st}, 'seeds')
                [q0, qd0] = scenario_seeds(C.sc, C.el, C.mu, C.S, C.D, z_work, arm_home, dt_9dof);
            else
                [q0, qd0] = scenario_seeds_screened(C.sc, C.el, C.mu, C.S, C.D, z_work, arm_home, SMIN);
            end
        catch ME
            fprintf('%-14s %-9s %-9s   no start: %s\n', C.name, starts{st}, '-', ME.message);
            continue
        end
        for v = 1:size(variants,1)
            for pj = 1:numel(perturb)
                if perturb(pj) == 0
                    qp = q0;  lbl = variants{v,1};
                elseif ~strcmp(variants{v,1}, 'block')
                    continue                     % perturb the block only
                else
                    rng(100*pj + st);            % repeatable displacement
                    qp = q0 + perturb(pj)*sign(randn(9,1));
                    lbl = sprintf('+%.0fcm/%.0fdeg', perturb(pj)*100, rad2deg(perturb(pj)));
                end
                R = run_case(C, qp, qd0, variants{v,1}, 0, r, l, alphas, dt_9dof, z_work);
                fprintf('%-14s %-9s %-11s %10s %9.4g %9.2g %10.3g %8.3f\n', C.name, starts{st}, ...
                        lbl, R.steps, R.peak, R.res, R.rmse, R.smin);
                T(end+1) = struct('case', C.name, 'start', starts{st}, 'solve', lbl, ...
                                  'ok', R.ok, 'peak', R.peak, 'res', R.res, 'rmse', R.rmse, ...
                                  'smin', R.smin); %#ok<AGROW>
            end
        end
    end
end

bad  = ~[T.ok];
over = [T.peak] > max_rpm;          % bounded, but not commandable on this robot
fprintf('\n  %d of %d runs completed; %d diverged\n', sum(~bad), numel(T), sum(bad));
fprintf('  %d of %d exceeded the %g rpm ceiling (bounded is not commandable)\n', ...
        sum(over & ~bad), numel(T), max_rpm);
if any(bad)
    for k = find(bad)
        fprintf('    diverged: %-14s %-9s %s\n', T(k).case, T(k).start, T(k).solve);
    end
end
for k = find(over & ~bad)
    fprintf('    over ceiling: %-14s %-9s %-11s peak %.1f rpm\n', ...
            T(k).case, T(k).start, T(k).solve, T(k).peak);
end
if any(bad)
    error('sweep_formulation:diverged', '%d of %d runs diverged', sum(bad), numel(T));
end
end


function R = run_case(C, q, qd, variant, lam, r, l, alphas, dt, z_work)
clear ukd_current
N = round(C.tf/dt);  peak = 0;  res = 0;  e2 = 0;  smin = inf;  ok = true;  k = 0;
for k = 1:N
    t = (k-1)*dt;
    [p, u, V, Ad] = EndEffectorTrajectory(t, C.sc, C.el, C.mu, C.S, C.D, z_work, C.par);
    [qc, qdc, w, ~, ~, rs] = ukd_current(q, qd, p, u, V, Ad, dt, double(k==1), r, l, alphas);
    s = seed_s(q);
    if ~all(isfinite([qc; w])) || max(abs(w)) > 1e4
        ok = false;  break
    end
    q = qc;  qd = qdc;
    peak = max(peak, max(abs(w)));  res = max(res, rs);  smin = min(smin, s);
    e2 = e2 + norm(ik9_fk(q) - p)^2;
end
R.ok = ok;
if ok, R.steps = sprintf('%d/%d', N, N); else, R.steps = sprintf('div@%d', k); end
R.peak = peak;  R.res = res;  R.rmse = sqrt(e2/max(k - ~ok, 1));  R.smin = smin;
end


function s = seed_s(q)
% s = sigma_min(Jc M^-1/2) at this configuration, the quantity screened.
[M, Jc] = mass_at(q);
s = sqrt(max(min(eig(Jc*(M\Jc.'))), 0));
end
