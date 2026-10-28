function [q0, qd0] = scenario_seeds(scenario, elements, mu, S, D, z_work, arm_home, dt)
%SCENARIO_SEEDS  Initial configuration and rates for the selected case study.
%
%   Both seeds depend on the trajectory, so they are solved for whichever
%   scenario is selected rather than carried as literals that would silently
%   go stale when the selector changes.
%
%     q0   9x1  configuration whose end effector sits on p_des(0), from the
%               same damped least squares the solve's own test uses. Starting
%               away from the reference means an initial pose error of order
%               the arm's reach, which drives the solve into a singularity.
%     qd0  9x1  rates giving the commanded twist at t = 0, pinv(Jc)*V_des(0).
%               Starting from rest against a reference already in motion
%               produces a wheel-speed burst in the first second.
par = scenario_par(scenario);       % 1x4 parameters for this case study
[p0, u0, V0] = EndEffectorTrajectory(0, scenario, elements, mu, S, D, z_work, par);
% Damped least squares converges to whichever branch is nearest its starting
% guess (Wampler 1986; Nakamura and Hanafusa 1986), and the branch decides
% whether the run holds together: on V-bar one guess gives 6.6 m of error
% where another gives 6.2e-04 m, with the same trajectory and the same gains.
%
% No static property of the seed predicts which is which. Manipulability does
% not: a branch at 0.367 diverges while one at 0.279 is stable, and choosing
% the best conditioned branch diverges on four scenarios out of five. So each
% candidate is tried on a short segment of the actual trajectory and the
% first one that stays bounded is kept. The first candidate is the guess the
% ellipse was verified with, so that case keeps the configuration it passed
% on.
% Each candidate is screened on the conditioning of the matrix the solve
% inverts before it is tried on the trajectory. The UK correction satisfies
%
%     ||qddot - a||  <=  ||M^(-1/2)|| ||b - A a|| / s,     s = sigma_min(A M^(-1/2)),
%
% so a candidate with small s asks for accelerations that grow as 1/s. The
% wrist singularity q5 = 0 is such a configuration: it gave s = 0.05 and a
% diverging run while the manipulability index read 0.367, which is why that
% index is not used here. Feasible runs hold s >= 0.40 throughout
% (sweep_formulation), and the accepted starts sit at 0.67 to 0.68.
s_min = 0.30;                       % screening threshold on s. Bracketed by
                                    %   measurement: 0.16 at the worst
                                    %   diverging start, 0.67 at the stable
                                    %   ones (metric_check, sweep_formulation)
psi0  = atan2(2*(u0(2)*u0(3) + u0(1)*u0(4)), 1 - 2*(u0(3)^2 + u0(4)^2));
                                    % yaw of the reference attitude, from
                                    %   R(2,1)/R(1,1) of the quaternion, so
                                    %   it holds for any tool direction
guess = [psi0, 0.10, -pi/2, pi/2, 0.0, pi];
                                    % psi0 first: base facing the chief,
                                    %   arm at arm_home. With the tool
                                    %   down the 0.10 guess lands on a
                                    %   branch with J1 = 52, J6 = -68 deg,
                                    %   55 and 68 deg off arm_home; the
                                    %   posture spring then unwinds through
                                    %   the hold at 89 rpm and 59 mm error.
q0 = [];
for gk = 1:numel(guess)
    qc = ik9_warm_start([0; 0; guess(gk); arm_home(:)], p0, u0);
    if norm(ik9_fk(qc) - p0) > 1e-6
        continue                        % that guess did not reach the pose
    end
    if seed_conditioning(qc) < s_min
        continue                        % too close to a singularity to start
    end
    if probe_stable(qc, scenario, elements, mu, S, D, z_work, par, dt)
        q0 = qc;   break
    end
end
assert(~isempty(q0), 'scenario_seeds:branch', ...
      ['No starting guess gave a bounded run for scenario %d. The seed is ' ...
       'not the whole story; check the trajectory scaling.'], scenario);

qd0 = pinv(ik9_jac(q0)) * V0;
end

function par = scenario_par(scenario)
[par, ~, ~, ~] = scenario_defaults(scenario);
end


function s = seed_conditioning(q)
%SEED_CONDITIONING  s = sigma_min(Jc M^-1/2) = sqrt(lambda_min(Jc M^-1 Jc')).
%   The smallest singular value of the matrix the UK solve inverts, at this
%   configuration. mass_at returns the same M and Jc the block builds.
[M, Jc] = mass_at(q);
s = sqrt(max(min(eig(Jc*(M\Jc.'))), 0));
end


function ok = probe_stable(q0, scenario, elements, mu, S, D, z_work, par, dt)
%PROBE_STABLE  Does this branch stay bounded over the opening of the run?
%   Forty steps is enough: a bad branch leaves the reference by more than a
%   metre inside ten.
q   = q0;
[~,~,V0] = EndEffectorTrajectory(0, scenario, elements, mu, S, D, z_work, par);
qd  = pinv(ik9_jac(q0)) * V0;
ok  = true;
for k = 1:40
    t = (k-1)*dt;
    [p,u,V,A] = EndEffectorTrajectory(t, scenario, elements, mu, S, D, z_work, par);
    [qc, qdc] = rome_uk_block(q, qd, p, u, V, A, dt, double(k==1));
    q = qc;  qd = qdc;
    e = norm(ik9_fk(q) - p);
    if ~isfinite(e) || e > 1.0
        ok = false;  return
    end
end
end
