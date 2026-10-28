function [q0, qd0, s0_seed, gk_used] = scenario_seeds_screened(scenario, elements, mu, S, D, z_work, arm_home, s_min)
%SCENARIO_SEEDS_SCREENED  Prototype of Step 3. Same candidate list as
%scenario_seeds, but a candidate is accepted only if the solve is well
%conditioned there:
%
%       s = sigma_min(Jc M^-1/2) = sqrt(lambda_min(Jc M^-1 Jc'))  >=  s_min
%
%   No trial run is needed. Not wired into define_constants.
%
%     q0       9x1  accepted configuration (m, rad)
%     qd0      9x1  pinv(Jc)*V_des(0) (m/s, rad/s)
%     s0_seed  1x1  s at q0
%     gk_used  1x1  index of the accepted guess

[par, ~, ~, ~] = scenario_defaults(scenario);
[p0, u0, V0] = EndEffectorTrajectory(0, scenario, elements, mu, S, D, z_work, par);
guess = [0.10, 2*atan2(u0(4), u0(1)), -pi/2, pi/2, 0.0, pi];
q0 = [];
for gk = 1:numel(guess)
    qc = ik9_warm_start([0; 0; guess(gk); arm_home(:)], p0, u0);
    if norm(ik9_fk(qc) - p0) > 1e-6
        continue
    end
    [M, Jc] = mass_at(qc);
    s = sqrt(max(min(eig(Jc*(M\Jc.'))), 0));
    if s >= s_min
        q0 = qc;  s0_seed = s;  gk_used = gk;
        break
    end
end
assert(~isempty(q0), 'scenario_seeds_screened:none', ...
       'No candidate start has s >= %.2f for scenario %d.', s_min, scenario);
qd0 = pinv(ik9_jac(q0)) * V0;
end
