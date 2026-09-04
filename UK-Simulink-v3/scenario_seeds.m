function [q0, qd0] = scenario_seeds(scenario, elements, mu, S, D, z_work, arm_home)
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
[p0, u0, V0] = EndEffectorTrajectory(0, scenario, elements, mu, S, D, z_work, ...
                                     scenario_par(scenario));
q0  = ik9_warm_start([0; 0; 0.10; arm_home(:)], p0, u0);
qd0 = pinv(ik9_jac(q0)) * V0;
end

function par = scenario_par(scenario)
[par, ~, ~, ~] = scenario_defaults(scenario);
end
