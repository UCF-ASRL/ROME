function verify_eetraj()
%VERIFY_EETRAJ  EndEffectorTrajectory must reproduce the model's own orbit.
%   Since 24 Sep 2026 the end-effector reference runs on the onset clock
%   sigma(t) of EndEffectorTrajectory (raised-cosine rate over T_ON = 10 s,
%   then sigma = t - T_ON/2), so it is compared with OrbitTrajectory at
%   sigma(t), the velocity scaled by sigma'. The clock is restated here on
%   purpose: this file verifies that formula.
%   EndEffectorTrajectory also scales mu by dist_scale^3 so that the
%   similarity leaves the period unchanged (its own comment, 16 Sep 2026);
%   OrbitTrajectory does not, so it is given mu dist_scale^3 here. Before
%   24 Sep 2026 this check compared the two at raw mu and could not pass.
d = fileparts(mfilename('fullpath')); cd(d); addpath(d);
evalc('define_constants');
z_work = 0.42;
T_ON = 10; T = T_scaled + T_ON/2; dt = 0.05; N = round(T/dt);
dp = zeros(N,1); dv = zeros(N,1); psid = zeros(N,1); qn = zeros(N,1);
for k = 1:N
    t = (k-1)*dt;
    if t < T_ON, sg = t/2 - (T_ON/(2*pi))*sin(pi*t/T_ON); sgd = (1 - cos(pi*t/T_ON))/2;
    else, sg = t - T_ON/2; sgd = 1; end
    [q_des, q_dot_des, ~] = OrbitTrajectory_ref(sg, elements, mu*dist_scale^3, time_scale, dist_scale);
    q_dot_des = q_dot_des*sgd;
    [p, u, V, ~] = EndEffectorTrajectory(t, 1, elements, mu, time_scale, dist_scale, z_work, par);
    dp(k) = norm(p(1:2) - q_des(1:2));        % planar position must match
    dv(k) = norm(V(1:2) - q_dot_des(1:2));    % planar velocity must match
    psid(k) = V(6);
    qn(k) = abs(norm(u) - 1);                 % quaternion must stay unit
end
fprintf('\n  orbit position  max |EE - OrbitTrajectory| : %.3e m\n', max(dp));
fprintf('  orbit velocity  max |EE - OrbitTrajectory| : %.3e m/s\n', max(dv));
fprintf('  quaternion norm max |‖u‖-1|                : %.3e\n', max(qn));
fprintf('  z held constant at %.3f m, yaw rate range [%.4f %.4f] rad/s\n', ...
        z_work, min(psid), max(psid));
assert(max(dp) < 1e-12 && max(dv) < 1e-12, 'verify:orbit', ...
    'EndEffectorTrajectory does not reproduce OrbitTrajectory.');
assert(max(qn) < 1e-12, 'verify:quat', 'u_des is not a unit quaternion.');

% The reference twist derivative A_des must be the time derivative of the
% reference twist V_des, rows 1, 2 and 6 (the others are zero by
% construction). Central difference of V_des with h = 1e-3 s, sampled
% through the onset and one full path; added 24 Sep 2026 with the yaw
% acceleration feed-forward.
h = 1e-3; tt = linspace(0.5, T - 0.5, 400); dA = zeros(numel(tt), 1); sA = 0;
for k = 1:numel(tt)
    [~, ~, Vp, ~] = EndEffectorTrajectory(tt(k) + h, 1, elements, mu, time_scale, dist_scale, z_work, par);
    [~, ~, Vm, ~] = EndEffectorTrajectory(tt(k) - h, 1, elements, mu, time_scale, dist_scale, z_work, par);
    [~, ~, ~, A]  = EndEffectorTrajectory(tt(k),     1, elements, mu, time_scale, dist_scale, z_work, par);
    dA(k) = norm(A - (Vp - Vm)/(2*h));     % mixed units, m/s^2 and rad/s^2
    sA = max(sA, norm(A));
end
fprintf('  twist derivative max |A_des - dV_des/dt| : %.3e (scale %.3e, ratio %.1e)\n', max(dA), sA, max(dA)/sA);
assert(max(dA) < 1e-6 * sA, 'verify:Ades', 'A_des is not the derivative of V_des.');
fprintf('  PASS - same orbit, unit quaternion, A_des = dV_des/dt.\n\n');
end
