function verify_eetraj()
%VERIFY_EETRAJ  EndEffectorTrajectory must reproduce the model's own orbit.
d = fileparts(mfilename('fullpath')); cd(d); addpath(d);
evalc('define_constants');
z_work = 0.42;
T = T_scaled; dt = 0.05; N = round(T/dt);
dp = zeros(N,1); dv = zeros(N,1); psid = zeros(N,1); qn = zeros(N,1);
for k = 1:N
    t = (k-1)*dt;
    [q_des, q_dot_des, ~] = OrbitTrajectory_ref(t, elements, mu, time_scale, dist_scale);
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
fprintf('  PASS - same orbit, unit quaternion.\n\n');
end
