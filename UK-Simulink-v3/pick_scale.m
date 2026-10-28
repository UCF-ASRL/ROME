function pick_scale()
%PICK_SCALE  What dist_scale fits the camera area?
%
%   dist_scale is a pure spatial similarity: EndEffectorTrajectory scales mu
%   by dist_scale^3, so the lap TIME is set by time_scale alone and does not
%   change when the orbit is made smaller. Shrinking the orbit to fit the room
%   therefore costs nothing in timing, only in how far the end effector
%   travels and how fast the wheels have to turn.
here = fileparts(mfilename('fullpath'));  cd(here);  addpath(here, fullfile(here,'Optitrack'));
evalc('define_constants');

fprintf(['\n  Scenario 1. Lap time is %.1f s at every scale below, because\n' ...
         '  dist_scale is a similarity.\n\n'], T_scaled);
fprintf('  %-12s %12s %14s %11s\n', 'dist_scale', 'EE reach (m)', 'floor needed', 'peak rpm');
for D = [1.00 0.75 0.50 0.40 0.30 0.25]
    [q0, ~] = scenario_seeds(1, elements, mu, time_scale, D, z_work, arm_home, dt_9dof);
    q = q0;  qd = zeros(9,1);
    N = round(T_scaled/dt_9dof);
    reach = 0;  peak = 0;  ok = true;
    clear ukd_current
    for k = 1:N
        t = (k-1)*dt_9dof;
        [p, u, V, A] = EndEffectorTrajectory(t, 1, elements, mu, time_scale, D, ...
                                             z_work, [0 0 0 0]);
        reach = max(reach, norm(p(1:2)));
        [qc, qdc, w] = ukd_current(q, qd, p, u, V, A, dt_9dof, double(k==1), ...
                                   r, l, alphas);
        if ~all(isfinite([qc; qdc; w])) || max(abs(w)) > 1e4, ok = false; break, end
        peak = max(peak, max(abs(w)));
        q = qc;  qd = qdc;
    end
    if ok
        fprintf('  %-12.2f %12.2f %9.1f x %.1f %11.1f\n', D, reach, 2*reach, ...
                2*reach, peak);
    else
        fprintf('  %-12.2f %12.2f %9.1f x %.1f %11s\n', D, reach, 2*reach, ...
                2*reach, 'DIVERGED');
    end
end
fprintf(['\n  Measure the camera area, pick the largest scale that fits with\n' ...
         '  clearance, and set dist_scale in the block at the top of\n' ...
         '  define_constants.m. Re-run gate_all afterwards.\n\n']);
end
