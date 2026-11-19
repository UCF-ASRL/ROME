function [yaw_deg, yawref_deg, eatt_deg] = ee_attitude(Re, u_des)
%EE_ATTITUDE  End-effector yaw, reference yaw and attitude error at one sample.
%   [yaw, yawref, eatt] = ee_attitude(Re, u_des) with Re the end-effector
%   rotation matrix from ik9_fk (3x3, inertial frame) and u_des the reference
%   quaternion from EndEffectorTrajectory (4x1, scalar first, the pure yaw
%   [0 cos(psi/2) sin(psi/2) 0]). Returns, in degrees,
%     yaw_deg     heading of the tool x axis in the floor plane, atan2(Re(2,1), Re(1,1))
%     yawref_deg  the reference yaw psi
%     eatt_deg    the angle of the residual rotation Re * R_ref', acos((tr - 1)/2)
%   The yaw is wrapped to (-180, 180]; unwrap it before plotting a full turn.
w = u_des(1); v = u_des(2:4); v = v(:);
K = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
R_ref = eye(3) + 2*w*K + 2*K*K;                    % rotation matrix of a unit quaternion
yaw_deg    = rad2deg(atan2(Re(2,1), Re(1,1)));
yawref_deg = rad2deg(2*atan2(u_des(3), u_des(2)));
c = (trace(Re*R_ref.') - 1)/2;
eatt_deg   = rad2deg(acos(max(-1, min(1, c))));
end
