function eul = quat2eul(q, seq)
%QUAT2EUL  Euler angles from a unit quaternion, without a toolbox.
%
%   eul = QUAT2EUL(q) and QUAT2EUL(q,'ZYX') return [yaw pitch roll].
%   eul = QUAT2EUL(q,'XYZ') returns [a1 a2 a3] such that R = Rx(a1) Ry(a2) Rz(a3).
%
%   q is scalar first, [w x y z], one row per quaternion. Hamilton convention.
%
%   WHY THIS EXISTS
%   MotiveData calls quat2eul(q,'XYZ'). The MathWorks quat2eul ships with
%   Robotics System Toolbox and Navigation Toolbox, and neither is installed
%   on every machine this model runs on: this one has only Aerospace Toolbox,
%   which provides quat2angle and quat2dcm but not quat2eul. Rather than make
%   the motion-capture path depend on a toolbox that may be absent, the two
%   sequences actually used are implemented here.
%
%   DO NOT substitute ConvertQuattoEulerXYZ from the dyn-sim utilities. It is
%   named XYZ but computes the NEGATED 321 (ZYX) sequence -- its own header
%   says "This is a 321 rotation". Checked against the rotation matrix over
%   random draws: it matches negated ZYX to 0.0e+00 and differs from XYZ by up
%   to 3.07 rad. See check_quat_conv.m.
%
%   Verified against the definition R = Rx*Ry*Rz (and Rz*Ry*Rx) built from the
%   quaternion's own rotation matrix, to better than 1e-14 rad.
if nargin < 2, seq = 'ZYX'; end
q = reshape(q, [], 4);
n = size(q, 1);
eul = zeros(n, 3);
for k = 1:n
    w = q(k,1); x = q(k,2); y = q(k,3); z = q(k,4);
    s = hypot(hypot(w, x), hypot(y, z));
    if s > 0, w = w/s; x = x/s; y = y/s; z = z/s; end
    R = [1-2*(y^2+z^2), 2*(x*y - z*w), 2*(x*z + y*w);
         2*(x*y + z*w), 1-2*(x^2+z^2), 2*(y*z - x*w);
         2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x^2+y^2)];
    switch upper(seq)
        case 'XYZ'      % R = Rx(a1) * Ry(a2) * Rz(a3)
            a2 = asin(max(-1, min(1, R(1,3))));
            if abs(R(1,3)) < 1 - 1e-12
                a1 = atan2(-R(2,3), R(3,3));
                a3 = atan2(-R(1,2), R(1,1));
            else                                % gimbal lock: fold into a1
                a1 = atan2(R(3,2), R(2,2));
                a3 = 0;
            end
            eul(k,:) = [a1 a2 a3];
        case 'ZYX'      % R = Rz(yaw) * Ry(pitch) * Rx(roll)
            p = asin(max(-1, min(1, -R(3,1))));
            if abs(R(3,1)) < 1 - 1e-12
                r  = atan2(R(3,2), R(3,3));
                yw = atan2(R(2,1), R(1,1));
            else
                r  = atan2(-R(2,3), R(2,2));
                yw = 0;
            end
            eul(k,:) = [yw p r];
        otherwise
            error('quat2eul:seq', 'Only XYZ and ZYX are implemented, got %s', seq);
    end
end
end
