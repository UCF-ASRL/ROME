function pose7 = floor_to_motive(base, z_rb, cal)
%FLOOR_TO_MOTIVE  RAW OptiTrack (Motive) rigid-body pose of a LEVEL rigid body at a given floor-frame base pose.
%   pose7 = FLOOR_TO_MOTIVE(base, z_rb, cal)
%     base   3xN [x; y; psi] in the floor frame: x, y in m; psi in rad (any value; not required in (-pi, pi]).
%     z_rb   scalar or 1xN, floor-frame height of the rigid-body origin (m).
%     cal    struct, the cal contract (same as motive_to_floor):
%              R      3x3 rotation, Motive frame -> floor frame (proper, det = +1)
%              t      3x1 m, floor-frame position of the Motive origin
%              scale  m per Motive unit
%            ORDER: p_floor = scale * R * p_motive + t, hence p_motive = R' * (p_floor - t) / scale.
%     pose7  7xN [x y z qx qy qz qw], position in Motive units, unit quaternion with the scalar LAST
%            (body -> Motive, Hamilton), the layout MotiveData / record_motive deliver. No sign convention
%            is imposed on the quaternion (q and -q are the same rotation).
%   Level rigid body: body z axis = floor z axis and body x axis at heading psi in the floor plane, so the
%   body orientation in the floor frame is Rz(psi) and in the Motive frame R' * Rz(psi). As a quaternion:
%   q_bm = conj(q_R) x q_z(psi), q_z(psi) = [cos(psi/2); 0; 0; sin(psi/2)] ([w x y z]), where q_R is the
%   quaternion of R. q_R is obtained from R with the largest-diagonal branch (Shepperd), which keeps full
%   precision for any R (all four branches run in tests/test_motive_floor.m, cals 1-9).
%   Exact inverse of motive_to_floor: motive_to_floor(floor_to_motive(b, z, cal), cal) = b, with psi
%   returned in [-pi, pi] (tests/test_motive_floor.m check i, 1e-12). Used by tests and by the camera
%   emulator (F7). No toolbox function, no state.
A = cal.R.';                                % rotation floor frame -> Motive frame (3x3)
N = size(base, 2);                          % number of poses (count)
f = [base(1:2, :); z_rb .* ones(1, N)];     % floor positions of the rigid-body origin (m), 3xN
p = (A * (f - cal.t)) / cal.scale;          % Motive positions (Motive units), 3xN
% quaternion [aw; ax; ay; az] of A (Shepperd: branch on the largest of trace and the diagonal)
tr = A(1,1) + A(2,2) + A(3,3);              % trace of A (-)
if tr >= A(1,1) && tr >= A(2,2) && tr >= A(3,3)
    aw = sqrt(1 + tr) / 2;                  % scalar part (-)
    ax = (A(3,2) - A(2,3)) / (4*aw);
    ay = (A(1,3) - A(3,1)) / (4*aw);
    az = (A(2,1) - A(1,2)) / (4*aw);
elseif A(1,1) >= A(2,2) && A(1,1) >= A(3,3)
    ax = sqrt(1 + A(1,1) - A(2,2) - A(3,3)) / 2;
    aw = (A(3,2) - A(2,3)) / (4*ax);
    ay = (A(1,2) + A(2,1)) / (4*ax);
    az = (A(1,3) + A(3,1)) / (4*ax);
elseif A(2,2) >= A(3,3)
    ay = sqrt(1 - A(1,1) + A(2,2) - A(3,3)) / 2;
    aw = (A(1,3) - A(3,1)) / (4*ay);
    ax = (A(1,2) + A(2,1)) / (4*ay);
    az = (A(2,3) + A(3,2)) / (4*ay);
else
    az = sqrt(1 - A(1,1) - A(2,2) + A(3,3)) / 2;
    aw = (A(2,1) - A(1,2)) / (4*az);
    ax = (A(1,3) + A(3,1)) / (4*az);
    ay = (A(2,3) + A(3,2)) / (4*az);
end
cz = cos(base(3, :) / 2);                   % cos(psi/2) (-), 1xN
sz = sin(base(3, :) / 2);                   % sin(psi/2) (-), 1xN
% Hamilton product [aw; ax; ay; az] x [cz; 0; 0; sz]
qw = aw*cz - az*sz;                         % scalar part (-), 1xN
qx = ax*cz + ay*sz;                         % vector part x (-), 1xN
qy = ay*cz - ax*sz;                         % vector part y (-), 1xN
qz = az*cz + aw*sz;                         % vector part z (-), 1xN
pose7 = [p; qx; qy; qz; qw];                % [x y z (Motive units); qx qy qz qw (-)], 7xN
end
