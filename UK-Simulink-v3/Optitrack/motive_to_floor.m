function base = motive_to_floor(pose7, cal)
%MOTIVE_TO_FLOOR  Base pose [x; y; psi] in the floor frame from a RAW OptiTrack (Motive) rigid-body pose.
%   base = MOTIVE_TO_FLOOR(pose7, cal)
%     pose7  7xN, one column per frame, [x y z qx qy qz qw] RAW as streamed (MotiveData output 7,
%            natnet_parse_rigidbody, record_motive rec.pose7): position in Motive units; quaternion with the
%            scalar LAST, the rigid body's orientation in the Motive frame (body -> Motive, Hamilton).
%            The quaternion need not be exactly unit: psi does not depend on its norm (see xb below).
%     cal    struct, the cal contract (Plan 2 Task 2; produced by lab/fit_motive_cal.m):
%              R      3x3 rotation, Motive frame -> floor frame (proper, det = +1)
%              t      3x1 m, floor-frame position of the Motive origin
%              scale  m per Motive unit
%            ORDER: p_floor = scale * R * p_motive + t   (rotate and scale the Motive vector, then translate).
%            Other fields of cal are ignored. There is no yaw_axis field: the yaw axis is the floor z axis,
%            by the floor-frame convention below (settled in F5a, 19 Nov 2026).
%     base   3xN [x; y; psi] in the floor frame: x, y in m; psi in rad in [-pi, pi] (atan2; -pi only for a -0 y component), NOT unwrapped
%            (the model block unwraps with yaw_unwrap; lab tools with unwrap()).
%   Floor frame: paper LVLH, x radial, y in-track, z up; psi is measured from floor +x, positive
%   counter-clockwise seen from above (about floor +z).
%   psi = heading of the rigid body's x axis projected on the floor plane:
%     xb = the body x axis in Motive coordinates (first column of R(q)), v = R * xb in the floor frame,
%     psi = atan2(v(2), v(1)). The floor z component of the position (rigid-body height) is discarded.
%   Degenerate case: a body x axis exactly vertical gives v(1) = v(2) = 0 and psi = atan2(0, 0) = 0. The
%   robot's rigid body is level, so this does not occur in use. A NaN in pose7 gives NaN in base.
%   Inverse: floor_to_motive (exact for a level rigid body). Single implementation: lab/lab_rec_to_floor.m
%   (used by the lab gate frame_check and by fit_motive_cal) calls this function; the model block will
%   call it too (F5b). Tests: tests/test_motive_floor.m.
%   No toolbox function, no persistent state, columns processed independently: usable in a MATLAB
%   Function block with a 7x1 input.
p  = pose7(1:3, :);                         % positions in the Motive frame (Motive units), 3xN
qx = pose7(4, :);                           % quaternion vector part x (-), 1xN
qy = pose7(5, :);                           % quaternion vector part y (-), 1xN
qz = pose7(6, :);                           % quaternion vector part z (-), 1xN
qw = pose7(7, :);                           % quaternion scalar part (-), 1xN
pos = cal.scale * (cal.R * p) + cal.t;      % positions in the floor frame (m), 3xN
xb = [qw.^2 + qx.^2 - qy.^2 - qz.^2; ...    % body x axis in the Motive frame times |q|^2 (3xN): first column of the
      2*(qx.*qy + qz.*qw); ...              % homogeneous rotation matrix of q; equals the first column of R(q) for a
      2*(qx.*qz - qy.*qw)];                 % unit q, and its direction (hence psi) does not depend on |q|
v = cal.R * xb;                             % the same axis in the floor frame (3xN)
psi = atan2(v(2, :), v(1, :));              % heading about floor z from floor +x (rad), 1xN, in [-pi, pi]
base = [pos(1:2, :); psi];                  % [x (m); y (m); psi (rad)], 3xN
end
