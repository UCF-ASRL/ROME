function [T06, P] = ar3_fk(q_arcs_deg)
%AR3_FK  Forward kinematics of the physical AR3 arm from the vendor DH table (ARCS 1.0).
%   [T06, P] = ar3_fk(q_arcs_deg)
%     q_arcs_deg  6x1 ARCS joint angles (deg), the numbers the ARCS GUI shows, ranges J1 -170..170,
%                 J2 -129.6..0, J3 1..143.7, J4 -164.5..164.5, J5 -104.15..104.15, J6 -148.1..148.1
%                 (not enforced here)
%     T06         4x4 pose of the flange frame 6 in the AR3 base frame 0 (rotation, position in m)
%     P           3x7 origins of frames 0..6 in frame 0 (m); P(:,1) = 0
%   Analysis only. The controller still uses the model's own DH (ik9_fk); switching geometry is Plan 2.
%   Degrees here because ARCS angles are a firmware-boundary quantity; everything after is SI.
%
%   Convention: standard DH, as in ARCS.py CalcFwdKin (L3499-3530, "J1 FRAME" block L3562-3580):
%       frame i = Rz(q_i + theta_off_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
%   chained J1*...*J6. The ARCS work and tool frames (ARbot.cal tuple 22-33) are all zero and are
%   left out. Values: ARbot.cal tuple 53-76 (in ARCS-source-files.zip, read with pickletools, never
%   unpickled) and "default calibration values.jpg" in the same zip; both agree. Extraction:
%   docs/verification/AR3_GEOMETRY_SOURCES.md.
%   Known posture (docs/verification/AR3_GEOMETRY_SOURCES.md section 6): q = [0 -90 0 0 0 0] is the
%   upper arm and forearm vertical; q2 = 0 is the upper arm horizontal along +x0.
alpha_deg     = [-90   0   90  -90   90    0];          % 1x6 link twist alpha_i (deg), ARbot.cal tuple 53-58 (DHr1..6)
a_m           = [64.2 305.0 0   0     0     0] * 1e-3;  % 1x6 link length a_i (m), tuple 59-64 (DHa1..6, mm in the file)
d_m           = [169.77 0  0  -222.63 0  -36.25] * 1e-3; % 1x6 link offset d_i (m), tuple 65-70 (DHd1..6, mm in the file)
theta_off_deg = [0     0  -90    0     0  180];         % 1x6 joint angle offset theta_off_i (deg), tuple 71-76 (DHt1..6)

q_deg = q_arcs_deg(:);                      % 6x1 ARCS joint angles (deg)
if numel(q_deg) ~= 6 || ~isnumeric(q_deg) || any(~isfinite(q_deg))
    error('ar3_fk:input', 'q_arcs_deg must be 6 finite numbers (deg), got %d elements', numel(q_deg));
end
T06 = eye(4);                               % running transform 0 -> i (rotation, position in m)
P   = zeros(3, 7);                          % frame origins 0..6 (m)
for i = 1:6
    th = deg2rad(q_deg(i) + theta_off_deg(i));   % DH joint angle theta_i (rad)
    al = deg2rad(alpha_deg(i));                  % DH twist alpha_i (rad)
    A  = [cos(th) -sin(th)*cos(al)  sin(th)*sin(al) a_m(i)*cos(th);
          sin(th)  cos(th)*cos(al) -cos(th)*sin(al) a_m(i)*sin(th);
          0        sin(al)          cos(al)         d_m(i);
          0        0                0               1];       % 4x4 transform i-1 -> i (m)
    T06 = T06 * A;
    P(:, i+1) = T06(1:3, 4);
end
end
