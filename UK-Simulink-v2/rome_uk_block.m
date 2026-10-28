function [q_cmd, qd_cmd, wheel_rpm, tau_arm, manip, resid] = ...
         rome_uk_block(q_meas, qd_meas, p_des, u_des, V_des, A_des, dt, reset)
%#codegen
%ROME_UK_BLOCK  Udwadia-Kalaba constraint solve, shaped for a Simulink MATLAB
%Function block. Outputs POSITION commands.
%
%   ============================ STATUS ==================================
%   THIS FILE IS A DRAFT. Read this before using it.
%
%   1. It does NOT match the interface of the repository's Simulink model.
%      ROME_Main.slx is 3-DOF, base only:
%          [A,b]      = Constraints(q, q_dot, q_des, q_dot_des, q_ddot_des, Kp, Kd)
%          q_ddot_cmd = Dynamics(A, b, m, Izz)
%          wheel_rpm  = InverseKinematics(q, q_dot, alphas, r)
%      This function takes 9-DOF state and end effector pose references.
%      Dropping it in requires new ports and an arm serial interface.
%   2. It diverged when tested from a poor initial configuration.
%   3. Its codegen compatibility is UNVERIFIED: MATLAB Coder is not licensed
%      on the machine where this was written, so "codegen ready" is a design
%      intention here, not a demonstrated fact.
%   4. The arm DH table and link inertias are PLACEHOLDERS.
%
%   For a change that actually fits the existing model, see
%   ROME_BASE_UK_DEMO, which needs only the Constraints block edited.
%   ======================================================================
%
%   INPUTS
%     q_meas   9x1  measured configuration [x y theta q1..q6],
%                   x,y in m; theta and q1..q6 in rad
%     qd_meas  9x1  measured rates, m/s and rad/s, used only to seed the
%                   integrator on a reset
%     p_des    3x1  desired end effector position, world frame (m)
%     u_des    4x1  desired attitude, unit quaternion, scalar first
%                   (dimensionless)
%     V_des    6x1  desired twist [v; omega], world frame (m/s, rad/s)
%     A_des    6x1  desired twist derivative (m/s^2, rad/s^2)
%     dt       1x1  sample time (s)
%     reset    1x1  nonzero re-seeds the internal integrator from q_meas
%                   (dimensionless flag)
%
%   OUTPUTS
%     q_cmd     9x1  POSITION command [x y theta q1..q6], m and rad
%     qd_cmd    9x1  rate command, m/s and rad/s
%     wheel_rpm 4x1  base wheel speeds, rpm. Check against MAX_RPM = 120.
%     tau_arm   6x1  arm joint torques, N m. MONITORING ONLY: the ROME arm is
%                    stepper driven and cannot accept a torque command. Use
%                    these for actuator margin and feasibility.
%     manip     1x1  sqrt(det(Jc Jc')), health signal. Falling toward zero
%                    means the arm is approaching a singularity. Units are
%                    MIXED and not physically meaningful on their own: Jc has
%                    three linear rows (m/rad) and three angular ones
%                    (dimensionless), so only the trend matters, not the
%                    absolute value.
%     resid     1x1  ||A qddot - b||. Rises near a singularity because the
%                    damping is trading constraint satisfaction for bounded
%                    force. That is intended.
%
%   WHY THE SOLVE HAS NO MATRIX SQUARE ROOT
%   The textbook fundamental equation carries M^(-1/2):
%       qddot = a + M^(-1/2) (A M^(-1/2))^+ (b - A a)
%   With B = A M^(-1/2) of full row rank, B^+ = B'(BB')^-1, so
%       M^(-1/2) B^+ = M^(-1) A' (A M^(-1) A')^-1
%   and the square root cancels exactly. This block uses that form. It is
%   algebraically identical, needs no eig, sqrtm or pinv, and is cheaper.
%
%   WHY A IS 6x9 AND NOT 9x9
%   A square, invertible A collapses the solve to qddot = b regardless of M
%   and Q: the pseudoinverse becomes an inverse and the mass metric cancels.
%   Six task rows on nine coordinates leaves a 3-D null space for the solve
%   to resolve. Secondary objectives go into Q as forces, never as extra
%   rows of A, because extra rows would square A up again.
%
%   ATTITUDE
%   Unit quaternions. The error quaternion is formed in the WORLD frame,
%   du = u (x) u_des^-1, so its vector part shares a frame with the angular
%   velocity error omega - omega_des that it is added to.
%
%   SELF-CONTAINED BY DESIGN
%   A Simulink MATLAB Function block can only call external functions that
%   are themselves on the codegen path, and this block is meant to drop into
%   a model without dragging a folder of dependencies and path setup behind
%   it. So the quaternion, wheel-map and manipulability helpers at the bottom
%   of this file deliberately duplicate what lives in util/.
%
%   Duplication is a licence to drift, so the drift is TESTED rather than
%   trusted: review_all.m section [2] drives this block through its public
%   interface and compares its manipulability, wheel speeds and constraint
%   residual against the independently verified rome_9dof_model and the
%   util/ helpers. If the two copies ever diverge, that check fails.
%
%   When editing, change BOTH, then run review_all. The util/ versions carry
%   the fuller derivations and are covered by util_selftest.
%
%   REFERENCES
%     Udwadia, F. E., and Kalaba, R. E., "Analytical Dynamics: A New
%       Approach," Cambridge University Press, 1996.
%     Spong, M. W., Hutchinson, S., and Vidyasagar, M., "Robot Modeling and
%       Control," Wiley, 2004.
%     Khatib, O., IEEE J. Robotics and Automation, Vol. 3, No. 1, 1987.
%     Peters, J., et al., IROS 2005, pp. 1824-1831.
%     Wampler, C. W., IEEE Trans. SMC, Vol. 16, No. 1, 1986.
%     Nakamura, Y., and Hanafusa, H., ASME JDSMC, Vol. 108, 1986.
%     Shepperd, S. W., J. Guidance and Control, Vol. 1, No. 3, 1978.
%     Sofwan, A., et al., ICITACEE 2019.
%
%   See also ROME_BASE_UK_DEMO, ROME_9DOF_UK, UK_SOLVE.

%% ------------------------------------------------------------ PARAMETERS
% Base: measured, from the ROME repository simulation/define_constants.m
mb  = 18.0;                     % base mass (kg)
bd  = 0.45;                     % base plate depth, along body x (m)
bw  = 0.45;                     % base plate width, along body y (m)
Ib  = (1/12)*mb*(bd^2 + bw^2);  % base yaw inertia (kg m^2), thin rectangular
                                %   plate about its own vertical axis
r_w = 0.0762;                   % omni-wheel rolling radius (m)
l_w = 0.35;                     % base centre to wheel contact distance (m)
h0  = 0.20;                     % height of arm joint 1 above the base (m)
alph = deg2rad([45; 135; 225; 315]);   % angular location of each wheel on
                                       %   the base, from body +x,
                                       %   counter-clockwise (rad), per
                                       %   thesis Figure 6 and Table 1. The
                                       %   deployed define_constants.m uses
                                       %   the reverse order; see
                                       %   docs/HARDWARE_IMPLEMENTATION.md
                                       %   section E8.

% ---- PLACEHOLDER ARM DATA. Replace from the AR3 URDF in ONE place. ------
% Standard Denavit-Hartenberg table, one row per joint, columns:
%   1  a       link length, distance along the common normal      (m)
%   2  alpha   link twist, angle between consecutive joint axes   (rad)
%   3  d       link offset, distance along the previous z axis    (m)
%   4  theta   constant offset ADDED to the commanded joint angle (rad)
dh = [ 0.000,  pi/2, 0.169, 0.0
       0.305,  0.0 , 0.000, -pi/2
       0.000,  pi/2, 0.000, 0.0
       0.000, -pi/2, 0.222, 0.0
       0.000,  pi/2, 0.000, 0.0
       0.000,  0.0 , 0.036, 0.0 ];
link_m = [0.90; 0.75; 0.60; 0.35; 0.25; 0.15];      % link masses, joints 1..6 (kg)
link_c = [ 0.0 -0.10  0.0                            % link centre of mass,
                                                     %   [x y z] per row,
                                                     %   in that link's own
                                                     %   frame (m)
          -0.15  0.0  0.0
           0.0 -0.04  0.0
           0.0  0.0 -0.09
           0.0  0.03  0.0
           0.0  0.0 -0.02 ];
link_I = [ 0.0030 0.0010 0.0030                      % link inertia about its
                                                     %   own centre of mass,
                                                     %   diagonal, per row
                                                     %   [Ixx Iyy Izz]
                                                     %   (kg m^2)
           0.0008 0.0060 0.0060
           0.0012 0.0012 0.0008
           0.0009 0.0009 0.0004
           0.0003 0.0003 0.0003
           0.0001 0.0001 0.0002 ];
% Mock spacecraft on the end effector. NOT a separate body: its mass and
% inertia are folded into link 6.
mock_m = 0.50;                      % mock spacecraft mass (kg)
mock_c = [0.0; 0.0; 0.06];          % its centre of mass, in link 6 frame (m)
mock_I = [0.0008 0.0008 0.0008];    % its inertia about its own centre of
                                    %   mass, diagonal [Ixx Iyy Izz] (kg m^2)
% -------------------------------------------------------------------------

gvec = [0.0; 0.0; -9.80665];        % gravitational acceleration, world
                                    %   frame, z up so this points down
                                    %   (m/s^2). This is standard gravity
                                    %   g_n, the conventional value fixed by
                                    %   the 3rd CGPM (1901) and listed in
                                    %   ISO 80000-3. It is a defined
                                    %   constant, not a measurement.

% Task-space gains. Chosen critically damped, Kp = wn^2 and Kd = 2*wn, so
% the error decays as fast as possible without overshooting.
wp = 4.0;                           % position loop natural frequency (rad/s)
wr = 6.0;                           % attitude loop natural frequency (rad/s)
                                    %   NOTE the error fed to this gain is
                                    %   the quaternion vector part, which is
                                    %   about half the angle in rad, so the
                                    %   EFFECTIVE attitude bandwidth is
                                    %   wr/sqrt(2) ~ 4.2 rad/s. See e_task.
KP = [wp^2;wp^2;wp^2; wr^2;wr^2;wr^2];  % proportional gain, position rows
                                        %   then attitude rows (1/s^2)
KD = [2*wp;2*wp;2*wp; 2*wr;2*wr;2*wr];  % derivative gain, same ordering (1/s)

% Null-space posture bias, applied to base yaw and the six arm joints. It
% cannot disturb the task: UK enforces the constraint rows exactly whatever
% Q is, so this only selects WHICH null-space motion happens.
kpost   = 3.0;                      % posture stiffness (1/s^2)
posture = [0.10; 0.0; -0.5; 0.9; 0.0; 0.7; 0.0];
                                    % preferred configuration,
                                    %   [base yaw; joints 1..6] (rad)

% Damped least squares schedule. Damping switches on only as the arm nears a
% singularity, trading exact constraint satisfaction for a bounded command.
w0   = 0.10;                        % manipulability at which damping starts
                                    %   (mixed units, see 'manip' above)
lam0 = 0.04;                        % damping factor at zero manipulability
                                    %   (same mixed units)
hfd  = 1e-6;                        % central-difference step for numerical
                                    %   derivatives. When perturbing a single
                                    %   coordinate the step is hfd itself, in
                                    %   m or rad to match that coordinate.
                                    %   For the directional derivatives along
                                    %   qd the step is hfd*qd, so it carries
                                    %   an extra factor of time.

%% ------------------------------------------------------- INTERNAL STATE
% The block integrates its own command state. Rates come from this
% integrator rather than from differencing the measurement, which is far
% less noisy at 20 Hz. Position error still comes from the measurement, so
% the task loop is closed on the real pose.
persistent q_int qd_int started
% q_int    9x1  integrated position command [x y theta q1..q6] (m, rad)
% qd_int   9x1  integrated rate command (m/s, rad/s)
% started  1x1  logical, false until the first call has seeded the state
if isempty(started)
    started = false;                  % first call, nothing seeded yet
    q_int   = zeros(9,1);             % command position state (m, rad)
    qd_int  = zeros(9,1);             % command rate state (m/s, rad/s)
end
if reset ~= 0 || ~started
    q_int  = q_meas;                  % re-seed position from measurement
    qd_int = qd_meas;                 % re-seed rate from measurement
    started = true;
end

q  = q_meas;    % configuration used this step (m, rad). Taken from the
                %   MEASUREMENT, so the task loop closes on the real pose.
qd = qd_int;    % rates used this step (m/s, rad/s). Taken from the
                %   INTEGRATOR, not from differencing the measurement, which
                %   at 20 Hz would be dominated by differentiation noise.

%% ------------------------------------------------------------- DYNAMICS
% pe  3x1    end effector position, world frame (m)
% Re  3x3     end effector orientation, world from body (dimensionless)
% Tl  4x4x6   homogeneous transform of each link frame, world (m in the
%             translation column, dimensionless in the rotation block)
% pc  3x6     centre of mass of each link, world frame (m)
[pe, Re, Tl, pc] = rome_fk(q, dh, link_m, link_c, mock_m, mock_c, h0);

% Jv  3x9x6   linear velocity Jacobian of each link centre of mass.
%             Columns 1-2 map base translation, so they are m/m, i.e.
%             dimensionless. Columns 3-9 map rotations, so they are m/rad.
% Jw  3x9x6   angular velocity Jacobian of each link. Columns 1-2 are zero,
%             columns 3-9 are rad/rad, i.e. dimensionless.
% Jc  6x9     end effector Jacobian, stacked [linear; angular], with the
%             same column units as Jv and Jw above.
[Jv, Jw, Jc]     = rome_jac(q, Tl, pc, pe, h0);

M = rome_mass(Jv, Jw, Tl, link_m, link_I, mock_m, mock_I, mb, Ib);
                % 9x9 mass matrix. Units are mixed by construction:
                %   kg in the translation block, kg m in the coupling,
                %   kg m^2 in the rotation block.
g = rome_grav(q, dh, link_m, link_c, mock_m, mock_c, gvec, h0, hfd);
                % 9x1 generalized gravity force: N in the translation rows,
                %   N m in the rotation rows.

% Coriolis and centrifugal by the Christoffel identity
%     (C qd)_k = [Mdot qd]_k - dT/dq_k,      T = 1/2 qd' M qd
% Mdot is a directional derivative along qd, so it costs two mass
% evaluations; the gradient is of a SCALAR, so it costs nine cheap ones.
% Building C this way makes Mdot - 2C skew-symmetric by construction, which
% is the passivity property used as a correctness check in rome_9dof_verify.
Mp = rome_mass_at(q + hfd*qd, dh, link_m, link_c, link_I, mock_m, mock_c, ...
                  mock_I, mb, Ib, h0);   % 9x9 mass matrix a step ALONG qd
Mm = rome_mass_at(q - hfd*qd, dh, link_m, link_c, link_I, mock_m, mock_c, ...
                  mock_I, mb, Ib, h0);   % 9x9 mass matrix a step AGAINST qd
Mdot_qd = ((Mp - Mm)/(2*hfd))*qd;        % 9x1 (dM/dt) qd (N, N m)

dTdq = zeros(9,1);          % 9x1 gradient of kinetic energy wrt q (N, N m)
for k = 1:9                 % k: coordinate index being perturbed
    dq_k = zeros(9,1);      % 9x1 perturbation, one coordinate at a time
    dq_k(k) = hfd;          %   step size hfd, in m or rad to match q(k)
    Tp = 0.5*qd.'*rome_mass_at(q+dq_k, dh, link_m, link_c, link_I, mock_m, ...
                               mock_c, mock_I, mb, Ib, h0)*qd;  % T ahead (J)
    Tm = 0.5*qd.'*rome_mass_at(q-dq_k, dh, link_m, link_c, link_I, mock_m, ...
                               mock_c, mock_I, mb, Ib, h0)*qd;  % T behind (J)
    dTdq(k) = (Tp - Tm)/(2*hfd);
end
C_qd = Mdot_qd - dTdq;      % 9x1 Coriolis and centrifugal generalized force
                            %   (N in translation rows, N m in rotation rows)

% Unconstrained applied force, plus the null-space posture bias. UK enforces
% the task rows exactly whatever Q is, so the posture term cannot disturb
% the task; it only selects which null-space motion happens.
Q = -C_qd - g;              % 9x1 applied generalized force before the
                            %   constraint (N, N m)
% Posture bias on base yaw and the six joints, as a critically damped spring
% toward 'posture'. The damping 2*sqrt(kpost) is critical for a unit inertia.
Q(3:9) = Q(3:9) - kpost*(q(3:9) - posture) - 2*sqrt(kpost)*qd(3:9);

%% ---------------------------------------------------------- CONSTRAINT
u  = rome_r2q(Re);                  % 4x1 current attitude quaternion,
                                    %   scalar first (dimensionless)
ud = u_des/max(norm(u_des), 1e-12); % 4x1 desired attitude, renormalised in
                                    %   case the caller passed a drifted one
du = rome_qmul(u, [ud(1); -ud(2:4)]);
                                    % 4x1 error quaternion, u (x) u_des^-1.
                                    %   Formed in the WORLD frame so its
                                    %   vector part shares a frame with the
                                    %   angular velocity error it is added to.
if du(1) < 0
    du = -du;   % q and -q are the same rotation; pick the one representing
                % the rotation under 180 deg, so the arm takes the short way
                % round instead of unwinding the long way.
end
e_task = [pe - p_des; du(2:4)];
    % 6x1 task error. Rows 1-3 are a position error in m. Rows 4-6 are the
    % VECTOR PART of the error quaternion, which for a rotation of angle phi
    % about axis n equals sin(phi/2)*n, so it is approximately phi/2 in rad
    % for small errors. That factor of one half is absorbed into KP; it is
    % noted here because it makes the effective attitude stiffness half of
    % what the gain value alone suggests.

Jcd = rome_jdot(q, qd, dh, link_m, link_c, mock_m, mock_c, h0, hfd);
                % 6x9 time derivative of Jc, needed because differentiating
                %   V = Jc*qd gives Vdot = Jc*qddot + Jcd*qd
V   = Jc*qd;    % 6x1 current end effector twist (m/s, rad/s)
b   = A_des - KD.*(V - V_des) - KP.*e_task - Jcd*qd;
                % 6x1 constraint right-hand side: the twist derivative the
                %   PD law asks for (m/s^2, rad/s^2)

%% -------------------------------------------------------------- SOLVE
a  = M\Q;       % 9x1 UNCONSTRAINED acceleration, what the system would do
                %   if the task constraint vanished (m/s^2, rad/s^2)
JM = M\(Jc.');  % 9x6 M^-1 A', the mass-weighted constraint directions
S  = Jc*JM;     % 6x6 A M^-1 A', the constraint-space inverse inertia
manip = sqrt(max(det(Jc*Jc.'), 0.0));
                % 1x1 Yoshikawa manipulability, mixed units, see header
if manip < w0
    lam2 = lam0^2*(1 - manip/w0)^2;  % squared damping, ramped in smoothly
else
    lam2 = 0.0;                      % far from a singularity: no damping,
                                     %   so the constraint is met exactly
end
qdd = a + JM*((S + lam2*eye(6))\(b - Jc*a));
                % 9x1 constrained acceleration (m/s^2, rad/s^2)
resid = norm(Jc*qdd - b);
                % 1x1 constraint residual. Like manip, this norm runs over
                %   rows with different units, three in m/s^2 and three in
                %   rad/s^2, so its absolute value is not a physical
                %   quantity. Use it as a relative health signal only. It is
                %   nonzero only when lam2 > 0, which is the intended trade.

%% ---------------------------------------------------- INTEGRATE, OUTPUT
% Semi-implicit (symplectic) Euler: the rate is updated FIRST, then the
% position is advanced using that new rate. For the same step size this is
% markedly more stable on oscillatory systems than the explicit form.
qd_int = qd_int + qdd*dt;   % advance rate command (m/s, rad/s)
q_int  = q_int  + qd_int*dt;% advance position command (m, rad)
q_cmd  = q_int;             % 9x1 position command out (m, rad)
qd_cmd = qd_int;            % 9x1 rate command out (m/s, rad/s)

Qc      = M*qdd - Q;        % 9x1 generalized constraint force, the force the
                            %   constraint had to apply (N, N m). This is the
                            %   quantity UK gives you in closed form.
tau_arm = Qc(4:9);          % 6x1 arm joint torques (N m), monitoring only

wheel_rpm = rome_wheels(q, qd_int, r_w, l_w, alph)*(60/(2*pi));
                            % 4x1 wheel speeds (rpm). The factor 60/(2*pi)
                            %   converts rad/s to rev/min.
end


%% =======================================================================
function [pe, Re, Tl, pc] = rome_fk(q, dh, link_m, link_c, mock_m, mock_c, h0)
%#codegen
%ROME_FK  Forward kinematics: link frames, end effector pose, link centres of
%mass, all expressed in the world frame.
%
%   METHOD
%   Standard Denavit-Hartenberg. Each joint contributes the homogeneous
%   transform
%
%       A_i = [ ct  -st*ca   st*sa   a*ct
%               st   ct*ca  -ct*sa   a*st
%                0      sa      ca      d
%                0       0       0      1 ]
%
%   with ct = cos(theta_i), sa = sin(alpha_i), and so on. Chaining them from
%   the base frame gives every link frame. DH is used because it is the
%   convention the AR3 URDF will supply, so swapping in real data is a table
%   edit rather than a rewrite.
%
%   The base contributes a planar transform: translation in x and y, rotation
%   about z by theta, and a fixed lift h0 to the first joint.
%
%   The mock spacecraft is folded into link 6 as a mass-weighted shift of
%   that link's centre of mass. It is NOT a separate body: it is rigidly
%   attached, so it adds inertia to the last link and nothing else.
%
%   REFERENCES
%     Denavit, J., and Hartenberg, R. S., "A Kinematic Notation for
%       Lower-Pair Mechanisms Based on Matrices," ASME J. Applied Mechanics,
%       Vol. 22, 1955, pp. 215-221.
%     Spong, Hutchinson & Vidyasagar, "Robot Modeling and Control," Ch. 3.
    x  = q(1);      % base position along world x (m)
    y  = q(2);      % base position along world y (m)
    th = q(3);      % base yaw angle theta (rad)
    Rb = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
                    % 3x3 base orientation, world from body (dimensionless)
    T  = [Rb, [x; y; h0]; 0 0 0 1];
                    % 4x4 running transform, world to the current frame.
                    %   Seeded at the base, then multiplied joint by joint.

    Tl = zeros(4,4,6);          % transform of each link frame, world (m)
    for i = 1:6                 % i: link index, 1 to 6
        a_  = dh(i,1);          % link length a (m)
        al  = dh(i,2);          % link twist alpha (rad)
        d_  = dh(i,3);          % link offset d (m)
        off = dh(i,4);          % constant joint angle offset (rad)
        ti  = q(3+i) + off;     % actual joint angle theta_i (rad)
        ct = cos(ti);           % cos(theta_i)   (dimensionless)
        st = sin(ti);           % sin(theta_i)   (dimensionless)
        ca = cos(al);           % cos(alpha_i)   (dimensionless)
        sa = sin(al);           % sin(alpha_i)   (dimensionless)
        T = T*[ ct, -st*ca,  st*sa, a_*ct;
                st,  ct*ca, -ct*sa, a_*st;
               0.0,     sa,     ca,    d_;
               0.0,    0.0,    0.0,   1.0];
        Tl(:,:,i) = T;
    end
    Re = T(1:3,1:3);    % end effector orientation, world (dimensionless)
    pe = T(1:3,4);      % end effector position, world (m)

    pc = zeros(3,6);    % centre of mass of each link, world frame (m)
    for i = 1:6                 % i: link index, 1 to 6
        ci = link_c(i,:).';     % 3x1 link CoM in its own frame (m)
        if i == 6
            % Link 6 carries the mock spacecraft. Combine the two into one
            % effective body: the joint CoM is the mass-weighted average.
            mt = link_m(6) + mock_m;    % combined mass of link 6 + mock (kg)
            ci = (link_m(6)*link_c(6,:).' + mock_m*mock_c)/mt;
                                        % combined CoM, link 6 frame (m)
        end
        pc(:,i) = Tl(1:3,1:3,i)*ci + Tl(1:3,4,i);   % rotate, then translate
    end
end


function [Jv, Jw, Jc] = rome_jac(q, Tl, pc, pe, h0)
%#codegen
%ROME_JAC  Linear and angular Jacobians of each link centre of mass, and the
%end effector Jacobian.
%
%   PURELY KINEMATIC. It depends only on geometry, never on mass. That is
%   why mass parameters are not arguments here: passing them would invite
%   the duplication bug this function used to have, where a stale copy of
%   the inertia table was declared locally and silently diverged from the
%   real one.
%
%   METHOD
%   For a revolute joint j with axis z_j through point p_j, a point at p
%   distal to it moves at
%
%       v = z_j x (p - p_j),        omega = z_j
%
%   which is the standard revolute Jacobian column. The base contributes
%   three columns: two unit translations in x and y, and a yaw column whose
%   linear part is again omega x r about the vertical through the base.
%
%   Column 3+j is populated only for joints j <= i, because joints beyond
%   link i do not move link i.
%
%   REFERENCE
%     Spong, Hutchinson & Vidyasagar, "Robot Modeling and Control," Sec. 4.8.
    th  = q(3);                 % base yaw angle theta (rad)
    pb  = [q(1); q(2); h0];     % 3x1 base reference point, world (m)
    zb  = [0;0;1];              % 3x1 base yaw axis, world (dimensionless).
                                %   The base rotates only about vertical.
    Rb0 = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
                                % 3x3 base orientation (dimensionless)

    % Joint axes and origins, computed once. Joint j rotates about the z
    % axis of the frame preceding it.
    zj = zeros(3,6);            % 3x6 rotation axis of each joint, world
                                %   (unit vectors, dimensionless)
    pj = zeros(3,6);            % 3x6 a point on each joint axis, world (m)
    zj(:,1) = Rb0*[0;0;1];      % joint 1 turns about the base vertical
    pj(:,1) = pb;
    for j = 2:6                 % j: joint index, 2 to 6
        % Joint j rotates about the z axis of the frame BEFORE it, j-1.
        zj(:,j) = Tl(1:3,1:3,j-1)*[0;0;1];
        pj(:,j) = Tl(1:3,4,j-1);
    end

    Jv = zeros(3,9,6);          % linear Jacobian per link. Columns 1-2 are
                                %   dimensionless, columns 3-9 are m/rad.
    Jw = zeros(3,9,6);          % angular Jacobian per link, dimensionless
    for i = 1:6                 % i: link whose centre of mass is tracked
        Jvi = zeros(3,9);       % 3x9 linear Jacobian of link i
        Jwi = zeros(3,9);       % 3x9 angular Jacobian of link i
        Jvi(1,1) = 1.0;                          % base x
        Jvi(2,2) = 1.0;                          % base y
        Jvi(:,3) = cross(zb, pc(:,i) - pb);      % base yaw
        Jwi(:,3) = zb;
        for j = 1:i
            Jvi(:,3+j) = cross(zj(:,j), pc(:,i) - pj(:,j));
            Jwi(:,3+j) = zj(:,j);
        end
        Jv(:,:,i) = Jvi; Jw(:,:,i) = Jwi;
    end

    Jc = zeros(6,9);            % 6x9 end effector Jacobian,
                                %   rows 1-3 linear, rows 4-6 angular
    Jc(1,1) = 1.0; Jc(2,2) = 1.0;   % base translation passes straight through
    Jc(1:3,3) = cross(zb, pe - pb);
    Jc(4:6,3) = zb;
    for j = 1:6
        Jc(1:3,3+j) = cross(zj(:,j), pe - pj(:,j));
        Jc(4:6,3+j) = zj(:,j);
    end
end


function M = rome_mass(Jv, Jw, Tl, link_m, link_I, mock_m, mock_I, mb, Ib)
%#codegen
%ROME_MASS  Mass matrix by the link-Jacobian sum.
%
%   METHOD
%       M(q) = sum_i [ m_i Jv_i' Jv_i + Jw_i' R_i I_i R_i' Jw_i ]
%
%   This is exact, not an approximation. It follows from writing the kinetic
%   energy as a sum over links, T = sum_i (1/2 m_i v_i'v_i + 1/2 w_i' I_i w_i),
%   substituting v_i = Jv_i qdot and w_i = Jw_i qdot, and reading off the
%   quadratic form T = 1/2 qdot' M qdot.
%
%   R_i I_i R_i' rotates each link's inertia, quoted about its own centre of
%   mass in its own frame, into the world frame. That rotation is why M
%   depends on configuration at all.
%
%   The base adds mb on the two translations and Ib on yaw. It is treated as
%   a thin plate, Ib = (1/12) mb (d^2 + w^2).
%
%   The result is symmetrised on the way out to remove accumulated
%   round-off; the underlying quantity is symmetric by construction.
%
%   NOTE ON COUPLING. The base-arm off-diagonal block of M is NOT small. On
%   this platform it runs at a median 24 percent of the diagonal blocks. The
%   manuscript's block-diagonal approximation is therefore not justified,
%   and this function does not make it.
%
%   REFERENCE
%     Spong, Hutchinson & Vidyasagar, "Robot Modeling and Control,"
%       Eq. (6.62).
    M = zeros(9,9);             % 9x9 mass matrix, mixed units (see header)
    M(1,1) = mb;                % base mass on world x   (kg)
    M(2,2) = mb;                % base mass on world y   (kg)
    M(3,3) = Ib;                % base inertia on yaw    (kg m^2)
    for i = 1:6                 % i: link index, 1 to 6
        mi = link_m(i);         % effective mass of link i (kg)
        Ii = diag(link_I(i,:)); % 3x3 inertia of link i about its own CoM,
                                %   in its own frame (kg m^2)
        if i == 6
            % Mock spacecraft rigidly attached: masses add, inertias add.
            % The parallel-axis term for the offset between the two centres
            % of mass is neglected here because that offset is small; add it
            % once real geometry is available.
            mi = link_m(6) + mock_m;    % combined mass (kg)
            Ii = diag(link_I(6,:) + mock_I);    % combined inertia (kg m^2)
        end
        Ri  = Tl(1:3,1:3,i);    % 3x3 orientation of link i, world
        Jvi = Jv(:,:,i);        % 3x9 linear Jacobian of link i
        Jwi = Jw(:,:,i);        % 3x9 angular Jacobian of link i
        % Ri*Ii*Ri' rotates the inertia into the world frame. That rotation
        % is the entire reason M depends on configuration.
        M = M + mi*(Jvi.'*Jvi) + Jwi.'*(Ri*Ii*Ri.')*Jwi;
    end
    M = 0.5*(M + M.');          % symmetrise to remove accumulated round-off
end


function M = rome_mass_at(q, dh, link_m, link_c, link_I, mock_m, mock_c, ...
                          mock_I, mb, Ib, h0)
%#codegen
%ROME_MASS_AT  Mass matrix at an arbitrary configuration.
%   Convenience wrapper: runs the kinematics then the mass sum. Used by the
%   finite-difference Coriolis construction, which needs M at perturbed
%   configurations.
    [pe, ~, Tl, pc] = rome_fk(q, dh, link_m, link_c, mock_m, mock_c, h0);
    [Jv, Jw, ~]     = rome_jac(q, Tl, pc, pe, h0);
    M = rome_mass(Jv, Jw, Tl, link_m, link_I, mock_m, mock_I, mb, Ib);
end


function U = rome_pot(q, dh, link_m, link_c, mock_m, mock_c, gvec, h0)
%#codegen
%ROME_POT  Gravitational potential energy of the whole mechanism.
%
%       U(q) = -sum_i m_i * g' * p_ci(q)
%
%   with p_ci the world-frame position of link i's centre of mass. The minus
%   sign is the usual convention with g pointing down, so U increases as the
%   arm lifts.
%
%   Only the arm contributes: the base moves in a horizontal plane, so its
%   height never changes and it adds a constant that differentiates away.
    th = q(3);      % base yaw angle theta (rad)
    Rb = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
                    % 3x3 base orientation (dimensionless)
    T  = [Rb, [q(1); q(2); h0]; 0 0 0 1];
                    % 4x4 running transform, world to current frame
    U  = 0.0;       % accumulated potential energy (J)
    for i = 1:6                 % i: link index, 1 to 6
        a_  = dh(i,1);          % link length a (m)
        al  = dh(i,2);          % link twist alpha (rad)
        d_  = dh(i,3);          % link offset d (m)
        off = dh(i,4);          % constant joint angle offset (rad)
        ti  = q(3+i) + off;     % actual joint angle theta_i (rad)
        ct = cos(ti); st = sin(ti); ca = cos(al); sa = sin(al);
                                % trig of the DH angles (dimensionless)
        T = T*[ ct, -st*ca,  st*sa, a_*ct;
                st,  ct*ca, -ct*sa, a_*st;
               0.0,     sa,     ca,    d_;
               0.0,    0.0,    0.0,   1.0];
        ci = link_c(i,:).';     % 3x1 link CoM in its own frame (m)
        mi = link_m(i);         % effective mass of link i (kg)
        if i == 6
            mi = link_m(6) + mock_m;                % link 6 + mock (kg)
            ci = (link_m(6)*link_c(6,:).' + mock_m*mock_c)/mi;
                                                    % combined CoM (m)
        end
        pci = T(1:3,1:3)*ci + T(1:3,4);  % 3x1 link CoM in world frame (m)
        U = U - mi*(gvec.'*pci);         % add this link's potential (J).
                                         %   gvec points down, so U grows as
                                         %   the link is lifted.
    end
end


function g = rome_grav(q, dh, link_m, link_c, mock_m, mock_c, gvec, h0, hfd)
%#codegen
%ROME_GRAV  Generalized gravity force, as the gradient of potential energy.
%
%       g(q) = dU/dq
%
%   evaluated by central differences. Deriving it this way rather than
%   writing out -sum Jv_i' m_i g guarantees it is consistent with the same
%   potential the energy check uses, so ROME_9DOF_VERIFY's energy
%   conservation test is a genuine test rather than a tautology.
%
%   Central differences are used, not forward, because the error is O(h^2)
%   instead of O(h) for the same number of evaluations.
    g = zeros(9,1);     % 9x1 generalized gravity force (N, N m)
    for k = 1:9                 % k: coordinate index being perturbed
        dq_k = zeros(9,1);      % 9x1 one-coordinate perturbation
        dq_k(k) = hfd;          %   step in m or rad, matching q(k)
        Up = rome_pot(q+dq_k, dh, link_m, link_c, mock_m, mock_c, gvec, h0);
                                % potential a step ahead (J)
        Um = rome_pot(q-dq_k, dh, link_m, link_c, mock_m, mock_c, gvec, h0);
                                % potential a step behind (J)
        g(k) = (Up - Um)/(2*hfd);
    end
end


function Jd = rome_jdot(q, qd, dh, link_m, link_c, mock_m, mock_c, h0, hfd)
%#codegen
%ROME_JDOT  Time derivative of the end effector Jacobian.
%
%   Jc depends on q only, so along a trajectory
%
%       dJc/dt = (dJc/dq) qdot
%
%   which is a DIRECTIONAL derivative along qdot. That means it costs two
%   Jacobian evaluations, at q +/- h*qdot, rather than the nine a full
%   gradient would need.
%
%   It appears in the constraint because differentiating V = Jc qdot gives
%   Vdot = Jc qddot + Jcdot qdot, so the Jcdot qdot term has to be moved to
%   the right-hand side to leave the constraint linear in qddot.
%
%   The mass parameters are NOT arguments: Jc is purely kinematic. An
%   earlier version of this function declared a private copy of the inertia
%   table, which would silently go stale whenever the parameters at the top
%   of this file were edited. Only geometry is passed now, so that failure
%   mode cannot recur.
    % Step ALONG the velocity direction, not along a coordinate axis: this
    % is a directional derivative, so two evaluations suffice.
    [pe1, ~, Tl1, pc1] = rome_fk(q + hfd*qd, dh, link_m, link_c, mock_m, mock_c, h0);
    [~,   ~,  Jp]      = rome_jac(q + hfd*qd, Tl1, pc1, pe1, h0);
                        % pe1, Tl1, pc1: kinematics a step ahead (m)
                        % Jp   6x9      : end effector Jacobian ahead
    [pe2, ~, Tl2, pc2] = rome_fk(q - hfd*qd, dh, link_m, link_c, mock_m, mock_c, h0);
    [~,   ~,  Jm]      = rome_jac(q - hfd*qd, Tl2, pc2, pe2, h0);
                        % pe2, Tl2, pc2: kinematics a step behind (m)
                        % Jm   6x9      : end effector Jacobian behind
    Jd = (Jp - Jm)/(2*hfd);     % 6x9 dJc/dt (1/s times the units of Jc)
end


function w = rome_wheels(q, qd, r, l, alpha)
%#codegen
%ROME_WHEELS  Omni-wheel angular speeds from world-frame base velocity.
%
%       qdot_B = R(-theta) qdot
%       w_i = ( sin(alpha_i) xdot_B + cos(alpha_i) ydot_B + l thetadot ) / r
%
%   The base is HOLONOMIC. This is an invertible velocity mapping applied
%   after the dynamics are solved, not a constraint: it contributes no rows
%   to A.
%
%   The yaw coefficient is l/r, about 4.6 here. Spinning the body at
%   thetadot moves each contact point tangentially at l*thetadot, so the
%   wheel turns at l*thetadot/r. Both the thesis Eq. (16) as printed and the
%   Simulink InverseKinematics block put r in that slot instead, giving
%   r/r = 1. It went unnoticed because the thesis trajectory commanded
%   theta_des = 0, so the base barely rotated.
%
%   REFERENCE
%     Sofwan, A., Mulyana, H. R., Afrisal, H., and Goni, A., ICITACEE 2019.
    th  = q(3);     % base yaw angle theta (rad)
    Rwb = [ cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1];
                    % 3x3 world to body rotation. Note this is the TRANSPOSE
                    %   of the body to world matrix used elsewhere.
    v_b = Rwb*qd(1:3);
                    % 3x1 base velocity in BODY axes:
                    %   [xdot; ydot] in m/s, thetadot in rad/s
    w = zeros(4,1); % 4x1 wheel angular speeds (rad/s)
    for i = 1:4                 % i: wheel index, 1 to 4
        w(i) = (sin(alpha(i))*v_b(1) + cos(alpha(i))*v_b(2) + l*v_b(3))/r;
    end
end


function u = rome_r2q(R)
%#codegen
%ROME_R2Q  Unit quaternion from a rotation matrix, Shepperd's method.
%
%   Scalar first, [w x y z].
%
%   The naive formula w = sqrt(1 + trace(R))/2 loses precision, and divides
%   by zero, as trace(R) approaches -1, which happens near 180 degrees.
%   Shepperd's method computes all four possible forms and takes whichever
%   has the largest denominator, so it stays well conditioned for every
%   rotation. That matters here because the emulated spacecraft can tumble
%   through large angles.
%
%   Duplicated from util/quat_from_rotm.m so this file stays self-contained
%   for Simulink. Keep both in step.
%
%   REFERENCE
%     Shepperd, S. W., "Quaternion from Rotation Matrix," Journal of
%       Guidance and Control, Vol. 1, No. 3, 1978, pp. 223-224.
    tr = R(1,1) + R(2,2) + R(3,3);
        % trace of R (dimensionless). Equals 1 + 2*cos(angle), so it runs
        % from 3 at zero rotation down to -1 at 180 degrees.
    if tr > 0.0
        S = sqrt(tr + 1.0)*2.0;     % scaling denominator (dimensionless).
                                    %   Each branch below picks whichever
                                    %   form keeps S largest, hence best
                                    %   conditioned.
        u = [0.25*S; (R(3,2)-R(2,3))/S; (R(1,3)-R(3,1))/S; (R(2,1)-R(1,2))/S];
    elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
        S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3))*2.0;
        u = [(R(3,2)-R(2,3))/S; 0.25*S; (R(1,2)+R(2,1))/S; (R(1,3)+R(3,1))/S];
    elseif R(2,2) > R(3,3)
        S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3))*2.0;
        u = [(R(1,3)-R(3,1))/S; (R(1,2)+R(2,1))/S; 0.25*S; (R(2,3)+R(3,2))/S];
    else
        S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2))*2.0;
        u = [(R(2,1)-R(1,2))/S; (R(1,3)+R(3,1))/S; (R(2,3)+R(3,2))/S; 0.25*S];
    end
    u = u/max(norm(u), 1e-12);  % renormalise against round-off. The max()
                                %   guards a division by zero that a valid
                                %   rotation matrix can never produce.
end


function c = rome_qmul(a, b)
%#codegen
%ROME_QMUL  Hamilton product of two scalar-first quaternions.
%
%       c0 = a0 b0 - av . bv
%       cv = a0 bv + b0 av + av x bv          <- Hamilton: PLUS the cross
%
%   This is the HAMILTON convention, not the JPL one. They differ in the
%   sign of the cross term, which reverses the order rotations compose in.
%   Mixing them is a classic and very hard-to-find bug, so the convention is
%   fixed here and used consistently everywhere in this project. Under
%   Hamilton, rome_qmul(a,b) corresponds to R(a)*R(b): apply b first.
%
%   Duplicated from util/quat_mul.m so this file stays self-contained for
%   Simulink. Keep both in step.
%
%   REFERENCE
%     Sola, J., "Quaternion Kinematics for the Error-State Kalman Filter,"
%       arXiv:1711.02508, 2017, Sec. 1, which compares the two conventions
%       side by side.
    c = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
         a(1)*b(2) + b(1)*a(2) + a(3)*b(4) - a(4)*b(3);
         a(1)*b(3) + b(1)*a(3) + a(4)*b(2) - a(2)*b(4);
         a(1)*b(4) + b(1)*a(4) + a(2)*b(3) - a(3)*b(2)];
end
