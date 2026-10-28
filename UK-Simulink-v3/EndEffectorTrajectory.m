function [p_des, u_des, V_des, A_des] = EndEffectorTrajectory(t, scenario, ...
                        elements, mu, time_scale, dist_scale, z_work, par)
%ENDEFFECTORTRAJECTORY  SE(3) end-effector reference for the 9-DOF UK solve.
%
%   Replaces OrbitTrajectory in the 9-DOF model. Produces the pose, twist and
%   twist derivative that UKDynamics constrains the end effector to follow.
%
%   SCENARIO
%     1  scaled elliptical orbit   the trajectory the 3-DOF model already flies
%     2  V-bar approach            manuscript Eq. (23), eq:vbar
%     3  R-bar approach            manuscript Eq. (24), eq:rbar
%     4  natural-motion circumnav  manuscript Eq. (25), eq:nmc
%     5  circular path             the e = 0 special case of scenario 1
%
%   FRAME. Scenarios 2 to 4 are written in the chief-centred LVLH frame of
%   manuscript Eq. (21), eq:lvlh: x radial outward (the R-bar axis), y in-track (the
%   V-bar axis), z orbit normal. The chief sits at the tabletop origin and the
%   LVLH (x, y) plane maps onto the floor. The cross-track coordinate is
%   identically zero in all three, so the end effector holds z_work.
%
%   SCALING. The RPO references are written at operational scale -- tens to
%   hundreds of metres over one chief orbit. dist_scale and time_scale carry
%   them onto the table using the same similarity the orbit scenario uses,
%
%       p(t) = D p_ref(S t),    v = D S v_ref,    a = D S^2 a_ref
%
%   with D = dist_scale and S = time_scale. The shape is preserved exactly;
%   only the size and the clock change.
%
%   INPUTS
%     t           1x1  simulation time (s)
%     scenario    1x1  selector, 1 to 5 as above
%     elements    1x6  [a, e, i, Omega, omega, nu0] for scenario 1; a in m
%                      (tabletop units), e dimensionless, angles in rad
%     mu          1x1  scaled gravitational parameter for scenario 1 (m^3/s^2)
%     time_scale  1x1  time scale factor S (dimensionless)
%     dist_scale  1x1  distance scale factor D (dimensionless)
%     z_work      1x1  end-effector working height above the table (m)
%     par         1x4  scenario parameters, meaning set by `scenario`:
%                        2  V-bar : [y0   v0   0    0]   m, m/s
%                        3  R-bar : [x0   v0   0    0]   m, m/s
%                        4  NMC   : [rho  n    phi  0]   m, rad/s, rad
%                        5  circle: [R    w    0    0]   m, rad/s
%                      unused for scenario 1
%
%   OUTPUTS
%     p_des       3x1  desired end-effector position, world frame (m)
%     u_des       4x1  desired attitude, unit quaternion, SCALAR FIRST
%     V_des       6x1  desired twist [v; omega], world (m/s, rad/s)
%     A_des       6x1  desired twist derivative (m/s^2, rad/s^2)
%
%   ATTITUDE. A pure yaw about world +z holding the end effector pointed at
%   the origin, where the chief sits. That is the line-of-sight requirement
%   the RPO scenarios impose on a sensor or a docking port, and it is the
%   criterion the 3-DOF heading task used. The yaw rate is the analytic
%   derivative of atan2, not a numerical difference.
%
%   REFERENCES
%     Clohessy, W. H., and Wiltshire, R. S., J. Aerosp. Sci. 27(9), 1960.
%     Fehse, W., Automated Rendezvous and Docking of Spacecraft, CUP 2003.
%     Brayman, K., Honors Undergraduate Thesis, UCF, 2026.

%% ------------------------------------------------- planar LVLH reference
% pos, vel, acc are 2x1 [radial; in-track] at the scenario's own scale.
pos = zeros(2,1);   vel = zeros(2,1);   acc = zeros(2,1);

% HOLD AT THE START. The reference does not begin until t_hold has passed,
% so the arm has time to drive itself to the start pose while the base sits
% still. Before that the reference is frozen at its t = 0 value, position AND
% rates, so the end effector is asked to sit still at a fixed point rather
% than chase a moving target. Without it the run has to begin with the arm already placed
% by hand, which is not a thing anyone can do.
T_HOLD = 10.0;              % seconds of hold before the path starts
ts = time_scale * max(0.0, t - T_HOLD);   % scenario time after scaling (s)
moving = double(t >= T_HOLD);  % 0 during the hold, 1 after. The RATES below
                               %   are multiplied by it: a frozen position
                               %   with the t = 0 velocity still commanded
                               %   made the end effector settle KD/KP*|V|
                               %   off the reference during the hold, 36 mm
                               %   on the ellipse (22 Sep 2026).

switch scenario

    case 1
        % Scaled elliptical orbit. The same Kepler propagation the 3-DOF
        % model runs, so the commanded path is identical to what the base
        % already flies. Its scaling is applied inside the propagation.
        [pos, vel, acc] = orbit_ref(ts, elements, mu, time_scale, dist_scale);

    case 2
        % V-bar approach, Eq. (23). In-track closing at constant speed with
        % the radial coordinate held identically zero. The radial hold is
        % what makes it non-trivial: the constraint supplies u_x = 2 n v0 to
        % oppose the Coriolis term that would otherwise drift the deputy.
        y0 = par(1);    v0 = par(2);
        pos = [0;  y0 - v0*ts];
        vel = [0;      -v0    ];
        acc = [0;        0    ];

    case 3
        % R-bar approach, Eq. (24). Radial descent with the in-track
        % coordinate held identically zero. More demanding than V-bar: the
        % natural radial acceleration 3 n^2 x points away from the chief and
        % opposes the descent, and a constant u_y = -2 n v0 holds y at zero.
        x0 = par(1);    v0 = par(2);
        pos = [x0 - v0*ts;  0];
        vel = [   -v0;      0];
        acc = [     0;      0];

    case 4
        % Natural-motion circumnavigation, Eq. (25): the unforced 2:1 ellipse
        % solution of the CW equations. The in-track semi-axis is exactly
        % twice the radial one. That ratio is not a choice; it is the
        % defining signature of unforced relative motion.
        rho = par(1);   n = par(2);   phi = par(3);
        c = cos(n*ts + phi);
        s = sin(n*ts + phi);
        pos = [ rho*c;        -2*rho*s    ];
        vel = [-rho*n*s;      -2*rho*n*c  ];
        acc = [-rho*n^2*c;     2*rho*n^2*s];

    case 5
        % Circular path. The simplest closed reference: constant speed,
        % constant radius, constant yaw rate under the pointing law. It is
        % the e = 0 special case of scenario 1, which verify_scenarios
        % confirms by running the orbit with zero eccentricity.
        R = par(1);     w = par(2);
        c = cos(w*ts);  s_ = sin(w*ts);
        pos = [ R*c;       R*s_    ];
        vel = [-R*w*s_;    R*w*c   ];
        acc = [-R*w^2*c;  -R*w^2*s_];

    otherwise
        % An unrecognised selector holds station at the origin rather than
        % commanding an undefined path.
        pos = zeros(2,1);
end

%% ------------------------------------------------------ similarity scale
% Scenario 1 applies both factors inside its own propagation, so it passes
% through untouched. Scenarios 2 to 5 are written at their own scale and are
% carried onto the table here.
if scenario ~= 1
    D = dist_scale;   S = time_scale;
    pos = D * pos;
    vel = D * S * vel;
    acc = D * S^2 * acc;
end

%% ------------------------------------------------------------- position
p_des = [pos(1); pos(2); z_work];   % 3x1 world position (m). The path is
                                    %   planar; the end effector holds a
                                    %   constant working height.

%% ------------------------------------------------------------- attitude
% Point at the chief, which sits at the origin.
psi = atan2(-p_des(2), -p_des(1));      % desired yaw (rad)
% Tool z points DOWN at the floor: R = Rz(psi) * Rx(pi). With the tool up
% every start the AR3 can physically take is badly conditioned (s <= 0.38,
% all five laps diverge, 22 Sep 2026); pointing it down is the same arm
% chain turned 180 deg about the shoulder axis, which keeps the tested
% conditioning (s = 0.68) and puts the elbow above the shoulder where the
% real arm can be. As a quaternion, scalar first:
%   [c; 0; 0; s] (x) [0; 1; 0; 0] = [0; c; s; 0],   c = cos(psi/2), s = sin(psi/2)
u_des = [0; cos(psi/2); sin(psi/2); 0]; % 4x1 quaternion, scalar first

% Analytic d/dt of atan2(y,x) = (x ydot - y xdot)/(x^2 + y^2). The pi offset
% between the outward bearing and the inward one differentiates away, so the
% same expression serves the pointing law above.
den = p_des(1)^2 + p_des(2)^2;          % squared planar radius (m^2)
den = max(den, 1e-9);                   % V-bar and R-bar close on the chief,
                                        %   where the bearing is undefined;
                                        %   the guard keeps this bounded
psi_dot = (p_des(1)*vel(2) - p_des(2)*vel(1)) / den;    % rad/s

%% ------------------------------------------------------- twist and rate
V_des = moving * [vel(1); vel(2); 0; 0; 0; psi_dot];
                        % 6x1 [v; omega], world. No commanded z velocity and
                        %   no roll or pitch rate: a planar path at fixed
                        %   height with a pure yaw pointing law.

A_des = moving * [acc(1); acc(2); 0; 0; 0; 0];
                        % 6x1 [a; alpha], world. Yaw acceleration is left at
                        %   zero; the PD attractor absorbs it. A scenario
                        %   commanding aggressive yaw would want the analytic
                        %   second derivative here rather than a difference.
end


%% ========================================================================
function [pos, vel, acc] = orbit_ref(ts, elements, mu, time_scale, dist_scale)
%ORBIT_REF  The scaled elliptical orbit the 3-DOF model flies.
%   Kepler propagation reproduced from OrbitTrajectory so the commanded path
%   is identical. verify_eetraj checks the two agree to zero.
a_raw = elements(1);   e = elements(2);   inc = elements(3);
Omega = elements(4);   omega = elements(5);   nu0 = elements(6);

a = a_raw * dist_scale;         % scaled semi-major axis (m)

% The gravitational parameter is scaled with the cube of dist_scale so that
% dist_scale is a pure spatial similarity: by Kepler's third law
% T = 2*pi*sqrt(a^3/mu), scaling a by D and mu by D^3 leaves the period
% unchanged, and the path is the D = 1 path scaled by D at the same clock.
% Without this, D = 0.1 shortened the lap from 26.0 s to 0.82 s and the
% reference asked for 1.85 m/s and 26 rad/s of yaw, which is what hardware
% run 1 of 16 September 2026 commanded.
mu_s = mu * dist_scale^3;       % similarity-scaled gravitational parameter

p_semi = a * (1 - e^2);         % semi-latus rectum (m)
n = sqrt(mu_s / a^3);           % mean motion (rad/s)

E0 = 2 * atan(sqrt((1-e)/(1+e)) * tan(nu0/2));   % initial eccentric anomaly
M0 = E0 - e*sin(E0);                             % initial mean anomaly (rad)
M  = M0 + n * ts;                                % mean anomaly now (rad)

E = M;                                           % eccentric anomaly (rad)
for j = 1:5                                      % Newton-Raphson, 5 passes
    E = E - (E - e*sin(E) - M) / (1 - e*cos(E));
end

nu = 2 * atan2(sqrt(1+e)*sin(E/2), sqrt(1-e)*cos(E/2));   % true anomaly (rad)
r_mag = p_semi / (1 + e*cos(nu));                         % radius (m)

r_pqw = [r_mag*cos(nu); r_mag*sin(nu); 0];                % 3x1 position (m)
v_pqw = sqrt(mu_s/p_semi) * [-sin(nu); e + cos(nu); 0] * time_scale;
a_pqw = (-mu_s / r_mag^3) * r_pqw * (time_scale^2);

cO = cos(Omega); sO = sin(Omega);
co = cos(omega); so = sin(omega);
ci = cos(inc);   si = sin(inc);
R = [cO*co-sO*so*ci, -cO*so-sO*co*ci,  sO*si;
     sO*co+cO*so*ci, -sO*so+cO*co*ci, -cO*si;
     so*si,           co*si,           ci];      % 3x3 perifocal -> ECI

r_eci = R * r_pqw;
v_eci = R * v_pqw;
a_eci = R * a_pqw;

pos = r_eci(1:2);
vel = v_eci(1:2);
acc = a_eci(1:2);
end
