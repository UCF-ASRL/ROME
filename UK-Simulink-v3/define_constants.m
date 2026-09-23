%% ========================================================================
%  EDIT THIS BLOCK. NOTHING ELSE IN THIS FILE NEEDS TOUCHING.
%  ========================================================================

scenario = 1;          % 1 ellipse | 2 V-bar | 3 R-bar | 4 NMC | 5 circle

EnableHardware = 0;    % 0 simulate only. 1 commands the real robot.
EnableMotive   = 0;    % 0 closes the loop on the model's own state.
                       % 1 closes it on the cameras. THIS IS THE LAB MODE.

% HOW THE RUN IS MEANT TO WORK
% Four cameras watch a rectangular area. The robot is placed anywhere inside
% it, the cameras report where it is, and the only thing commanded is the end
% effector: the solve decides how base and arm share the job. That is the
% whole point of the formulation, so in the lab EnableMotive = 1 and the base
% pose is MEASURED, never typed.
%
% base_override below is the fallback for when the cameras are not running --
% bench testing, or a dry run at a desk. Leave it NaN and the start pose is
% computed: inverse kinematics puts the end effector on the reference at
% t = 0, screened so the solve is well conditioned there. That is what the
% five case studies are verified against.
%
% With EnableMotive = 1 this is ignored: the cameras win.
base_override = [NaN; NaN; NaN];        % [x (m); y (m); yaw (deg)]

% SPEED AND SIZE. These apply to EVERY scenario, on top of whatever that
% scenario was designed around, so one number slows the whole set down.
%
%   speed_factor  1.0 = as designed.  0.5 = half speed, so twice the lap time.
%   size_factor   1.0 = as designed.  0.5 = half the orbit, half the floor.
%
% Size costs nothing in timing: dist_scale is a pure spatial similarity
% (EndEffectorTrajectory scales mu by dist_scale^3), so shrinking the orbit
% leaves the lap time alone and only reduces how far and how fast the base
% has to travel.
%
% The defaults below are deliberately slow for a first run. Turn them up one
% step at a time, re-running gate_all each time.
speed_factor = 0.50;   % of the designed speed
size_factor  = 0.50;   % of the designed size

% ARM JOINT CONVENTION. The model's joint angles and the Teensy firmware's are
% not the same numbers. The firmware accepts J2 in 0..132 deg and J3 in
% 1..141 deg and REJECTS anything outside (ValidateTraj returns 2, the arm does
% not move). The model's start pose has J2 near -53 deg, which the firmware
% would refuse. The offset below is ADDED to the model's angles, in degrees,
% before they are sent. It has to be measured once:
%
%   1. CAL_ARM. The arm goes to the firmware home, [0 90 90 1 0 0] deg.
%   2. Read actualJointDeg from the ARM, telemetry line.
%   3. Note the physical posture and compare with what the model's forward
%      kinematics gives at candidate angles -- see GETTING_STARTED.md.
%
% Leave zeros until that is done. With zeros the arm command is what the model
% computes, which the firmware will reject for J2 and J3.
arm_offset_deg = [0 0 0 0 0 0];     % added to model angles before sending (deg)

%% ========================================================================
%  Below here is the formulation. Leave it alone.
%  ========================================================================

% Define constants used throughout the model
% This file is run once automatically when the project is opened
% Run this file again after making any changes to update the workspace

%% Ground Vehicle
m = 18; % mass kg
r = 0.0762; % wheel radius meters
l = 0.35; % distance from robot C.G to wheels meters
d = 0.45; % length of robot side(s) meters
w = 0.45; % width of robot side(s) meters 
Izz = (1/12)*m*(d^2+w^2); % base moment of interia
alphas = [deg2rad(315), deg2rad(225), deg2rad(135), deg2rad(45)]; % angle to each wheel from positive y-axis

%% Execution Flags

%% 9-DOF integration (Part H)
dt_9dof   = 0.05;   % solver step AND the UKDynamics dt input (s). 20 Hz,
                    % matching DT_PID in MatlabPIDLoop.ino. At this step
                    % test_uk_block gave 2.829e-03 m EE RMSE on its own
                    % test trajectory (docs/HARDWARE_IMPLEMENTATION.md H1).
z_work    = 0.42;   % end-effector working height above the table (m).
                    % test_uk_block started the EE at [0.220 0 0.420].
                    % manip is checked over a full lap before a new value
                    % is adopted.
arm_home  = deg2rad([-3.0; -53.0; -10.0; 0.0; -27.0; 0.0]);
                    %   6x1 calibration home AND null-space posture target
                    %   (rad). Chosen so the end effector sits AT working
                    %   height: the previous value put it at z = -0.017 m,
                    %   below the table, which is why a run starting from
                    %   home had a 0.44 m error and diverged. This posture
                    %   is within a few degrees of the IK start for four of
                    %   the five scenarios.
                    % 6x1 starting arm posture (rad), from rome_9dof_uk.m.
                    % Inside the workspace and AWAY FROM THE STRETCHED
                    % SINGULARITY. A straight arm sits on that singularity,
                    % so the run does not start there.
max_rpm   = 120;    % firmware ceiling, MAX_RPM in MatlabPIDLoop.ino (rev/min).
                    % CommandGuard saturates the wheel command to this before
                    % it reaches ROMECommand.
EnableArm = 0;      % used by check_9dof and build_9dof only; the model has
                    % no separate arm sink (ROMECommand sends both)

% PLACEMENT. With EnableMotive = 0 the loop closes on the delayed command,
% so the robot must physically start at q0_9dof (base x, y, theta and the
% six joints), computed at the end of this file.



%% Orbit Parameters (scenario 1)
% Orbital Elements [a, e, i, Omega, omega, nu0]
elements   = [1.4, 0.5, 0, 0, 0, 0];
mu         = 1.0;


a = elements(1);
e = elements(2);
inc_deg = rad2deg(elements(3));
nu0_deg = rad2deg(elements(6));

%Time Calculations
T_physical = 2*pi*sqrt(a^3/mu);      % unscaled orbit period (s)

%% Case study selection
% 1 elliptical orbit | 2 V-bar | 3 R-bar | 4 NMC | 5 circle
% Scenario 1 flies the orbit defined above. Scenarios 2 to 5 take the
% manuscript parameters and their tabletop scaling from scenario_defaults.
if scenario == 1
    par = [0 0 0 0];
    D_s = 1.00;                 % scenario 1 as designed
    S_s = 0.40;
    tf_s = T_physical / S_s;
else
    [par, D_s, S_s, tf_s] = scenario_defaults(scenario);
end

% Apply the two user factors uniformly. tf scales inversely with S because a
% lap is tf = (orbit period)/S for every scenario.
time_scale = S_s  * speed_factor;
dist_scale = D_s  * size_factor;
T_scaled   = tf_s / speed_factor;

% Distance Calculations (Projected onto the floor)
% Max distance from center occurs at apogee: r = a(1+e)
max_dist_3d = a * (1 + e) * dist_scale; 
max_dist_projected = max_dist_3d;


%% 9-DOF initial seeds, solved for the selected case study
% Both depend on the trajectory, so they are re-solved whenever `scenario`,
% the orbital elements, the scaling, z_work or arm_home change. Carrying
% them as literals would let them go stale without any sign.
[q0_9dof, qd0_9dof] = scenario_seeds(scenario, elements, mu, ...
                                     time_scale, dist_scale, z_work, arm_home, dt_9dof);

% START FROM REST. scenario_seeds returns pinv(Jc)*V_des(0) as the start rate,
% nonzero in all nine coordinates and 0.166 m/s on base y. Nothing placed on a
% floor is already moving at 17 cm/s, so carrying that into the model makes the
% simulation disagree with the robot from the first step, and makes the printed
% startup card untrue.
%
% Measured by ic_factors, all five scenarios, starting from rest:
%   peak wheel speed 2.7 to 116.1 rpm, all under the ceiling, none diverging.
% The start rate is worth a few rpm; it is the ARM POSE that matters, which is
% why the card prints the joint angles and says not to skip them.
qd0_9dof = zeros(9,1);

% BASE OVERRIDE. If the user gave a measured placement, move the base there and
% re-solve the arm for it, so the end effector still starts on the reference.
% Re-solving matters: keeping the seed's arm angles under a different base pose
% puts the end effector somewhere else entirely, and ic_factors showed a 0.44 m
% offset is enough to diverge.
if ~any(isnan(base_override)) && ~EnableMotive
    q0_9dof(1) = base_override(1);
    q0_9dof(2) = base_override(2);
    q0_9dof(3) = deg2rad(base_override(3));
    [p_ref0, u_ref0] = EndEffectorTrajectory(0, scenario, elements, mu, ...
                                             time_scale, dist_scale, z_work, par);
    q0_9dof = ik9_warm_start(q0_9dof, p_ref0, u_ref0);
    q0_9dof(1:3) = [base_override(1); base_override(2); deg2rad(base_override(3))];
end

% Printout
fprintf('\n==========================================\n');
fprintf('      ORBITAL TRAJECTORY PARAMETERS       \n');
fprintf('      Date: %s                \n', datestr(now));
fprintf('==========================================\n');
fprintf('Semi-major Axis (a):    %.3f units\n', a);
fprintf('Eccentricity (e):       %.3f\n', e);
fprintf('Inclination (i):        %.2f degrees\n', inc_deg);
fprintf('Initial Anomaly (nu0):  %.2f degrees\n', nu0_deg);
fprintf('------------------------------------------\n');
fprintf('TIME ANALYSIS:\n');
fprintf('  Normalized Period:    %.2f s\n', T_physical);
fprintf('  MAX REAL TIME (Lap):  %.2f s\n', T_scaled);
fprintf('------------------------------------------\n');
fprintf('SPATIAL ANALYSIS:\n');
fprintf('  Distance Scale:       %.2f\n', dist_scale);
fprintf('  MAX REAL DISTANCE:    %.2f meters\n', max_dist_projected);
fprintf('  Required Workspace:   %.2f x %.2f meters\n', ...
        max_dist_projected*2, max_dist_projected*2);
fprintf('==========================================\n\n');

% ---------------------------------------------------------------- STARTUP
% Where the robot has to be before the run starts, in the units the hardware
% takes. Printed because the numbers change with scenario, dist_scale,
% time_scale, z_work and arm_home, and a stale copy taped to the bench is
% worse than none.
%
% WHY THE ARM POSE MATTERS AND THE START RATE DOES NOT.
% Measured by ic_factors over all five scenarios:
%
%   arm at these angles, robot at rest      works, peak 2.7 to 122.3 rpm
%   arm at these angles, seeded start rate  works, peak 2.5 to  92.8 rpm
%   arm at arm_home, robot at rest          DIVERGES, step 19 to 32
%   arm at arm_home, seeded start rate      DIVERGES, step 18 to 32
%
% So starting from rest is fine; starting with the arm at its calibrated home
% was not, until 22 Sep 2026: the OLD arm_home put the end effector at
% z = -0.017 m, below the table, 0.44 m from the reference, and every run
% all five scenarios start from it, at rest, and pass gate_all.
% Command the arm to the angles below first, as an ordinary joint move, then
% start tracking.
fprintf('==========================================\n');
fprintf('      PLACE THE ROBOT LIKE THIS           \n');
fprintf('==========================================\n');
fprintf('  Base    x %+7.3f m   y %+7.3f m   yaw %+7.2f deg\n', ...
        q0_9dof(1), q0_9dof(2), rad2deg(q0_9dof(3)));
fprintf('  Arm     %s deg\n', num2str(rad2deg(q0_9dof(4:9)).', '%+8.2f'));
fprintf('  Rates   all zero. Let it sit still before you start.\n');
fprintf('\n  Order: place base -> CAL_ARM -> EnableHardware = 1.\n');
fprintf('  The model drives the arm to the angles above by itself during\n');
fprintf('  the 10 s start hold. arm_offset_deg must be set first: with\n');
fprintf('  zeros the firmware rejects J2 and J3. GETTING_STARTED.md step 3.\n');
fprintf('==========================================\n\n');

% Safety Check
if max_dist_projected > 1.0
    warning('The orbit exceeds a 2m x 2m area! Check dist_scale or a.');
end
if T_scaled > 30
    fprintf('Note: Orbit duration is currently %.1fs (Target: <30s).\n', T_scaled);
end