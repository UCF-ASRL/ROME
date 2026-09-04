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
EnableHardware = 0;
EnableMotive = 0;

%% 9-DOF integration (Part H)
dt_9dof   = 0.05;   % solver step AND the UKDynamics dt input (s). 20 Hz,
                    % matching DT_PID in MatlabPIDLoop.ino. Validated in H1:
                    % 2.829e-03 m EE RMSE, 21.6 rpm peak.
z_work    = 0.42;   % end-effector working height above the table (m).
                    % test_uk_block started the EE at [0.220 0 0.420].
                    % manip is checked over a full lap before a new value
                    % is adopted.
arm_home  = [0.0; -0.5; 0.9; 0.0; 0.7; 0.0];
                    % 6x1 starting arm posture (rad), from rome_9dof_uk.m.
                    % Inside the workspace and AWAY FROM THE STRETCHED
                    % SINGULARITY. A straight arm sits on that singularity,
                    % so the run does not start there.
EnableArm = 0;      % gate the arm serial sink, independent of EnableHardware
k_th = 0.4; % heading posture stiffness (1/s^2). This gain, not the
            % lap time, sets the peak wheel speed: the peak comes
            % from the heading capture transient. 1.0 peaks at
            % 119.7 rpm against the 120 rpm firmware ceiling;
            % 0.4 peaks at 98.7 rpm. Start low on hardware.
            % Sweep it with sweep_kth.

% FRAME / PLACEMENT WARNING. The heading task points the vehicle at
% the origin using th_ref = atan2(-q(2), -q(1)), i.e. the MATHS
% convention, 0 = +x. InitialConditions seeds the integrator with
% q0(3) = 0, so model heading zero IS world +x, and:
%
%     THE ROBOT MUST BE PLACED POINTING ALONG WORLD +x AT t = 0.
%
% Nothing measures or corrects a placement error: the base has no
% heading feedback until Stage 2 closes the OptiTrack loop.
%
% The heading_offset = -pi/2 inside OrbitTrajectory and
% InitialConditions (robot 0 = North vs maths 0 = East) is DEAD CODE
% today, because both blocks zero their theta output. If those
% commented-out -theta lines are ever restored, that offset and the
% heading task will disagree by pi/2 and in sign. See
% docs/HARDWARE_IMPLEMENTATION.md Part F 1.4 and Part A6.



%% Case study selection (Part H)
% 1 circle | 2 V-bar | 3 R-bar | 4 NMC | 5 elliptical orbit
% Change this one number to switch case study. scenario_defaults supplies the
% manuscript parameters and the tabletop scaling for whichever is selected.
scenario = 1;
[par, dist_scale, time_scale, T_scaled] = scenario_defaults(scenario);

%% Orbit Parameters
% Orbital Elements [a, e, i, Omega, omega, nu0]
elements = [10, 0.5, 0, 0, 0, 0];
mu = 1.0;
% Elements: [a, e, i, Omega, omega, nu0]
a = elements(1);
e = elements(2);
inc_deg = rad2deg(elements(3));
nu0_deg = rad2deg(elements(6));

%Time Calculations
T_physical = 2 * pi * sqrt(a^3 / mu);
T_physical = 2*pi*sqrt(a^3/mu);   % unscaled orbit period (s)

% Distance Calculations (Projected onto the floor)
% Max distance from center occurs at apogee: r = a(1+e)
max_dist_3d = a * (1 + e) * dist_scale; 
max_dist_projected = max_dist_3d;


%% 9-DOF initial seeds, solved for the selected case study
% Both depend on the trajectory, so they are re-solved whenever `scenario`,
% the orbital elements, the scaling, z_work or arm_home change. Carrying
% them as literals would let them go stale without any sign.
[q0_9dof, qd0_9dof] = scenario_seeds(scenario, elements, mu, ...
                                     time_scale, dist_scale, z_work, arm_home);

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

% Safety Check
if max_dist_projected > 1.0
    warning('The orbit exceeds a 2m x 2m area! Check dist_scale or a.');
end
if T_scaled > 30
    fprintf('Note: Orbit duration is currently %.1fs (Target: <30s).\n', T_scaled);
end