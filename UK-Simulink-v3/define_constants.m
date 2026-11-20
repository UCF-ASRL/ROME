%% ========================================================================
%  EDIT THIS BLOCK. NOTHING ELSE IN THIS FILE NEEDS TOUCHING.
%  ========================================================================
%  Set the mode with rome_mode(n) (it rewrites run_mode below and re-runs this file).
%  Lab constants (arm map, firmware ranges, wheel ceiling, camera calibration, sensor
%  model, clearance margin) are in lab_settings.m, kept by the TA.

scenario = 1;          % case study: 1 ellipse | 2 V-bar | 3 R-bar | 4 NMC | 5 circle (-)
run_mode = 1;          % 1 simulation only | 2 robot, arm frozen | 3 robot, arm moving | 4 robot + cameras (-)
speed_factor = 1.0;    % fraction of the designed speed (-): 1.0 as designed, 0.5 half speed
size_factor  = 0.50;   % fraction of the designed path size (-): 1.0 as designed, 0.5 half size

%% ========================================================================
%  Below here is the formulation. Leave it alone.
%  ========================================================================
arm_freeze_user = NaN;  % arm freeze flag the model reads (flag): NaN until the LAST line of this file sets it, so a run after a partial define_constants is refused (arm_freeze_banner)
% LAB SETTINGS (focus plan F14a, 19 Nov 2026). The TA file lab_settings.m, next to this file, defines every lab
% constant (arm_sign, arm_offset_deg, arm_fw_switch, arm_fw_other, arm_fw_margin_deg, max_rpm, motive_cal,
% base_override, sensor_model, mocap_sigma, mocap_seed, enc_step_deg, enc_step_rad, clearance_margin); nothing else
% assigns them. It is run from THIS file's folder (lab_settings_file.m), never found on the MATLAB path, and it runs
% after arm_freeze_user = NaN, so an error in it leaves the flag NaN and the model refuses to run (arm_freeze_banner).
run(lab_settings_file([mfilename('fullpath') '.m']));   % defines the lab constants above in this workspace (lab_settings.m)
% RUN MODE (focus plan F12, 19 Nov 2026). The four run flags are DERIVED from run_mode (user block) by
% run_mode_table.m, the only place the mapping lives. A run_mode other than 1-4 stops this file here
% (run_mode_table:bad, naming the four modes) with arm_freeze_user still NaN, so the model refuses to run too.
% run_mode (user block), as it was commented there before F14a (19 Nov 2026):
% THE RUN MODE, 1 to 4 (the author's four modes, 19 Nov 2026):
%   1 simulation only (end-effector dynamics sim, pose data from sim)
%   2 ground vehicle moving, frozen arm, camera off (fake camera)
%   3 ground vehicle moving, moving arm, camera off (fake camera)
%   4 ground vehicle moving, moving arm, camera on (OptiTrack; refused until motive_cal is measured)
% EnableHardware, EnableMotive, motive_source and arm_freeze are DERIVED from it by
% run_mode_table.m (below "Below here is the formulation"); they are not typed here.
% Change it here and re-run define_constants, or type rome_mode(n), or press a Mode button
% on the model's canvas (both re-run this file). Every run of this file prints the mode.
run_mode_cfg   = run_mode_table(run_mode);        % the mode's flags and banner (struct)
run_mode_test_override = {};                      % TEST HOOK (cell of flag names): empty here; only tests/dc_set_mode lines in temp copies add to it (arm_freeze_banner then lets those flags differ from the mode)
EnableHardware = run_mode_cfg.EnableHardware;     % 0 simulate only | 1 command the real robot (flag)
EnableMotive   = run_mode_cfg.EnableMotive;       % 1 the base pose reaches the controller through the camera pipeline (flag)
motive_source  = run_mode_cfg.motive_source;      % 'emulator' fake camera | 'optitrack' real cameras (char)
arm_freeze     = run_mode_cfg.arm_freeze;         % 1 joint command held at q0_9dof(4:9) | 0 the controller's command (flag)
fprintf('%s\n', run_mode_cfg.banner);             % one line naming the mode, at every run of this file (model load included)
% arm_freeze (derived above), as the user block commented it before F14a:
% ARM FREEZE. 1 = the joint command sent to the robot is the parked start
% posture for the whole run, whatever the model computes; the firmware sees
% "already at target" and never moves the arm. Needed while the Teensy still
% carries the direction bug in encoderRunToVal_nb (commanded moves on J1,
% J2, J5 run the wrong way until the range end; calibration is unaffected).
% The base still does everything the model asks, so comms, wheel map,
% cameras and the guard get exercised; END-EFFECTOR TRACKING IS NOT
% MEANINGFUL with this on, because the model believes the arm is moving.
% The freeze is run_mode 2 (arm_freeze = 1, derived below); use run_mode 3 or 4
% (arm moving) only after the Teensy is flashed with the fix and jog_joint
% shows every joint following its command.
% WARNING: ROMECommand.m L54-57 (Keanu's, kept as is, author 19 Nov 2026) sets
% arm_freeze = 0 in the workspace at every run start. So the model reads
% arm_freeze_user, copied from the derived arm_freeze at the END of this file: its switch
% ArmFreezeSwitch sends q0_9dof(4:9) to ROMECommand while it is 1. Re-run
% define_constants after changing run_mode; the Run banner says ARM FROZEN / ARM WILL MOVE.
% speed_factor and size_factor (user block), as they were commented there before F14a:
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
% speed_factor = 1.0     of the designed speed. Wheel peak scales with it:
%   0.25 gave 14.6 rpm on the ellipse (24 Sep 2026, onset model);
%   1.0 restored on 24 Sep 2026 (evening) at the author's request,
%   with max_rpm raised to 80 below so the gate can pass.
% size_factor  = 0.50    of the designed size
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
% Wheel angles (CLAUDE.md section 15). Thesis Figure 6 plus Table 1 define
% the angles counter-clockwise from +x, ascending 45/135/225/315 deg, with
% drive direction [-sin, cos]. This line lists them descending and the wheel
% map uses +sin; section 15: "descending with `+sin` equals ascending with
% `-sin` (5.6e-15), so the deployed model and the 9-DOF `UKDynamics` block
% compute the correct map." Do not change this line alone (section 15).
alphas = [deg2rad(315), deg2rad(225), deg2rad(135), deg2rad(45)]; % 1x4 wheel angles (rad), descending, used with the +sin map; see above

%% Execution Flags

%% 9-DOF integration (Part H)
dt_9dof   = 0.05;   % solver step AND the UKDynamics dt input (s). 20 Hz,
                    % matching DT_PID in MatlabPIDLoop.ino. At this step
                    % test_uk_block gave 2.829e-03 m EE RMSE on its own
                    % test trajectory (docs/HARDWARE_IMPLEMENTATION.md H1).
z_work    = 0.32;   % end-effector working height above the floor (m).
                    % Tool points DOWN (EndEffectorTrajectory), wrist just
                    % below the shoulder (0.369 m): the old 0.42 mirrored.
                    % test_uk_block started the EE at [0.220 0 0.420].
                    % manip is checked over a full lap before a new value
                    % is adopted.
arm_home  = deg2rad([-3.0; 127.0; -10.0; 0.0; -27.0; 0.0]);
                    % Elbow up and forward, forearm down, tool pointing at
                    % the floor. This is the previous start (J2 = -53, which
                    % the AR3 cannot reach) turned 180 deg about the
                    % shoulder axis, so its conditioning s = 0.68 carries
                    % over. Inside every firmware range: J2 127 -> fw 53,
                    % J3 -10 -> fw 100, J5 -27.
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
tool_tilt_deg = 0;  % tilt of the tool z axis from straight down, about the tool's
                    %   own horizontal (y) axis (deg). 0 = tool at the floor, as every
                    %   run before 24 Sep 2026. Positive tips the tool tip toward the
                    %   chief. Enters only the reference attitude (EndEffectorTrajectory,
                    %   trailing optional argument); the start posture follows through
                    %   scenario_seeds, and check_9dof rebuilds the reference with it.
                    %   Probed on scenario 1 at +15 and -15 deg (24 Sep 2026): both seed
                    %   and screen stable, s 0.683 / 0.667 against 0.677 at 0, no joint
                    %   clamped, J5 -13.2 / -33.7 firmware deg against -23.8.
% max_rpm is in lab_settings.m (80, author 19 Nov 2026); the firmware's
% own cap is 120 (ROME_Mega_Code.ino:8).
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

% RUN LENGTH. EndEffectorTrajectory starts from rest with a raised-cosine onset
% over T_ON, after which its clock lags t by T_ON/2, so the path ends at
% T_scaled + T_ON/2. Every simulation path runs to T_stop: the model's StopTime
% ('T_stop'), check_9dof / gate_all (through the model) and run_9dof_cases.
T_ON   = 10.0;                % reference onset (s), must equal EndEffectorTrajectory.m T_ON (tests/test_run_length.m parses both)
T_stop = T_scaled + T_ON/2;   % run length (s): the whole path, onset included

% SENSOR MODEL OFF ON HARDWARE (final review I5(4), 19 Nov 2026). With
% sensor_model ~= 0 the SensorModel block adds synthetic camera noise
% (mocap_sigma) and joint quantisation (enc_step_rad) to the state the
% controller reads. That noise is simulation-only: on the robot the
% controller must read the real measurement, not a made-up error on top of
% it. Lab main has no SensorModel block. apply_pacing (below, and at every
% run start) refuses a hardware run with sensor_model ~= 0, so this override
% must stay ABOVE the apply_pacing call.
if EnableHardware ~= 0
    sensor_model = 0;  % forced: 0 exact state to the controller (flag)
    fprintf('define_constants: EnableHardware ~= 0, so sensor_model is forced to 0 (synthetic sensor noise is simulation-only).\n');
end

% PACING. Real-time pacing of the model only when the real robot is commanded
% (EnableHardware = 1); a simulation-only run goes as fast as it can. The rule
% lives in apply_pacing.m, which the model also calls from its InitFcn at every
% run start. Set here only when the model is loaded, so this file still runs
% with no model open.
% CAMERA SOURCE (focus plan F7). motive_source must be 'emulator' or 'optitrack'
% (motive_source_check.m); anything else stops this file here. The model's
% Constant blocks PoseSelect and MotiveReaderEnable call the same function at
% every compile.
% CAMERA CALIBRATION REFUSAL (focus plan F5b, narrowed by F7). EnableHardware = 1
% with EnableMotive = 1 and motive_source = 'optitrack' stops this file here, with
% the reason, while motive_cal.measured is false (motive_cal_check.m; apply_pacing
% calls it again at every run start). Stopping here leaves arm_freeze_user = NaN,
% so the model refuses to run as well. This file passes its OWN values: tools run
% it inside their function workspace, where the base may be empty or stale.
motive_source_check(motive_source);
motive_cal_check(EnableHardware, EnableMotive, motive_source, motive_cal);
% EMULATOR NOISE (focus plan F7; simulation only). 1 adds the SensorModel camera
% noise (mocap_sigma, the same MocapNoise draws) to the emulated pose, so mode 1
% (EnableMotive = 1, 'emulator') reproduces the paper's noisy runs (Table 3)
% to round-off (tests/test_motive_emulator.m (ii)). It only acts while
% sensor_model ~= 0, which a hardware run forces to 0 (above) and refuses
% otherwise (apply_pacing): modes 2 and 3 on the robot never see it.
motive_emulate_noise = 1;  % 1 SensorModel noise on the emulated camera pose (sim only) | 0 noise-free (flag)
if bdIsLoaded('ROME_9DOF')   % this file's OWN values, not the base workspace (F7 fix round 1, I1)
    apply_pacing('ROME_9DOF', struct('EnableHardware', EnableHardware, 'EnableMotive', EnableMotive, ...
        'sensor_model', sensor_model, 'motive_source', motive_source, 'motive_cal', motive_cal));
end

% Distance Calculations (Projected onto the floor)
% Max distance from center occurs at apogee: r = a(1+e)
max_dist_3d = a * (1 + e) * dist_scale; 
max_dist_projected = max_dist_3d;


%% 9-DOF initial seeds, solved for the selected case study
% Both depend on the trajectory, so they are re-solved whenever `scenario`,
% the orbital elements, the scaling, z_work or arm_home change. Carrying
% them as literals would let them go stale without any sign.
[q0_9dof, qd0_9dof] = scenario_seeds(scenario, elements, mu, ...
                                     time_scale, dist_scale, z_work, arm_home, dt_9dof, tool_tilt_deg);
% The parked start posture in firmware degrees: what calibrateROMEArm sends,
% and what ROMECommand sends while arm_freeze_user = 1 (ArmFreezeSwitch, model).
arm_freeze_deg = arm_clamp(arm_sign(:).' .* rad2deg(q0_9dof(4:9)).' + arm_offset_deg(:).', ...
                           arm_fw_switch, arm_fw_other, arm_fw_margin_deg);

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
if ~any(isnan(base_override)) && ~(EnableMotive && strcmp(motive_source, 'optitrack'))   % F7: the fake camera honours a typed placement
    q0_9dof(1) = base_override(1);
    q0_9dof(2) = base_override(2);
    q0_9dof(3) = deg2rad(base_override(3));
    [p_ref0, u_ref0] = EndEffectorTrajectory(0, scenario, elements, mu, ...
                                             time_scale, dist_scale, z_work, par, tool_tilt_deg);
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
fprintf('\n  Order: set run_mode FIRST (rome_mode(n)) -> place base -> calibrateROMEArm -> Run.\n');
fprintf('  calibrateROMEArm parks the arm at the angles above (mapped to\n');
fprintf('  firmware degrees). arm_sign / arm_offset_deg are derived from\n');
fprintf('  the AR3 files, not measured (lab_settings.m, kept by the TA). Steps:\n');
fprintf('  docs/student_procedure/ROME_student_sheet.pdf. Joint commands are capped to\n');
fprintf('  the firmware ranges, wheels to max_rpm.\n');
fprintf('==========================================\n\n');

% Safety Check
if max_dist_projected > 1.0
    warning('The orbit exceeds a 2m x 2m area! Check dist_scale or a.');
end
if T_scaled > 30
    fprintf('Note: Orbit duration is currently %.1fs (Target: <30s).\n', T_scaled);
end
dc_sha_at_run = settings_sha([mfilename('fullpath') '.m']);  % SHA-256 (hex char) over this file AND lab_settings.m as run (F14a); arm_freeze_banner refuses a Run when either changed since (stale workspace)
% ARM FREEZE, the value the model uses (Plan 2 Task 5 / focus plan F1, 19 Nov 2026).
% ROMECommand.m L56 writes arm_freeze = 0 into the base workspace when a run
% starts (every run, also EnableHardware = 0). The model's Constant is read at
% compile, before that write, so a FIRST run would still see 1, but every later
% Run of the same session would see 0 and move the arm (measured:
% docs/verification/2026-11-19_f1_eval_order_probe.log). Hence this copy, which
% ROMECommand never writes. Read by ArmFreezeSwitch (tools/wire_arm_freeze.m)
% and by the banner arm_freeze_banner.m (model InitFcn).
arm_freeze_user = arm_freeze;   % student's arm_freeze, captured last (flag): 1 joint command held at q0_9dof(4:9), 0 controller's command