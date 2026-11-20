%% ========================================================================
%  LAB SETTINGS. TA FILE: students do not edit it (focus plan F14a, 19 Nov 2026).
%  ========================================================================
% Every lab constant the robot runs depend on, in ONE place: the arm map and signs, the firmware ranges and
% margin, the wheel ceiling, the camera-frame calibration, the simulated sensor model, the start-pose fallback
% and the clearance margin. define_constants.m runs this file from its own folder (lab_settings_file.m), right
% below "Below here is the formulation"; nothing else assigns these names (tests/test_lab_settings.m).
% THE STALE-FILE CHECK COVERS THIS FILE: define_constants stores one SHA-256 over define_constants.m AND this
% file (settings_sha.m -> dc_sha_at_run). Edit here, then re-run define_constants (or rome_mode(n)); otherwise
% the next Run is refused (arm_freeze_banner:stale). Re-calibrate the arm after any change here.
% Moved verbatim from the user block of define_constants.m (F14a); no value changed. Comments that say "below"
% or "the user block" were written there.
%
% NOT DEFINED HERE, listed so the TA knows where they are (each sits in a file that cannot read this one;
% tests/test_lab_settings.m checks every site still holds the value written here):
%   ESP32 address 192.168.4.1, TCP port 3333: ROMECommand.m (Keanu's lines kept unchanged, send log lines added: Nontunable
%     properties ESP32_IP / ESP32_Port), calibrateROMEArm.m, jog_joint.m, jogROMEArm.m, returnHomeROMEArm.m,
%     testGV.m, testLimitSwitches.m
%   Motive rigid-body Streaming ID 1, multicast group 239.255.42.99, data port 1511: Optitrack/MotiveData.m
%     (System object properties; the lab's own Motive settings are NOT VERIFIED, Part C C9.1)
%   Serial port COM4: Ground Vehicle/GroundVehicleCommand.m

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
% With the real cameras (EnableMotive = 1, 'optitrack') this is ignored: the cameras win.
base_override = [NaN; NaN; NaN];        % [x (m); y (m); yaw (deg)]

% CAMERA FRAME (EnableMotive = 1; focus plan F5, 19 Nov 2026). The cameras report the robot's rigid body in
% Motive's own frame. motive_cal turns that into the floor frame the model uses (paper LVLH: x radial,
% y in-track, z up):   p_floor = motive_cal.scale * motive_cal.R * p_motive + motive_cal.t
% and the heading is the rigid body's x axis projected on the floor, from floor +x (Optitrack/motive_to_floor.m).
% The base pose the controller then receives is ABSOLUTE: where the robot actually is on the floor.
% THE VALUES BELOW ARE NOT MEASURED. Identity, zero and 1 are placeholders until the lab frame calibration
% (Plan 3 Part C, lab/fit_motive_cal.m); only then is measured set to true. While measured is false, a run with
% EnableHardware = 1, EnableMotive = 1 and motive_source = 'optitrack' is refused (motive_cal_check.m), because
% through an unmeasured transform the controller would believe the robot is wherever Motive's origin and axes put
% it. motive_source = 'emulator' needs no calibration: its pose goes out and back through the same transform.
motive_cal = struct( ...
    'R',        eye(3), ...       % 3x3 rotation, Motive frame -> floor frame (-). UNMEASURED placeholder
    't',        zeros(3, 1), ...  % 3x1 floor-frame position of the Motive origin (m). UNMEASURED placeholder
    'scale',    1, ...            % metres per Motive unit (m/unit). UNMEASURED placeholder
    'measured', false);           % true only once the Part C values are entered above (flag)

% WHEEL CEILING. The student-tunable wheel limit. CommandGuard scales every
% wheel command down to it, the whole vector at once so the direction is
% kept, and check_9dof FAILS a run whose solve asks for more, because a
% scaled command means the real base moves slower than the model believes.
% The firmware's hard cap is 120 rev/min (#define MAX_RPM 120,
% UK-Arduino/ROME_Mega_Code/ROME_Mega_Code.ino:8). 80 was set by the author
% on 19 Nov 2026: "80 is okay. The 60 was an old estimate, but we actually
% dont know for sure. I think 80 is okay. This should be tunable on the
% define_constants though". Tuning it is Plan 2's max_rpm study (TRACKER P4).
% Change it one step at a time, re-running gate_all each time.
max_rpm = 80;          % wheel limit (rev/min), applied by CommandGuard

% MEASUREMENT MODEL (simulation, EnableMotive = 0; added 24 Sep 2026). The
% controller does not read the model's state. It reads a simulated
% measurement of it, through the SensorModel block: the base pose as the
% cameras would report it, with white Gaussian noise at the vendor's stated
% accuracy, and the joint angles as the arm firmware reports them, at one
% motor step. The rates are not measured; UKDynamics integrates its own.
% sensor_model = 0 passes the exact state through (the loop before this
% date). The plant itself is unchanged: the command one step late is the
% state the ideal actuators are taken to reach.
sensor_model = 1;                         % 1 measured state | 0 exact state
mocap_sigma  = [0.3e-3; 0.3e-3; deg2rad(0.05)];
                       % 3x1 standard deviation of the camera noise on x, y (m)
                       %   and yaw (rad). OptiTrack, "Motion Capture for
                       %   Robotics" (web page, Literature Review, accessed
                       %   24 Sep 2026): "positional error under 0.3 mm and
                       %   rotational error under 0.05 deg". The stated bound
                       %   is used as the standard deviation, which is the
                       %   conservative reading of it.
mocap_seed   = [11 12 13];                % Random Number seeds per channel, fixed so a run repeats
enc_step_deg = 1 ./ [44.44444444, 55.55555556, 55.55555556, 46.66666667, 10, 20];
                       % 1x6 one motor step per joint (deg): stepsDeg in
                       %   Arm/WorkingCode/Stepper_Commands_JacobsVer/
                       %   Stepper_Commands_JacobsVer.ino (ucf-asrl/ROME),
                       %   the table the firmware uses to convert steps to
                       %   degrees and to report the encoder position.
                       %   J1 0.0225, J2 0.0180, J3 0.0180, J4 0.0214,
                       %   J5 0.100, J6 0.050 deg.
enc_step_rad = deg2rad(enc_step_deg(:));  % 6x1 the same in rad, what SensorModel uses

% ARM JOINT CONVENTION. The model's joint angles and the Teensy firmware's
% are NOT the same numbers. Model J2 = 0 is the upper arm hanging straight
% DOWN, 90 is horizontal forward, 180 is straight up; model J3 = 90 is the
% forearm aligned with the upper arm, 0 is bent 90 deg forward. The AR3
% (ARCS 1.0 defaults, Annin_AR3_software in the Literature Review folder)
% has J2 = 0 vertical up and travels 129.6 deg forward, J3 = 0 aligned and
% travels 143.7 deg forward. The ROME firmware rebases those to J2 0..132
% and J3 1..141 with the switch at the forward / folded end. Hence
%
%     firmware_deg = arm_sign .* model_deg + arm_offset_deg
%
% with the values below DERIVED from those files, not yet measured on the
% arm. GETTING_STARTED.md step 3 is the jog test that confirms each joint;
% do it before EnableHardware = 1. J1, J4, J5, J6 are symmetric ranges, so
% only their sign is in question.
% J1: the arm is mounted so its swing plane is the base's +y (22 Sep 2026,
% after turning the arm 180 deg on the plate). The model's J1 = 0 swings
% along +x, so physical "forward" is model J1 = +90 while the firmware reads
% about 0 there: offset = -90 * arm_sign(1). Sign from jog_joint(1, 30).
arm_sign       = [ 1  -1  -1   1   1   1];   % +1 same direction as the model
arm_offset_deg = [ 0 180  90   0   0   0];   % added after the sign, degrees
% J1 offset back to 0 (23 Sep 2026): with -90 the calibration stalled on J3/J4,
% which then fold and roll through a different quadrant of the base. 0 is the
% set that calibrated cleanly at 22:40. On the flipped mount the arm then faces
% +y, not the model's +x: fine while arm_freeze = 1, to be fixed by the jog
% test (arm_sign(1), then offset -90 or +90) before tracking is attempted.

% FIRMWARE RANGES, limits[] / otherLimits[] in ROME_Teensy_Code.ino. The
% command is capped to these in ROMECommand (arm_clamp) after the map above,
% so nothing outside them is ever sent. The wheel command is capped
% separately in CommandGuard (max_rpm, scaled as a vector).
% Copied from ROME_Teensy_Code.ino as they are written there: limits[] is
% the angle the firmware assigns at each joint's limit switch, otherLimits[]
% the far end of travel. arm_clamp sorts each pair itself.
arm_fw_switch = [-180  132  141 -165   90  180];  % limits[]      (switch side)
arm_fw_other  = [ 160    0    1  165  -90 -170];  % otherLimits[] (non-switch side)

arm_fw_margin_deg = 2.0;    % cap stays this far inside BOTH ends of every
                            % joint, so a calibration off by this much still
                            % never reaches a switch. -90..90 becomes -88..88.
clearance_margin = 0.05; % m. check_9dof fails a run whose arm link points come closer than
                         % this to z = 0 (clearance_9dof z_min, whole run). z = 0 is the base
                         % plate / floor / table plane of the model; every z shifts with the
                         % unmeasured 0.20 m mount height (ik9_fk.m).
                         % [author confirmed the value 0.05 m on 19 Nov 2026 ("Confirm", TRACKER Q-T7a); it is not derived, see Q-T7a]
% LAB SIGN-OFFS (focus plan F2, read by prerun_check.m at every robot Run). The TA enters the date a part was
% signed, 'yyyy-mm-dd' (char); '' = not signed. prerun_check_core.m refuses every robot run (run_mode 2, 3, 4)
% without the Part F date (docs/lab/PART_I_RUN_PLAN.md:47), and a camera run (run_mode 4, 'optitrack') without
% the Part C date (PART_I_RUN_PLAN.md:48, :85). Editing this file changes the settings SHA: re-run
% define_constants and calibrateROMEArm afterwards (the park record carries the SHA).
signoff_partF = '';   % Part F bench sign-off date (char, 'yyyy-mm-dd'; '' = not signed)
signoff_partC = '';   % Part C camera-frame sign-off date (char, 'yyyy-mm-dd'; '' = not signed)
% SEND LOG (20 Nov 2026). ROMECommand.m logs every line it writes to the robot (START_GV, each ROME line, STOP_ALL:
% the text, the time, the step) and leaves the log in the base workspace as rome_send_log when the run ends; the
% RUN VERDICT judges "STOP_ALL sent" and "no watchdog stop" from it, and the run file keeps it (snap.send_log).
% 0 switches it off: ROMECommand then runs only Keanu's original statements and those two items read NOT MEASURED.
romecommand_sendlog = 1;   % 1 log the lines sent to the robot | 0 off (flag)
