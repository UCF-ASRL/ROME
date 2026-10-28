function [wheel_out, joint_out, guard_flag] = CommandGuard(wheel_in, joint_in, max_rpm, arm_home)
%#codegen
%COMMANDGUARD  Last check between the solve and the hardware link.
%
%   Sits between UKDynamics and ROMECommand. Nothing downstream limits a
%   command: the Mega maps +/-max_rpm onto +/-255 PWM with an integer map()
%   and no clamp, so a larger value drives the pins to full or arbitrary
%   duty, and a non-finite value leaves its PI integral non-finite until the
%   board is reset. The three recorded runs of 16 September 2026 sent 197 to
%   1036 rpm on their first packet and non-finite values within a second.
%
%   RULES
%     1. A non-finite wheel or joint command is never transmitted. The wheels
%        are commanded to rest and the last finite joint command is held.
%        Before any finite command has arrived the hold is arm_home, the
%        posture the arm is placed in before a run -- NOT zeros, which is the
%        stretched singularity and would command a large uncommanded motion.
%     2. Wheel commands are limited by scaling the whole 4-vector, not by
%        clipping each wheel. Four omni wheels drive three base freedoms, so
%        the achievable set satisfies -w1 + w2 - w3 + w4 = 0; clipping one
%        wheel breaks that and the wheels fight each other. Scaling keeps the
%        direction of motion and only slows it.
%     3. Joint commands pass through when finite. They are capped later, in
%        ROMECommand (arm_clamp), AFTER the model-to-firmware map, against the
%        firmware ranges limits[] / otherLimits[] of ROME_Teensy_Code.ino,
%        pulled in by arm_fw_margin_deg. The cap has to sit after the map
%        because the model's joint zero is not the firmware's
%        (define_constants, ARM JOINT CONVENTION).
%
%   INPUTS
%     wheel_in   4x1  commanded wheel speeds from UKDynamics (rev/min)
%     joint_in   6x1  commanded arm joint angles (rad)
%     max_rpm    1x1  firmware ceiling, MAX_RPM in MatlabPIDLoop.ino (rev/min)
%     arm_home   6x1  posture the arm starts in (rad), the initial hold
%
%   OUTPUTS
%     wheel_out  4x1  wheel speeds safe to transmit (rev/min)
%     joint_out  6x1  joint angles safe to transmit (rad)
%     guard_flag 1x1  0 clean, 1 scaled back to the ceiling, 2 non-finite
%                     command blocked. Logged as guard_flag; a run that ends
%                     with any 2 in it did not execute what the solve asked.

persistent joint_hold started kstep
if isempty(started)
    started    = true;
    joint_hold = arm_home(:);   % 6x1 hold before any finite command (rad)
    kstep      = 0;             % steps since the run began, for the soft start
end

% SOFT START. From a standstill the solve asks for 116 rpm within the first
% second: the end effector has to catch a reference that is already moving.
% That is within the ceiling but it is a step change into a stationary drive
% train. The wheel command is ramped linearly over RAMP_STEPS so the base
% accelerates instead of being kicked. The joint command is NOT ramped: the
% arm is already at the commanded pose and ramping it would walk it away.
%
% 40 steps at dt = 0.05 s is 2 s. The reference has moved about 7 percent of
% a 27.4 s lap by then, which the attractor closes afterwards.
RAMP_STEPS = 40;
kstep = kstep + 1;
ramp  = min(1.0, kstep/RAMP_STEPS);

w = wheel_in(:);            % 4x1 (rev/min)
j = joint_in(:);            % 6x1 (rad)

if ~all(isfinite(w)) || ~all(isfinite(j))
    wheel_out  = zeros(4,1);            % Rule 1
    joint_out  = joint_hold;
    guard_flag = 2;
    return
end

joint_hold = j;             % this command is finite, so it becomes the hold
joint_out  = j;

w = w*ramp;                 % soft start, see above. Scales the whole vector,
                            %   so the consistency condition n'w = 0 holds.

lim  = abs(max_rpm);
peak = max(abs(w));
if peak > lim                           % Rule 2: scale, do not clip
    wheel_out  = w*(lim/peak);
    guard_flag = 1;
else
    wheel_out  = w;
    guard_flag = 0;
end
end
