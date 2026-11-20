function q_arm = tel_or_park_block(tel, arm, park)
%TEL_OR_PARK_BLOCK  Script of the MATLAB Function block TelOrPark in ROME_9DOF.slx (wire3 fix round 1, review C2/I1).
%   Copied verbatim by tools/wire_camera_guard_fix1.m (CLAUDE.md section 27). Between TelemetryDelay (ROMECommand output 2
%   one step late, model rad) and JointSource input 1, i.e. the arm angles the controller reads in modes 3 and 4.
%     tel    6x1 delayed telemetry (model rad); before the first ARM line it is ROMECommand's seed rad2deg(arm_home)
%     arm    4x1 ArmReady [go waiting refuse lost]; ArmReady reads the same one-step-late outputs, so go at step k means
%            the telemetry delivered at step k is a real ARM line
%     park   6x1 q0_9dof(4:9) (model rad), where calibrateROMEArm parked the arm
%     q_arm  tel once go is given; the parked posture before. So the controller, re-seeded every step of the wait
%            (RunGate reset), is seeded from the parked posture, never from arm_home, and from the first real line at go.
%   Mode 1 and 2: JointSource takes SimJoints, this output is not read.
if arm(1) ~= 0
    q_arm = tel(:);
else
    q_arm = park(:);
end
end
