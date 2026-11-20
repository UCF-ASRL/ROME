function q_out = joint_gate_block(q_cmd, cam, arm, park)
%JOINT_GATE_BLOCK  Script of the MATLAB Function block JointGate in ROME_9DOF.slx (wire3 fix round 1, review C2).
%   Copied verbatim by tools/wire_camera_guard_fix1.m (CLAUDE.md section 27). Between ArmFreezeSwitch (the joint command:
%   the controller's in modes 1, 3, 4; q0_9dof(4:9) in mode 2) and ROMECommand input 2 / To Workspace joint_sent_rad.
%     q_cmd  6x1 joint command (model rad)
%     cam    6x1 CameraGuard [wait hold stop reason longest_s n]; all 0 with EnableMotive = 0
%     arm    4x1 ArmReady [go waiting refuse lost]
%     park   6x1 q0_9dof(4:9) (model rad): the start posture calibrateROMEArm parks the arm at (F11: the park record
%            holds exactly this posture through the map and clamp)
%     q_out  6x1 joint command sent (model rad)
%   The joints follow the controller only while it runs on real data: a good camera frame (no wait, hold or stop), the
%   arm-ready go given, no refusal and no telemetry loss. Otherwise:
%     before the controller ever ran (camera wait, arm wait, or a stop before the first such step): the PARKED posture,
%       never ROMECommand's arm_home seed, so the arm does not move from where calibrateROMEArm left it;
%     after it ran (camera dropout hold, telemetry loss, stop): the LAST joint command sent, held. Not the parked posture:
%       mid-run the arm is away from the park, and commanding the park would move it (the old CommandGuard NaN hold,
%       CommandGuard.m:66-70, also held the last command, which F6 removed by taking the NaN away).
%   Mode 1 and mode 2: go at the first sample and good frames, so q_out = q_cmd bit for bit (no arithmetic).
persistent last started
if isempty(started)
    started = false;
    last = park(:);
end
running = cam(1) == 0 && cam(2) == 0 && cam(3) == 0 && arm(1) ~= 0 && arm(3) == 0 && arm(4) == 0;
if running
    q_out = q_cmd(:);
    last = q_out;
    started = true;
elseif ~started
    q_out = park(:);
else
    q_out = last;
end
end
