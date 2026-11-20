function q_plant = plant_gate_block(q_cmd, gate, cam, arm, q0)
%PLANT_GATE_BLOCK  Script of the MATLAB Function block PlantGate in ROME_9DOF.slx (wire3 fix 4). Copied verbatim by
%   tools/wire_camera_guard_fix4.m (CLAUDE.md section 27). Between UKDynamics output 1 (q_cmd) and StateDelay, i.e. the
%   model's plant ("the delayed command is the state the ideal actuators reach", CLAUDE.md section 28). StateDelay feeds
%   SensorModel and MotiveEmulator, so in modes 1-3 this is ALSO the base the fake camera reports to the controller.
%   Before fix 4 StateDelay took q_cmd ungated: while CmdGate sent zero wheels (camera wait, arm-ready wait, stop) the
%   emulated base still moved by the controller's command, and the controller re-seeded from a pose the robot never
%   reached (wire3fix3 log: 0.39 mm after one waiting step; the arm wait can last 40 steps).
%     q_cmd    9x1 UKDynamics command (m, m, rad, joints rad)
%     gate     RunGate output 1: 1 when CmdGate passes the wheels
%     cam      6x1 CameraGuard output;  arm  4x1 ArmReady output [go waiting refuse lost]
%     q0       9x1 q0_9dof, the plant's start (the StateDelay initial condition)
%     q_plant  9x1 what the plant executes: base = q_cmd(1:3) while the wheel gate is open, else the last executed base
%              (held); joints = q_cmd(4:9) while JointGate passes the controller's joints (the same condition as
%              joint_gate_block.m), else the last executed joints (held: JointGate sends the park before the run starts,
%              which is q0(4:9), and the last command after)
%   Mode 1 and mode 2: both gates open from the first sample, so q_plant = q_cmd bit for bit (no arithmetic).
persistent last
if isempty(last)
    last = q0(:);
end
q_plant = last;
if gate ~= 0
    q_plant(1:3) = q_cmd(1:3);
end
if cam(1) == 0 && cam(2) == 0 && cam(3) == 0 && arm(1) ~= 0 && arm(3) == 0 && arm(4) == 0
    q_plant(4:9) = q_cmd(4:9);
end
last = q_plant;
end
