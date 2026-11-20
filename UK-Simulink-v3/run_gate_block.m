function [gate, stop, reset] = run_gate_block(cam, arm)
%RUN_GATE_BLOCK  Script of the MATLAB Function block RunGate in ROME_9DOF.slx (focus plan F6; wire3 fix round 1). Copied
%   verbatim by tools/wire_camera_guard.m / wire_camera_guard_fix1.m (CLAUDE.md section 27).
%     cam    6x1 CameraGuard output [wait; hold; stop; reason; longest dropout (s); dropouts]; all 0 when the Enabled
%            Subsystem is disabled (EnableMotive = 0, the regression path: its outputs hold their initial value 0)
%     arm    4x1 ArmReady output [go; waiting; refuse; lost] (refuse also when arm telemetry stays lost after go)
%     gate   1 when the wheels may move: a good or held camera frame, no stop, the arm-ready wait passed and no
%            refusal. CmdGate multiplies CommandGuard's wheel command by it in the SAME step, so the command of the stop
%            step is already zero. The joints are gated by JointGate (parked before the run starts, held after)
%     stop   1 when the camera guard stopped (wait ran out or dropout too long) or the arm wait refused or the arm
%            telemetry stayed lost: drives Stop Simulation (via StopAny); termination then sends STOP_ALL
%     reset  1 while the camera has no first frame or the arm wait has not passed, AND at the step where either ends
%            (review I1: the first real sample re-seeds the controller; without it the integrator kept the value of the
%            step before). UKDynamics input 8: ukd_current.m re-seeds from what the controller reads (q_meas).
%   Mode 1 and 2: reset is 1 at the first sample only, where ukd_current.m seeds anyway (~started): no change.
persistent prev_wait prev_go
if isempty(prev_wait)
    prev_wait = true;                       % before the first sample: as if waiting, so the first good sample re-seeds
    prev_go = false;
end
waitNow = cam(1) ~= 0;  goNow = arm(1) ~= 0;
gate  = double(cam(1) == 0 && cam(3) == 0 && goNow && arm(3) == 0);
stop  = double(cam(3) ~= 0 || arm(3) ~= 0);
reset = double(waitNow || ~goNow || (prev_wait && ~waitNow) || (goNow && ~prev_go));
prev_wait = waitNow;  prev_go = goNow;
end
