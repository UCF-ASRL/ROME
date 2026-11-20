function t_ref = ref_clock_block(t, cam, arm)
%REF_CLOCK_BLOCK  Script of the MATLAB Function block RefClock in ROME_9DOF.slx (wire3 fix round 1, review I2 / Q-F6-b).
%   Copied verbatim by tools/wire_camera_guard_fix1.m (CLAUDE.md section 27). Between Clock9 and EndEffectorTrajectory
%   input 1: the reference clock is FROZEN at 0 while the run waits (camera wait or arm wait) and runs from the step at
%   which both are over, t_ref = t - t_go. So the reference starts from its start (onset included) when the robot is
%   released, not up to 2 s ahead.
%     t      simulation time (s), Clock9
%     cam    6x1 CameraGuard output; arm 4x1 ArmReady output
%     t_ref  reference time (s)
%   Mode 1 and 2: go at the first sample (t = 0), so t_ref = t - 0 = t bit for bit.
%   Limit: the run still ends at T_stop of simulation time, so a run that waited t_go ends t_go before its path end.
persistent t_go
if isempty(t_go)
    t_go = -1;                              % not released yet
end
if t_go < 0 && cam(1) == 0 && cam(3) == 0 && arm(1) ~= 0 && arm(3) == 0
    t_go = t;
end
if t_go < 0
    t_ref = 0;
else
    t_ref = t - t_go;
end
end
