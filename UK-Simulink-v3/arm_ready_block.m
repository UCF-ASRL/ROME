function arm = arm_ready_block(armReady, jointDeg, t, use_tel, lim, home_deg)
%ARM_READY_BLOCK  Script of the MATLAB Function block ArmReady in ROME_9DOF.slx (focus plan F6; wire3 fix round 1).
%   Copied verbatim by tools/wire_camera_guard.m / wire_camera_guard_fix1.m (CLAUDE.md section 27). The wait logic is
%   arm_ready_core.m (tests/test_camera_guard_core.m (l)-(q)); this wrapper keeps its state and adds the LOSS rule.
%     armReady  ROMECommand output 3, one step late (ArmReadyDelay), flag
%     jointDeg  6x1 ROMECommand output 2 (model deg), one step late (ArmDegDelay, IC rad2deg(arm_home(:)))
%     t         simulation time (s), Clock9
%     use_tel   ArmWaitOn: 1 when the controller reads arm telemetry (robot, arm moving, EnableMotive ~= 0)
%     lim       3x1 [dt_9dof (s); arm wait limit (s); arm telemetry loss limit (s)], rome_limit('arm_wait')
%     home_deg  6x1 rad2deg(arm_home(:)), ROMECommand's seed (model deg)
%     arm       4x1 [go; waiting; refuse; lost] (flags; go and refuse latched)
%   LOSS after go (wire3 review I4): ROMECommand drops armReady 0.5 s after the last GV/ARM line (ROMECommand.m:182-184)
%   and keeps outputting the last angles, so the controller would close its loop on frozen readings. While armReady is
%   false after go, lost = 1 (JointGate holds the last joint command; the wheels keep running, as in a camera hold);
%   after lim(3)/dt such samples the next one sets refuse (latched): RunGate stops the run. A READY sample ends a loss.
%   Limit counted in samples, like the camera guard. use_tel = 0 (modes 1, 2): never lost.
persistent S nlost stopLost
if isempty(S)
    S = arm_ready_core();
    nlost = 0;
    stopLost = false;
end
cfg = struct('use_telemetry', use_tel, 'home_deg', home_deg(:), 'dt', lim(1), 'max_wait_s', lim(2));
[go, waiting, refuse, S] = arm_ready_core(S, armReady, jointDeg(:), t, cfg);
lost = 0;
if go ~= 0 && use_tel ~= 0 && ~(double(armReady) == 1)
    nlost = nlost + 1;
    lost = 1;
    if nlost > round(lim(3) / lim(1))
        stopLost = true;
    end
elseif go ~= 0
    nlost = 0;
end
if stopLost
    refuse = 1;
end
arm = [double(go); double(waiting); double(refuse); double(lost)];
end
