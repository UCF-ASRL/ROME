function [pose_out, cam] = camera_guard_block(pose, valid, t, lim, seed)
%CAMERA_GUARD_BLOCK  Script of the MATLAB Function block CameraGuard in ROME_9DOF.slx (Enabled Subsystem, focus plan F6).
%   tools/wire_camera_guard.m copies THIS FILE, verbatim, into the chart and checks after saving that the chart script
%   equals it (CLAUDE.md section 27). Edit here, then re-run the wiring script.
%   Sits between MotiveFloor (floor-frame base pose) and Reshape / Discrete Derivative, so the emulator path (modes
%   1-3) and the OptiTrack path (mode 4) both pass through it. All logic is camera_guard_core.m (tested offline:
%   tests/test_camera_guard_core.m); this wrapper only keeps its state.
%     pose      3x1 MotiveFloor output [x (m); y (m); psi (rad)], NaN before MotiveData's first frame
%     valid     ValidSource output: MotiveData output 8 ('optitrack') or EmuDropout's valid ('emulator') (flag)
%     t         simulation time (s), Clock
%     lim       3x1 [dt_9dof (s); camera wait limit (s); camera dropout limit (s)], from rome_limit.m (allowed_ranges.m)
%     seed      3x1 q0_9dof(1:3), the pose output before the first good frame (not a measurement: the run is gated)
%     pose_out  3x1, always finite (camera_guard_core)
%     cam       6x1 [wait; hold; stop; reason (0 | 1 wait ran out | 2 dropout); longest dropout so far (s);
%               dropouts so far (count)]
%   State: persistent S, created at the first call of a simulation. A state not reset for a new run fails loudly
%   (camera_guard_core:time, t not after the previous sample).
persistent S
if isempty(S)
    S = camera_guard_core(3);
end
cfg = struct('dt', lim(1), 'max_wait_s', lim(2), 'max_dropout_s', lim(3), 'seed', seed(:));
[pose_out, w, h, st, S] = camera_guard_core(S, pose(:), valid, t, cfg);
cam = [w; h; st; S.reason; S.n_drop_max * lim(1); S.n_dropouts];
end
