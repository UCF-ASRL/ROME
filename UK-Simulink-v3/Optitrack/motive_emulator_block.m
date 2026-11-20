function pose7 = motive_emulator_block(q_pred, n, motive_cal, mocap_sigma, sensor_model, motive_emulate_noise)
%MOTIVE_EMULATOR_BLOCK  Script of the MATLAB Function block MotiveEmulator in ROME_9DOF.slx (focus plan F7, camera emulator).
%   tools/wire_motive_emulator.m copies THIS FILE, verbatim, into the chart and checks after saving that the chart
%   script equals it (CLAUDE.md section 27). Edit here, then re-run the wiring script.
%   Mode 2 (author, 19 Nov 2026: "the entire robot (vehicle + arm) with 'fake' camera measurements", "Camera emulator
%   path"): the base pose reaches the controller through the SAME camera pipeline as the real cameras, built from the
%   model's own predicted base pose. This block makes the raw camera packet pose; the Enabled Subsystem's switch
%   PoseSource (motive_source = 'emulator') sends it to MotiveFloor (motive_to_floor + yaw_unwrap), as it would send
%   MotiveData's pose7 with 'optitrack'.
%     q_pred      9x1 the model's predicted configuration, StateDelay output = q_cmd one step late (m, rad)
%     n           3x1 unit white Gaussian noise, the MocapNoise block (seeds mocap_seed), the draws SensorModel uses
%     motive_cal  chart Parameter, the define_constants struct (R, t, scale); out and back through it is exact
%                 (tests/test_motive_floor.m check i), so the placeholder (measured = false) is fine here
%     mocap_sigma, sensor_model, motive_emulate_noise   chart Parameters (define_constants)
%     pose7       7x1 [x y z qx qy qz qw] as MotiveData output 7 delivers it (Motive units, -); z = floor_to_motive of
%                 a rigid body at floor height 0 m (motive_to_floor ignores z)
%   NOISE: none unless motive_emulate_noise ~= 0 AND sensor_model ~= 0 (simulation only: a hardware run forces
%   sensor_model = 0 in define_constants and apply_pacing refuses it otherwise). Then the base gets the SensorModel
%   noise, the same expression on the same draws, added in the floor frame before the transform (Plan 2 Task 3 brief:
%   equivalent in distribution to noise after it, and it reproduces the paper's draws exactly).
%   Not done (F7 choice): no single-precision cast and no latency; both would make the emulated base differ from the
%   model's own (test (i) is exact), and the author's mode 2 asks for the pipeline, not a camera error model.
b = q_pred(1:3);                                % predicted base [x (m); y (m); psi (rad)]
if motive_emulate_noise ~= 0 && sensor_model ~= 0
    b = q_pred(1:3) + mocap_sigma(:) .* n(:);   % SensorModel's camera noise, same draws (m, m, rad)
end
pose7 = floor_to_motive(b, 0, motive_cal);      % raw camera pose (Motive units, -), 7x1
end
