function base = motive_floor_block(pose7, motive_cal, q0_9dof)
%MOTIVE_FLOOR_BLOCK  Script of the MATLAB Function block MotiveFloor in ROME_9DOF.slx (Enabled Subsystem, focus plan F5b).
%   tools/wire_motive_absolute.m copies THIS FILE, verbatim, into the chart and checks after saving that the chart
%   script equals it (CLAUDE.md section 27). Edit here, then re-run the wiring script.
%     pose7       7x1 RAW rigid-body pose from MotiveData output 7, [x y z qx qy qz qw] (Motive units, -); NaN(7,1)
%                 before the first valid frame (MotiveData holds the last valid pose after that).
%     motive_cal  chart Parameter = the base-workspace struct of define_constants (R 3x3 Motive -> floor, t 3x1 m,
%                 scale m per Motive unit; p_floor = scale * R * p_motive + t). Read at compile.
%     q0_9dof     chart Parameter, 9x1 start configuration; q0_9dof(3) (rad) seeds the continuous heading.
%     base        3x1 [x (m); y (m); psi (rad)] in the floor frame, psi CONTINUOUS (not wrapped to [-pi, pi]);
%                 NaN while pose7 is NaN.
%   Single implementation: motive_to_floor (absolute floor pose, heading in [-pi, pi]) and yaw_unwrap (one step of
%   psi = psi_prev + wrap(psi_raw - psi_prev)), the functions tests/test_motive_floor.m checks, called by name.
%   Unwrap state: persistent psi_prev, set to q0_9dof(3) at the first call of a simulation (persistent variables of a
%   MATLAB Function block are initialised at model start), and updated ONLY when the new heading is finite, so the
%   NaN before the first valid frame cannot leave it NaN for the rest of the run. Outside Simulink the state persists
%   between calls until "clear motive_floor_block" (tests/test_motive_model.m check n).
%   NaN pose: base is NaN and reaches StateSource / UKDynamics unguarded, the behaviour before F5b; the NaN-safe
%   start is focus plan F6 (Plan 2 Task 1b items 1-3), not this block.
persistent psi_prev
if isempty(psi_prev)
    psi_prev = q0_9dof(3);                  % heading the unwrap starts from (rad): the planned start heading
end
b = motive_to_floor(pose7, motive_cal);     % [x (m); y (m); psi in [-pi, pi] (rad)]
psi = yaw_unwrap(b(3), psi_prev);           % continuous heading nearest psi_prev (rad); NaN if b(3) is NaN
if isfinite(psi)
    psi_prev = psi;
end
base = [b(1); b(2); psi];                   % [x (m); y (m); psi (rad)]
end
