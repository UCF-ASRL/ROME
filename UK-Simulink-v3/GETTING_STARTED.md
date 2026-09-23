# Running the 9-DOF model

Everything you change is the block at the top of `define_constants.m`. Below
the line marked *"Below here is the formulation"* is the solve; leave it.

Verified 22 September 2026: all five scenarios pass `gate_all` from the
calibration home, at rest, in the Simulink model. Peaks 1.4 to 33.2 rpm,
tracking within limits.

---

## 1. The block you edit

```matlab
scenario       = 1;                % 1 ellipse | 2 V-bar | 3 R-bar | 4 NMC | 5 circle
EnableHardware = 0;                % 1 commands the real robot
EnableMotive   = 0;                % 1 closes the loop on the cameras
base_override  = [NaN; NaN; NaN];  % [x m; y m; yaw deg], only with cameras off
speed_factor   = 0.50;             % of the designed speed. Slow on purpose.
size_factor    = 0.50;             % of the designed size. 2.1 x 2.1 m of floor.
arm_offset_deg = [0 0 0 0 0 0];    % model -> firmware joint offset. SEE STEP 3.
```

Save, reopen the model. `define_constants` runs on every load.

`speed_factor` and `size_factor` apply to every scenario. Halving size costs
nothing in timing; it only shrinks the orbit. Turn either up one step at a
time and re-run `gate_all`.

## 2. What the model does at the start

- **10 s hold.** The reference is frozen at its starting point for the first
  10 s, so the arm settles onto it while the base sits still.
- **2 s wheel ramp.** After the hold the wheel command rises linearly to full
  over 2 s instead of stepping.
- **Starts from rest.** No initial velocity is assumed anywhere.

The arm's calibration home now sits at working height (`z = 0.419 m`). The
old one put the end effector 1.7 cm *below the table*, which is why any run
from home diverged.

## 3. The joint convention — do this once, before anything moves

The model's joint angles and the Teensy's are **not the same numbers**.
Firmware ranges (`ROME_Teensy_Code.ino`):

| joint | range (deg) |
|---|---|
| J1 | −180 … 160 |
| J2 | **0 … 132** |
| J3 | **1 … 141** |
| J4 | −165 … 165 |
| J5 | −90 … 90 |
| J6 | −170 … 180 |

Firmware home is `[0 90 90 1 0 0]`. The model's start has J2 ≈ −53°, which
the firmware **rejects** (`ValidateTraj` returns 2; the arm stays put; the run
then diverges because the model thinks it moved).

To find the offset:

1. `CAL_ARM`. Arm goes to firmware home.
2. Read `actualJointDeg` from the `ARM,` telemetry line. Expect `[0 90 90 1 0 0]`.
3. Look at the arm. In MATLAB, `ik9_fk([0;0;0;deg2rad(q)])` gives the end-effector
   position for model angles `q`. Find the `q` whose picture matches what you see.
   Likely candidates: `[0 0 0 1 0 0]` (offset would be `[0 90 90 0 0 0]`) or
   `[0 90 90 1 0 0]` (offset zero).
4. Set `arm_offset_deg = firmware − model` for that posture. Save.

Until this is done, leave zeros and keep `EnableHardware = 0`.

## 4. Simulation first

```matlab
check_9dof      % selected scenario, errors on failure
gate_all        % all five
```

## 5. On the robot

1. Place the base in the camera area. With `EnableMotive = 1` the cameras
   report where it is. Cameras off: put it where the printout says, or type the
   measured pose into `base_override`.
2. `CAL_ARM`.
3. `EnableHardware = 1` and run. The model commands the arm to the start pose
   during the 10 s hold — with `arm_offset_deg` set, the firmware accepts it.
4. Watch the first 12 s. Arm moves, base does not; then the base starts slowly.

## If it misbehaves

**Arm does not move on start.** `arm_offset_deg` is wrong or zero. The
firmware rejected the angle. Nothing is damaged; fix the offset.

**Base does not start.** Mega waits for `'Y'`; the ESP32 translates
`START_GV` into it. Check the ESP32 is on the WiFi sketch and connected.

**Base moves wrongly.** Bench-test wheel order: jack it up, one wheel at a
time. `verify_wheelmap` proves the model is self-consistent; it cannot see the
wiring.

**It will not stop.** `STOP_ALL` → `!` → Mega returns *before* writing PWM,
so the last command stays applied. **Cut power.** Known gap.

**Camera mode misbehaves.** `EnableMotive = 1` has never been run. Fall back
to `EnableMotive = 0` with `base_override` set; that path is verified.

## Do not change without re-running `gate_all`

`arm_home`, anything below the line in `define_constants.m`, `T_HOLD` in
`EndEffectorTrajectory.m`, `RAMP_STEPS` in `CommandGuard.m`.
