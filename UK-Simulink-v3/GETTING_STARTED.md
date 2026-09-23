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
arm_sign       = [1 1 1 1 1 1];    % model -> firmware joint map. SEE STEP 3.
arm_offset_deg = [0 0 0 0 0 0];    %   firmware = arm_sign .* model + arm_offset_deg
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

## 3. The joint convention — read this before anything moves

The model's joint angles and the Teensy's are **not the same numbers**, and
J2 does not even run the same way. Model angles, from the DH table in
`ik9_fk.m` / `ukd_current.m`:

| model angles (deg) | what the arm looks like |
|---|---|
| J2 = 0 | upper arm hanging **straight down** |
| J2 = 90 | upper arm horizontal, forward (base +x) |
| J2 = 180 | upper arm straight up |
| J2 = 180, J3 = 0 | upper arm up, forearm horizontal forward (the L) |
| J2 = 90, J3 = 90 | whole arm stretched out horizontally |
| `arm_home` (J2 = −53) | elbow **below the shoulder, behind the column** |

The AR3's shoulder leans −42° … +90° from vertical-up. In model numbers that
is roughly J2 = 222 … 90. **Model J2 = 0 and `arm_home` are postures the real
arm cannot take**, whatever the numbers are relabeled to. Sending them will
either be rejected (`ValidateTraj` returns 2, arm stays put, the run diverges
because the model thinks it moved) or, through `HOME,` during calibration,
driven into an end stop — that path has no validation.

Firmware ranges (`ROME_Teensy_Code.ino`, after `CAL_ARM` the switch side reads
`limits[]`): J1 −180…160, **J2 0…132**, **J3 1…141**, J4 −165…165,
J5 −90…90, J6 −170…180. Firmware home `[0 90 90 1 0 0]`.

What has to happen, in order:

1. **`arm_home` has to move into the reachable window** (model J2 in
   90…222, elbow above the shoulder) and `gate_all` re-run. One number,
   one re-test. Not done yet — it changes the start posture every case
   study was gated from.
2. **Measure the map, one joint at a time.** `CAL_ARM`, then send a +10°
   jog on one joint with zero wheel speed:
   ```matlab
   c = tcpclient("192.168.4.1", 3333);
   fw = [0 90 90 1 0 0];  fw(2) = fw(2) + 10;          % jog J2 by +10
   write(c, uint8(char(sprintf('ROME,0,0,0,0,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n', fw))));
   ```
   In MATLAB, from the model posture that matches what you saw after
   `CAL_ARM` (use the table above; for the L it is `[0 180 0 0 0 0]`):
   ```matlab
   q = deg2rad([0 180 0 0 0 0]); j = 2;  dq = zeros(6,1); dq(j) = deg2rad(10);
   ik9_fk([0;0;0;q(:)+dq]) - ik9_fk([0;0;0;q(:)])     % EE displacement, base frame
   ```
   Same direction → `arm_sign(j) = +1`, opposite → `−1`. Then
   `arm_offset_deg(j) = firmware_home(j) − arm_sign(j) * model_home(j)`.
3. Put the mapped start angles into `calibrateROMEArm.m` (`homePosDeg`, sent
   as `HOME,`) so calibration parks the arm where the model starts. They must
   be **firmware** numbers, inside the ranges above.

Until 1 and 2 are done, leave the defaults and keep `EnableHardware = 0`.

Known geometry gap, not fixed: the model's DH table has no shoulder offset
(`a1 = 0`); the AR3 has 64.2 mm. Every model end-effector position is off by
that much along the upper-arm direction.

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
   during the 10 s hold. Only after step 3 above is complete — a reachable
   `arm_home` and a measured `arm_sign` / `arm_offset_deg`.
4. Watch the first 12 s. Arm moves, base does not; then the base starts slowly.

## If it misbehaves

**Arm does not move on start.** The firmware rejected the angle: the
model→firmware map is wrong or unmeasured, or the commanded posture is
outside the firmware ranges. Nothing is damaged; go back to step 3.

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
