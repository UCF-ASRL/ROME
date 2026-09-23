# Running the 9-DOF model

Everything you change is the block at the top of `define_constants.m`. Below
the line marked *"Below here is the formulation"* is the solve; leave it.
Every other file that talks to the robot reads that block: `ROMECommand`
(joint map and cap), `CommandGuard` (wheel ceiling, start hold),
`calibrateROMEArm` (parking pose), `check_9dof` / `gate_all` (the gate).
Change a number there, re-run `gate_all`, and all of them follow.

**Verification status, 22 September 2026, 22:40 — `gate_all` passed.**
With the code as pushed: all five scenarios pass `check_9dof` in the
Simulink model (two static checks, constraint residual < 1e-11, wheel peaks
18.9 / 1.7 / 1.7 / 5.2 / 3.7 rpm against the 60 rpm ceiling, min s
0.675-0.678 over every run, steady errors 0.08-1.65 mm under the lag bound,
joints never closer than 34 deg to a firmware limit). `derivation_checks`:
all passed. `check_guard`: 10/10. Placement sweep: 45 of 45 runs (each
scenario from the card plus eight placements up to 15 cm / 15 deg off it and
the arm up to 5 deg off, all from rest) complete inside the joint window,
worst peak 34 rpm. The one item simulation cannot clear is the joint
convention on the real arm (step 3): on the first lab try J1, J2 and J5
moved the opposite way to their targets — a Teensy encoder-loop issue, not
the numbers sent. Measure it with `jog_joint` before `EnableHardware = 1`.

---

## 1. The block you edit

```matlab
scenario       = 1;                % 1 ellipse | 2 V-bar | 3 R-bar | 4 NMC | 5 circle
EnableHardware = 0;                % 1 commands the real robot
EnableMotive   = 0;                % 1 closes the loop on the cameras
base_override  = [NaN; NaN; NaN];  % [x m; y m; yaw deg], only with cameras off
speed_factor   = 0.25;             % of the designed speed. Slow on purpose.
size_factor    = 0.50;             % of the designed size. 2.1 x 2.1 m of floor.
max_rpm        = 60;               % wheel ceiling (firmware's own cap is 120)
arm_sign       = [ 1 -1 -1  1  1  1];   % model -> firmware joint map, SEE STEP 3
arm_offset_deg = [-90 180 90  0  0  0]; %   firmware = arm_sign .* model + arm_offset_deg
arm_fw_switch / arm_fw_other       % limits[] / otherLimits[], copied from ROME_Teensy_Code.ino
arm_fw_margin_deg = 2.0;           % the cap stays this far inside both ends
arm_freeze     = 1;                % 1 = arm stays parked all run; base only. SEE BELOW
```

**`arm_freeze` (23 Sep 2026).** While the Teensy still has the direction
bug in `encoderRunToVal_nb`, a commanded move on J1, J2 or J5 runs the
wrong way to the range end. Calibration is unaffected (different path), but
the model's joint commands change every step once the hold ends. With
`arm_freeze = 1` `ROMECommand` sends the parked posture for the whole run,
so the arm never moves and the base does everything the model asks: a
pipeline test (comms, wheel map, cameras, guard), **not** a tracking test —
the model believes the arm is moving. Set it to 0 only after flashing the
fixed sketch and confirming every joint with `jog_joint`.

Save, reopen the model. `define_constants` runs on every load and prints the
card: where to put the base, what the arm angles are, in model degrees.

`speed_factor` and `size_factor` apply to every scenario. The wheel peak
scales with `speed_factor`; at 0.50 the solve asked for 113 rpm, at 0.25 it
stays under the 60 rpm ceiling. Raise either one step at a time, `gate_all`
after each.

## 2. What the model does

- **Tool points at the floor.** The end effector works at `z_work = 0.32 m`
  with the tool z-axis down, yaw following the pointing law. With the tool
  up, every start the AR3 can physically take was badly conditioned
  (s <= 0.38, all five laps diverged); pointing it down is the same arm
  turned 180 deg about the shoulder, which keeps the conditioning that was
  tested (s = 0.68) and puts the elbow above the shoulder.
- **Start posture `arm_home`** = model `[-3 127 -10 0 -27 0]` deg: elbow up
  and forward, forearm down, tool at the floor about 0.34 m in front of
  the base axis. Firmware `[-3 53 100 0 -27 0]`.
- **10 s hold.** The reference is frozen for the first 10 s.
- **2 s wheel ramp**, then `CommandGuard` scales the whole wheel vector to
  `max_rpm` if the solve ever asks for more (it should not: the gate fails
  a run that does).
- **Joint cap.** After the map to firmware degrees, `arm_clamp` pulls every
  joint inside `[lo + margin, hi - margin]`. The gate also fails a run whose
  commands come within the margin of a limit, so on a gated run the cap
  never acts.
- **Starts from rest.** No initial velocity anywhere.
- **Camera loop.** With `EnableMotive = 1`, `StateSource` feeds the solve
  the Motive base pose and the arm's reported joints (converted back to
  model degrees inside `ROMECommand`) instead of its own delayed command.

## 3. The joint convention — confirm it once, before anything moves

Model angles and firmware angles are different numbers. From the AR3's own
files (`Literature Review/Annin_AR3_software`, `ARbot.cal`) and the ROME
Teensy code:

| | model | AR3 / ROME firmware |
|---|---|---|
| J2 = 0 | upper arm hanging straight down | vertical up |
| J2 range | 48 … 180 (reachable) | 0 … 132, switch at the forward end |
| J3 = 0 | forearm bent 90 deg forward | forearm aligned with upper arm |
| J3 range | −51 … 89 | 1 … 141 |
| J5 | ±90 | ±90 |

So `firmware J2 = 180 − model J2`, `firmware J3 = 90 − model J3`. J1: the
arm is mounted with its swing plane along the base's **+y**, which is model
J1 = +90 while the firmware reads ≈ 0 there, so `firmware J1 = model J1 −
90` (−90 becomes +90 if the J1 jog runs opposite to the model). J4, J5, J6
are symmetric ranges and only their sign is in question. Those are the
defaults in the block. They are **derived, not measured**. The jog test:

1. `calibrateROMEArm`. It sends `HOME` with the scenario's start posture
   (firmware degrees) first, then `CAL_ARM`, so the calibration itself ends
   at the start posture; then it watches the `ARM,` line and prints a
   per-joint verdict. Note the firmware reports its forced home values
   right after calibrating, so "at target" there is not proof — look at
   the arm. J4 has no encoder; its reading is always the command.
2. Jog one joint +10 deg in firmware with zero wheel speed:
   ```matlab
   jog_joint(2, 10)      % J2, +10 deg: prints reading before/after, feeds the watchdog
   ```
   What the 22 Sep 2026 logs established: calibration moves (switch to
   home, up to 178 deg) converge on every joint's encoder, so the encoders
   count in the right sense. Commanded moves went the wrong way on J1, J2,
   J5. The difference is in `encoderRunToVal_nb`: it calls `setSpeed(+-
   runSpeed)` every iteration with a sign taken from `path`, and AccelStepper
   steps in that direction whenever `moveTo` sees an unchanged target.
   Calibration passes `path = 3` (sign `+negspeeds`, right for switch->home);
   a command goes through `ValidateTraj`'s cw/ccw search, `path` 0 or 1,
   whose sign is wrong on those joints, so each step goes the wrong way and
   the reading (correct) drifts off. Fix in the sketch (applied, needs
   flashing): the step sign that increases a joint's angle is
   `negspeeds[x] * sgn(otherLimits[x] - limits[x])` (on J1, J2, J3, J5
   positive steps DEcrease the angle; on J4, J6 they increase it), so
   `setSpeed(runSpeed * negspeeds[x] * toward * sgn(error))` replaces the
   `path` branches; for calibration it reduces to the old `+negspeeds`.
   `negspeeds[]` tuned by trial and error is why calibration works while
   commands did not.
   With the jog: *reading runs away from the command* = this bug;
   *reading follows, arm goes the other way* = encoder sense (not seen so far).
3. What the model says the same jog does, from the start posture:
   ```matlab
   q = arm_home; j = 2;  dq = zeros(6,1); dq(j) = deg2rad(10) * arm_sign(j);
   ik9_fk([0;0;0;q(:)+dq]) - ik9_fk([0;0;0;q(:)])     % EE displacement, base frame
   ```
   Same direction → the sign is right. Opposite → flip `arm_sign(j)`, and
   set `arm_offset_deg(j)` so the parked posture reads the firmware angle
   the arm actually reports (`ARM,` telemetry line).
4. `gate_all`, then `EnableHardware = 1`.

Known geometry gap, not fixed: the model's DH table has no shoulder offset
(`a1 = 0`); the AR3 has 64.2 mm.

## 4. Simulation first

```matlab
check_9dof      % selected scenario: residual, wheel peak < max_rpm, s, tracking, joint window
gate_all        % all five
check_guard     % the guard and the joint cap
```

## 5. On the robot

1. Place the base in the camera area. `EnableMotive = 1`: the cameras report
   where it is. Cameras off: put it where the card says, or type the measured
   pose into `base_override`.
2. `calibrateROMEArm`. Sends the start posture as the firmware home,
   calibrates straight to it, and tells you what it should look like
   (elbow up-forward, forearm down, tool at the floor ~0.34 m ahead of the
   base axis). If it does not look like that: step 3, `jog_joint`.
3. `EnableHardware = 1`, run. First 10 s nothing moves (hold), then the base
   starts slowly. Slow lap.

## If it misbehaves

**Arm does not move.** Firmware rejected the angle: map or range. Step 3.

**Base does not start.** Mega waits for `'Y'`; the ESP32 translates
`START_GV` into it. Check the ESP32 is on the WiFi sketch and connected.

**Base moves wrongly.** Bench-test wheel order: jack it up, one wheel at a
time. `verify_wheelmap` proves the model is self-consistent; it cannot see the
wiring.

**It will not stop.** The arm does: the Teensy's 500 ms watchdog holds
the joints once commands stop, and `STOP_ALL` holds them explicitly once
the ESP32 is re-flashed with the 22 Sep fix (it used to forward `STOP`,
which the Teensy does not understand). The base does not: `STOP_ALL` → `!`
→ Mega returns *before* writing PWM, so the last wheel command stays
applied. **Cut power.** Known gap.

**Camera mode misbehaves.** `EnableMotive = 1` has never been run. Fall back
to `EnableMotive = 0` with `base_override` set; that path is gated.

## Do not change without re-running `gate_all`

`arm_home`, `z_work`, anything below the line in `define_constants.m`,
`T_HOLD` in `EndEffectorTrajectory.m`, `RAMP_STEPS` in `CommandGuard.m`.
After editing any MATLAB Function block source (`EndEffectorTrajectory.m`,
`ukd_current.m`, `CommandGuard.m`) run `refresh_block_code('ROME_9DOF.slx', true)`
so the model carries the same code.
