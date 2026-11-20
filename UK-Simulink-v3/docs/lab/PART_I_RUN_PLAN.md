# Part I — Hardware run plan (run modes 2, 3, 4)

Prepared 19 Nov 2026 (Plan 3 Task 6, Step 1); run list rewritten 19 Nov 2026 (night, doc pass) for the author's four run modes. **NOT EXECUTED.** Nothing here has been run on the robot; no MATLAB was run to write it.
Spec: `docs/2026-11-18-rome-sim-to-lab-plan.md` Part I (steps 35-36) and the author's answers of 18 and 19 Nov (TRACKER §2): the four modes, "Base-path test (as built)" for mode 2, "Full Part F" before any robot run, and "We need to recalibrate no matter what (I believe, based on the current setup?)".
Rule: a hardware fact carries file:line or a TRACKER row. Anything not found is **NOT VERIFIED**. A pass bound that does not exist yet is written `BLOCKED: <what produces it>`; no number is invented.
Paths are under `ConstrainedMultibodySubmission/`. `define_constants.m` = `ROME_merged/define_constants.m`; other `.m` files without a folder are in `ROME_merged/`. File:line citations of the doc pass were read at HEAD 94ab2bc. TRACKER rows are cited as "TRACKER §0/§2/§3, row <date> <start of text>". Firmware = `UK-Arduino/...` in the lab repo (read-only).
Companion documents (not repeated here): `docs/lab/PART_C_LAB_VISIT.md` (lab frame, Motive), `docs/lab/PART_F_BENCH_CHECKLIST.md` (stop tests, joint signs, logging), `docs/student_procedure/ROME_lab_procedure.tex` Section 8 (the step-by-step for a student) and Section 9 (stopping).

**What changed on 19 Nov (doc pass).** The old run list (run 0: base only with `EnableMotive = 0`; run 1: base only with cameras; run 2: base + arm) is replaced by the author's run modes 2, 3 and 4, in that order. `EnableMotive = 0` is no student mode any more: with the camera off the model runs the fake camera (TRACKER §2, row 19 Nov "Author's four modes"). The old run 1 (arm frozen, cameras on) has no run mode and is dropped. The run settings are not typed: one number, `run_mode`, sets them.

## Read before any run

1. **The run mode sets the run flags.** The user block of `define_constants.m` holds one line, `run_mode = 1;` (`define_constants.m:7`). `EnableHardware`, `EnableMotive`, `motive_source` and `arm_freeze` are derived from it by `run_mode_table.m` (`define_constants.m:177`, `:179-182`; the table is `run_mode_table.m:30-33`) and are never typed: a Run whose flags differ from the run mode is refused (`arm_freeze_banner.m:76-79`). Set the mode with `run_mode = n;` and `define_constants`, or `rome_mode(n)`, or the Mode n button on the model's canvas (`rome_mode.m:1-20`). Every Run prints the mode line (`arm_freeze_banner.m:120`), then `*** ARM FROZEN` (mode 2, `arm_freeze_banner.m:140`) or `*** ARM WILL MOVE` (modes 3 and 4, `arm_freeze_banner.m:143`). The simulation checks (`check_9dof`, `plot_9dof`, `gate_all`, `run_9dof_cases`, `check_guard`) run in mode 1 only (`assert_sim_mode.m:35-42`). A button click is NOT VERIFIED (no test can click; TRACKER §2 F12 row).
   - **The arm freeze (focus plan F1).** In mode 2 the model sends the start posture `q0_9dof(4:9)` (model rad, mapped and clamped by ROMECommand to firmware degrees) for the whole run, and the controller reads the arm angles from its own model (`tools/wire_motive_emulator.m:70`). Shown against a recording fake robot (`tests/test_hw_nocam.m`, `tests/test_arm_freeze.m`); on the robot NOT VERIFIED until the first mode 2 run. That the real arm is parked there is checked only once the pre-run check against the calibration record (F11) is wired.
2. **Mode 4 (cameras on) is refused until the Part C sign-off** records `motive_cal.measured = true` (`define_constants.m:38-42`, `:47`; refusal `motive_cal_check.m:84-86`, called at `define_constants.m:325`). Modes 2 and 3 use the fake camera and need no lab frame (TRACKER §3 Q-F5b-c).
3. **The lab PC runs R2026a** (TRACKER §2, row 19 Nov "Lab PC, R2026a"). Do not use R2025a: `derivation_checks` fails there (TRACKER §0, row 19 Nov "`derivation_checks` fails on R2025a"). MATLAB may exit with an access violation (exit 139, TRACKER §0, rows 18-19 Nov); re-run once before concluding anything. Never edit `define_constants.m`, or press a Mode button, while `gate_all` or `check_guard` runs (CLAUDE.md §20.4); a mode change is refused while the lock exists (`set_run_mode.m:36-39`).
4. **Stop findings that apply to every run** (all from code reading; none observed on the robot, Part F item 25 observes them; carried over unchanged from the first draft, the firmware lines were not re-read in the doc pass):
   - The Teensy `STOP_ALL` does not latch; the next joint packet moves the arm again (`ROME_Teensy_Code.ino:181-188, 241-257`; TRACKER §0, row 19 Nov "Arm does not stay stopped after STOP_ALL"). The wheels do latch until `START_GV` (`ROME_Mega_Code.ino:143-144`).
   - `CAL_ARM` is stoppable only by the power cut (`ROME_Teensy_Code.ino:160-163, 385-418`; TRACKER §0, row 19 Nov "Nothing but the power cut").
   - J4 has no encoder; `STOP_ALL` probably does not stop a J4 move in progress (`ROME_Teensy_Code.ino:296`; TRACKER §0, row 19 Nov "STOP_ALL probably does not stop a J4").
   - A bare `!` from MATLAB stops nothing; only the exact line `STOP_ALL` is forwarded (`ROME_ESP32_Code.ino:141-146, 156-165`; TRACKER §0, row 19 Nov "A bare").
   - Wheel telemetry cannot show direction (`ROME_Mega_Code.ino:58-61, 283-284`; TRACKER §0, row 19 Nov "Wheel telemetry"). Direction is judged by eye (Part F item 26).
   - Comms loss: wheels to zero after 250 ms, arm holds after 500 ms (`ROME_Mega_Code.ino:10`, `ROME_Teensy_Code.ino:14`; student procedure Section 9). Observed: NOT VERIFIED until Part F 25d.

## Common safety block (all runs)

| Item | Rule |
|---|---|
| Who holds the power cut | A second person, hand on it for the whole motion, **not** the person at the keyboard (Part F SAFETY, second tick). Location of the e-stop / power cut: **NOT VERIFIED** (Part F SAFETY, first tick; item 25c measures whether one switch cuts everything). Name for each run: ____ |
| Who is at the keyboard | ____ (cannot be the person at the power cut) |
| Clear zone, base | Modes 2 and 3: an area of at least the size `define_constants` prints ("Required Workspace"), around the start pose placed by hand (student procedure Section 7). Mode 4: a circle around the taped origin of at least the radius printed by `check_9dof` in the line "base travels to ... m from the origin -- clear that space" (`check_9dof.m:251`). Nobody and nothing inside (student procedure Section 8). Floor marks (mode 4) come from Part C C11. |
| Clear zone, arm | Radius = measured fully-extended reach from the J1 axis + margin set by the author (Part F SAFETY, "Arm clear zone"; both blank: ____ m). Applies to every run while the arm is powered, mode 2 included. |
| Arm may fall when power is cut | **NOT VERIFIED** (Part F SAFETY). Support or keep clear on every power cut. |
| Stop sequence | 1. Press **Stop** in Simulink first (`ROMECommand.m:194-196`, `releaseImpl` sends `STOP_ALL`). 2. If anything still moves, send the exact line `STOP_ALL` (not `!`). 3. Cut power. Because the arm's `STOP_ALL` is not a latch, a model that is still running re-sends joint packets, so step 1 must come first. During calibration: power cut only. |
| Before every run | Fast Restart off: refused in every mode (`apply_pacing.m:67-70`). After the run: `run_mode` back to 1 (student procedure Section 8, last step of each mode). |

## What every run records

- **Logged signals** (14 To Workspace names since focus plan F1 added `joint_guard_rad`, the CommandGuard joint output; `joint_sent_rad` is now taken after the arm-freeze switch, i.e. what enters ROMECommand; Part F item 27, taken from `docs/verification/2026-11-19_plan2_interface_map.md` L79; **not re-checked against the .slx for this plan**): `q_cmd`, `q_dot_cmd`, `wheel_speeds_rpm`, `tau_arm`, `manip`, `resid`, `guard_flag`, `q_meas`, `wheel_sent`, `joint_sent_rad`, `wheel_meas`, `joint_meas_deg`, `arm_ready`. Part F item 27 must have PASSED first (all 13 present, same length, time-stamped).
- **Run file:** saved with `save_run(out, tag)`, **BLOCKED: Plan 3 Task 3, not yet wired** (Part F item 27 step 1). Until it exists, the saving method is the author's: ____. The file must carry the `define_constants.m` SHA-256, the run mode and the values in the tables below (Part C header uses the same hash practice).
- **Also record by hand:** the run mode, the start pose and how it was placed (Part C C0.1 cards for mode 4), photos of the floor, wall-clock time of start and stop, who held the power cut, anything seen (tool toward the base or floor, wheels spinning the wrong way, the arm moving in mode 2).
- **Not measurable from the logs:** wheel direction (TRACKER §0 "Wheel telemetry"), the J4 position (no encoder), any force or torque at the wheels; in modes 2 and 3 the base pose (the fake camera reports the model's own prediction, `run_mode_table.m:7`).

## Every robot run: preconditions and order

**Preconditions (all modes)**
- **The whole Part F bench checklist signed, the joint-map test (item 24) included, before ANY robot run, mode 2 included** (author, 19 Nov: "Full Part F"; `docs/lab/PART_F_BENCH_CHECKLIST.md` item 28). In particular 23 reflash, 24a-d joint signs, J2 map, limit switches, mount; 25a wheels stop, 25b arm stops, 25c power cut, 25d comms-loss hold; 26 wheel order; 27 logging. Sign-off date in `lab_signoff.m`: **BLOCKED: Plan 3 Task 4, not yet written**.
- Simulation of exactly these settings in mode 1: `check_9dof` prints `RESULT: PASS` (`check_9dof.m:256`). The pre-run check (Plan 3 Task 4) summarises the same: **BLOCKED: Plan 3 Task 4**.
- `speed_factor` chosen from the Plan 2 max_rpm / onset study: **BLOCKED: Plan 2 study (TRACKER P4, Plan 2 Task 8), NOT YET run.** Today's default is `speed_factor = 1.0` (`define_constants.m:62`). At 1.0 the ellipse wheel peak is 42.9 rpm in simulation (TRACKER §0, row 24 Sep "Wheels cut") against the shipped `max_rpm = 80` (`define_constants.m:78`); at 0.25 it was 14.6 rpm (comment at `define_constants.m:63`). These are simulation numbers; what the real wheels do at them is unmeasured.
- Tests that must have passed on the working copy used for the run: `test_suite` (it exits 139 intermittently on this PC, TRACKER §0 rows 19 Nov, so the reference run is on the lab PC, TRACKER §2 row "Lab PC, R2026a"). That reference run is **BLOCKED: Plan 3 Task 7**, after all tests here are done. The F12 fix-round-1 suite crashed at step 9 (TRACKER §2 F12 row): NOT VERIFIED on 94ab2bc.

**Order on the day (every mode)** — the start card prints it (`define_constants.m:430`):
1. Set the run mode first: `run_mode = n;` saved and `define_constants`, or `rome_mode(n)`, or the Mode n button. The mode line must name mode n.
2. Place the robot (modes 2 and 3 by hand at the card's start pose, no taped marks; mode 4 on the taped marks).
3. Calibrate the arm (`calibrateROMEArm`, power cut in hand). **Re-calibrate before every robot run**, also when nothing changed (author, 19 Nov: "We need to recalibrate no matter what (I believe, based on the current setup?)"; that it is physically needed between runs is NOT VERIFIED). Change nothing in `define_constants.m` after this step.
4. Open the model, Run: the mode line, then the arm line of the mode.
5. Save the run, then set `run_mode` back to 1.

**User-block values** (`define_constants.m`; only `run_mode` differs between the modes)

| Variable | Value | Line | Note |
|---|---|---|---|
| `run_mode` | 2, 3 or 4 (see each run) | :7 | Derives EnableHardware 1, EnableMotive 1, `motive_source` and `arm_freeze` (`run_mode_table.m:31-33`). Back to 1 after the run. |
| `scenario` | 1 (case 1, ellipse) | :5 | The student procedure covers case 1 in the robot modes. Another case is the author's choice: ____. |
| `speed_factor` | **BLOCKED** (Plan 2 study); default 1.0 | :62 | Not above the value of the previous mode's passed run, unless the author says. |
| `size_factor` | 0.50 (shipped) | :66 | Not changed by this plan. |
| `max_rpm` | 80 (author 19 Nov, tunable; firmware cap 120, `ROME_Mega_Code.ino:8`) | :78 | |
| `base_override` | `[NaN; NaN; NaN]` (the default; the start is computed) | :31 | Ignored in mode 4 (the cameras win, comment at :30). In modes 2 and 3 the robot must be placed where the model assumes. |
| `tool_tilt_deg` | 0 (shipped) | :234 | Unchanged. The 15 deg value in the lab repo copy is not used (CLAUDE.md §33). |
| `sensor_model` | forced to 0 on the robot by the file | :304-306 | Do not set by hand. |
| `T_stop` | computed, `T_scaled + T_ON/2` | :294 | Do not edit. |
| `arm_sign`, `arm_offset_deg`, `arm_fw_margin_deg` | values decided by Part F 24b, not by this plan | :128-129, :161 | Derived from the AR3 files, not measured (comment at :116-121). Do not change on the day. **NOT VERIFIED** until 24b. |

## Run A — mode 2: base-path test (ground vehicle moving, frozen arm, camera off)

**Purpose.** "Base-path test (as built)" (author, 19 Nov). The ground vehicle follows the base path of the full 9-DOF plan of base and arm; the arm stays parked at the start posture and the **end effector is not tracked**. The controller plans with its model of the arm (`tools/wire_motive_emulator.m:70`), so the wheel commands include the arm's planned motion. The base pose comes from the fake camera (the model's own prediction, `run_mode_table.m:7`), so a placement error or a drift of the base is neither seen nor corrected. First student robot run (TRACKER §2, row 19 Nov "First student robot run").

**Preconditions:** every-run preconditions above. No Part C. Tested before the robot against the recording fake robot (`tests/test_hw_nocam.m`; test_motive_emulator h3 is mode 2, TRACKER §2 row 19 Nov "Mode 2 (ground vehicle, frozen arm)").

**Settings:** `run_mode = 2;` → `MODE 2: ground vehicle moving, frozen arm, camera off (fake camera)` (`run_mode_table.m:31`). Run banner `*** ARM FROZEN` (`arm_freeze_banner.m:140`); `*** ARM WILL MOVE` means stop at once.

**Safety block:** common block. The arm is powered and parked: keep the arm clear zone. If the arm moves, stop.

**Record:** common list. Plus the simulated twin of the same settings (mode 1, same `define_constants.m` apart from `run_mode`), saved before the robot run.

**Pass condition**
- Safety: no stop was needed; the arm did not move; `guard_flag` never 1 (a scaled command means the robot is not where the model believes, CLAUDE.md §20.3).
- Base path and wheel response against the twin: `compare_hw_sim` bounds on the base path (from the logs only `wheel_meas` vs `wheel_sent` is measured; the base pose is the model's own in mode 2), lag. **BLOCKED: `compare_hw_sim` (Plan 3 Task 3, NOT YET wired) and the Plan 2 tolerance report (NOT YET produced).** The end effector is not part of the mode 2 pass condition.
- Values that already exist: the wheel peak of the simulation, 42.9 rpm on the ellipse at speed 1.0 (TRACKER §0 row 18 Nov "`check_9dof` ... 39.5 rpm", fixed 19 Nov). Simulation values, not hardware pass bounds. Open-loop shortfall of the paced start-up: TRACKER §3 Q-P2T1c-b.

## Run B — mode 3: ground vehicle moving, moving arm, camera off

**Preconditions**
- Every-run preconditions above (Part F includes the J2 map decision of 24b; Q-T5a, Q-B2). Reason it matters here: the model-to-firmware map for J2 puts the real upper arm where the model does not believe it is (TRACKER §0, row 19 Nov "The model-to-firmware map for J2"), and the arm mount on the base is unmeasured (TRACKER §0, row 19 Nov "The arm's mount").
- Run A (mode 2) of the same settings passed: **BLOCKED** (its pass condition is BLOCKED).
- The floor-clearance gate is a floor-excursion tripwire only, on the placeholder arm geometry (TRACKER §0, row 19 Nov "The new floor-clearance gate"); the `'ar3'` geometry gate is Plan 2 Task 6, **BLOCKED / NOT YET**.
- The "tool would try to surpass the vehicle / hit the ground" observation of 24 Sep is not explained (TRACKER §0, row 24 Sep "Robot moved 'but not ...'"). Mode 3 does not start before the commanding check (Plan 1 Task 6, `docs/verification/2026-11-19_commanding_check.md`) has been read against the measured joint map: **NOT VERIFIED that this closes it.**

**Settings:** `run_mode = 3;` → `MODE 3: ground vehicle moving, moving arm, camera off (fake camera)` (`run_mode_table.m:32`). The controller reads the real arm telemetry (`tools/wire_motive_emulator.m:70`). Run banner `*** ARM WILL MOVE` (`arm_freeze_banner.m:143`).

**Safety block:** common block, with these additions.
- The arm clear zone applies with the arm powered (Part F SAFETY).
- Calibration (`calibrateROMEArm`) is stoppable only by the power cut: stand clear, hand on the cut-off (`ROME_Teensy_Code.ino:160-163`).
- Stop sequence: Simulink Stop first, then `STOP_ALL`, then power. Because the arm does not latch, never `STOP_ALL` with the model still running. J4 may continue to its target after `STOP_ALL` (no encoder, TRACKER §0 row 19 Nov "STOP_ALL probably does not stop a J4"); keep clear of the J4 sweep and use the power cut for J4.
- The arm may fall when power is cut (NOT VERIFIED): support or keep clear.
- Watch for the tool heading toward the base or the floor (student procedure Section 8): stop at once.

**Record:** common list, plus photos of the arm posture at start, mid-lap and end, and the joint readings `joint_meas_deg` against `joint_sent_rad` through the map. J4 reading is the command, not a measurement (`ROME_Teensy_Code.ino:296`).

**Pass condition:** `compare_hw_sim` bounds for the joint tracking and the wheel response against the twin, plus the clearance of the real arm against the `'ar3'` geometry. **BLOCKED: `compare_hw_sim` (Plan 3 Task 3, NOT YET wired), the Plan 2 tolerance report (NOT YET produced) and the `'ar3'` clearance gate (Plan 2 Task 6, NOT YET).** No end-effector measurement exists in mode 3: the base pose is the model's own (fake camera) and no tool rigid body is used (TRACKER §2, row 18 Nov "Cameras measure the BASE ONLY").

## Run C — mode 4: ground vehicle moving, moving arm, camera on

**Preconditions**
- Every-run preconditions above, and Run B (mode 3) of the same settings passed: **BLOCKED** (its pass condition is BLOCKED).
- **Part C sign-off** (lab frame measured: up-axis, units, ground plane, rigid body, floor frame C10-C11, push test C12) recorded in `lab_signoff.m`, and `motive_cal` entered with `measured = true` (`define_constants.m:43-47`); until then mode 4 is refused (`motive_cal_check.m:84-86`), and `rome_mode(4)` sets the previous mode back (`rome_mode.m:36-47`). **BLOCKED: Part C visit (lab); `lab_signoff.m`, Plan 3 Task 4, NOT YET written.** `fit_motive_cal.m` and `record_motive.m` exist (`ROME_merged/lab/`); the fit's gate values are provisional (TRACKER §2 row 19 Nov, Q-P3T1a).
- **Q-P2T1a decided by the author.** With no valid frame at the first step the model is "safe but dead" (TRACKER §3 Q-P2T1a). A Motive dropout moves the base estimate by metres (TRACKER §0, row 19 Nov "A Motive dropout"); the wait for a valid frame and the stop on a dropout (Plan 2 Task 1b) are **NOT built yet** (student procedure Section 8, mode 4).
- Occlusion with the arm up checked (Part C C9.7).
- The words "(OptiTrack)" of the mode 4 line are Claude's, NOT VERIFIED with the author (`run_mode_table.m:23-25`).

**Settings:** `run_mode = 4;` → `MODE 4: ground vehicle moving, moving arm, camera on (OptiTrack)` (`run_mode_table.m:33`). The cameras give the base pose, converted to the floor frame (`define_constants.m:33-37`); the controller reads the real arm telemetry. Run banner `*** ARM WILL MOVE` (`arm_freeze_banner.m:143`).

**Safety block:** as mode 3. Also: if the cameras lose the robot, stop (the behaviour on a dropout is not yet specified, see Q-P2T1a). A person standing in the camera volume can occlude markers (Part C C9.7); keep the volume clear apart from the person holding the power cut, who stands outside the clear zone.

**Record:** as mode 3, plus the raw Motive stream or a recording of it for the run (`record_motive`, `ROME_merged/lab/record_motive.m`, needs `opts.live = true`, TRACKER §2 row 19 Nov "`record_motive` needs"), and the frame sign-off date.

**Pass condition:** `compare_hw_sim` bounds for the closed-loop base pose error (measured vs reference) and the comparison against mode 3 and against the twin; the end-effector pose computed from the measured base pose and `joint_meas_deg`. **BLOCKED: `compare_hw_sim` (Plan 3 Task 3, NOT YET wired) and the Plan 2 tolerance report (NOT YET produced).** Camera-noise values already in the code (`mocap_sigma`, `enc_step_rad`) are a model of the sensor, not a pass bound.

## Open decisions (author)

1. Scenario and `speed_factor` for the first mode 2 run (after the Plan 2 study).
2. Q-P2T1a (behaviour when no frame arrives), before mode 4.
3. Names and margins in the safety tables (power cut location, clear-zone margin).

Closed: how the arm is kept still for the base-only run (F1; it is run mode 2); with or without cameras for the arm run (the four modes: mode 3 without, mode 4 with); the bench sign-off for mode 2 ("Full Part F", TRACKER §3 Q-P4-a). TRACKER §3 Q-P3T6a, which asked these together, is not closed by this document: the author has not answered it as such.
