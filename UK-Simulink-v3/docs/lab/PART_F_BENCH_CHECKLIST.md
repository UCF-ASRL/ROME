# Part F — Bench checklist (signed)

Prepared 19 Nov 2026 (drafted by Claude for the author). **NOT EXECUTED.** Every blank is empty on purpose.
Spec: `docs/2026-11-18-rome-sim-to-lab-plan.md` Part F steps 23-28. Plan: `docs/superpowers/plans/2026-11-19-plan3-lab-procedure.md` Task 2.
After signing, save a copy under `docs/verification/` (spec step 28).

Rule: a statement about hardware or code carries file and line, or document and printed page. Anything not found is **NOT VERIFIED**, with what would verify it.
Abbreviations: **MEGA** = `UK-Arduino/ROME_Mega_Code/ROME_Mega_Code.ino`, **LIB** = `ROME_Mega_Code/ROME.cpp`/`ROME.h`, **ESP** = `UK-Arduino/ROME_ESP32_Code/ROME_ESP32_Code.ino`, **TNSY** = `UK-Arduino/ROME_Teensy_Code/ROME_Teensy_Code.ino` (lab repo `ucf-asrl/ROME`, read-only, HEAD `9867ca4d709fe5e846e4e8307fce2018b0dabcaf`, 2026-11-18 15:33:06 -0500). Other paths are under `ConstrainedMultibodySubmission/ROME_merged/`. Thesis page numbers are printed ones.

| Session | |
|---|---|
| Date / who | ____ / ____ |
| Witness | ____ |
| MATLAB release (R2026a, full path; TRACKER §0) | ____ |

## SAFETY — read first, tick before every session
- [ ] **E-stop / physical power cut-off location: NOT VERIFIED.** Fill in: ____ . Photograph it (`F_01_estop.jpg`). The thesis says the whole system runs from one lithium-ion battery (p. 8); whether one switch cuts everything (Mega, ESP32, Teensy, stepper drivers, wheel drivers): NOT VERIFIED → item 25c measures it. `GETTING_STARTED.md` (lab repo `UK-Simulink-v3`) L179-184 says "Cut power. Known gap." for the base; that text describes different code from MEGA L143-153 (see item 25).
- [ ] **One hand stays within reach of the cut-off during every motion, and the person at the cut-off is not the person at the keyboard.**
- [ ] **Base jacked up for every wheel test (items 25a, 25d, 26, 27):** all four wheels clear of the floor, on a stand rated for ≥ ____ kg (base is 18 kg without the arm, thesis Table 1 p. 15; with arm: ____ kg measured), stable against the push of a spinning wheel. Nobody touches the wheels while the Mega is started. Stand description: ____.
- [ ] **Arm clear zone.** Fully-extended reach from the J1 axis, measured: ____ m. Clear zone radius = reach + ____ m (author sets), floor marked: ☐. Nobody and nothing inside it while the arm is powered. Measured, not taken from a datasheet (the AR3's reach was not read for this draft).
- [ ] **Arm may fall when power is cut: NOT VERIFIED** (steppers, no brake found in the files read). Support or keep clear on every power cut: ____.
- [ ] **Three behaviours of the firmware that change how you stop it** (all from the code; item 25 tests them):
  1. `CAL_ARM` runs the whole calibration inside one function with `while` loops that never read the serial port (TNSY L160-163, `fullCalibrate` L385-418). **During calibration nothing you send stops the arm; only the power cut does.** Calibration drives every joint into its limit switch (TNSY L384-406): stand clear, hand on the cut-off.
  2. The arm's `STOP_ALL` is **not a latch** (TNSY L181-188): it sets desired = actual and the arm is still READY, so the next joint packet moves it again (L241-257). The wheels do latch (MEGA L143-144: `started = false`, needs `START_GV`).
  3. Joint 4 has no encoder (TNSY L65-70); `actualJointDeg[3]` is set equal to the command every loop (L296), so `STOP_ALL` does not stop a J4 move already commanded (the loop re-issues `moveTo` to the same target, L292-297). **Code reading only: NOT VERIFIED on the arm.** Item 25b observes it.
- [ ] Old firmware signature: with the pre-22-Sep Teensy sketch, commanded moves on J1, J2, J5 ran **away** from the target to the range end while the reading looked fine (`GETTING_STARTED.md` L112-134 in `UK-Simulink-v3`; TRACKER §0 row "22 Sep"). **Do item 23 before item 24.** Whether the arm now on the robot has the fix: NOT VERIFIED (TRACKER Q-F1).

## Order of execution (numbering is the spec's; the order is for safety)
**23 reflash → 25a wheel stop (jacked) → 25d watchdog (jacked) → 26 wheel order (jacked) → calibrate the arm (`calibrateROMEArm`, power cut at hand) → 25b arm stop → 24 joint signs → 25c power cut → 27 logging → 28 sign-off.**
Nothing moves before its stop has been shown to work.

---

## Item 23 — Firmware identity
**Question:** does any board report a version or build string over serial or telemetry?
**Search done (19 Nov, read-only):** read all three `.ino` files and the Mega library files in full, and searched them for `version`, `build`, `__DATE__`, `__TIME__`, `firmware`, `FW_`, `VER` (case-insensitive). Hits: none that is an identifier (the only matches were the words "server" and "reverse"). Result per board:

| Board | What it prints | Version or build string? |
|---|---|---|
| Mega (MEGA) | `GV,act1..4,cmd1..4` at 20 Hz on `Serial1` only (L192-208). No banner, nothing on USB serial (`Serial1.begin`, L67; `Serial` is never used). | **None found** |
| ESP32 (ESP) | USB serial: `AP IP: <ip>` (L50-51), `ROME ESP32 Unified Router Ready.` (L56), `[CMD] …` lines (L127, L135-136, L145). TCP: `#CONNECTED` (L82). | **None found.** The L56 banner is a fixed string, not a version; whether older builds printed a different banner: NOT VERIFIED (history not compared). |
| Teensy (TNSY) | `STATUS` returns the state name (L191-198); `ARM,j1..j6,STATE` at 20 Hz (L362-367); `ARM_READY` (L164); `LIMIT,i` (L402-403); `LIMIT_TEST_READY` (L176); `LIMITS,…` (L343-349). | **None found** |

**Conclusion:** no board reports a version. What is on the robot cannot be read back; the plan's rule applies: **reflash all three boards from the repo commit below.** The Mega's flash includes the two library files `ROME.cpp`/`ROME.h`.

**Reference for the flash.** Hashes from `git -C "<repo>" log -1 --format=%H -- UK-Arduino/<file>` (no checkout) and `git rev-parse HEAD:UK-Arduino/<file>`; the working-tree file hash (`git hash-object`) equals the HEAD blob for all five files, and `git status` of `UK-Arduino/` is clean (19 Nov):

| File | Last-change commit | Blob at HEAD (= working tree) |
|---|---|---|
| `ROME_Mega_Code/ROME_Mega_Code.ino` | `b6b8798cab55d69158f6275e1dcd145d3ce883aa` (2026-09-03) | `026394f3e5167b165ee719570519e2e6a36f1811` |
| `ROME_Mega_Code/ROME.cpp` | `b6b8798cab55d69158f6275e1dcd145d3ce883aa` | `384bbd5430e1472a2a9987a37ace70f3b7e53eee` |
| `ROME_Mega_Code/ROME.h` | `b6b8798cab55d69158f6275e1dcd145d3ce883aa` | `bdaad1936ac23bbd7ce2d5e3dc7558a167ea174a` |
| `ROME_ESP32_Code/ROME_ESP32_Code.ino` | `63a10aa8c422267712600a2c69873c2303fc818d` (2026-09-24 15:37) | `2f30239814e290c304d0f2a93a2c0b65ea168cdf` |
| `ROME_Teensy_Code/ROME_Teensy_Code.ino` | `63a10aa8c422267712600a2c69873c2303fc818d` (2026-09-24 15:37) | `c11c102ebe21378f04af33481ce71d756d6255c9` |

Reflash instruction (for the author): "reflash from repo commit `9867ca4d709fe5e846e4e8307fce2018b0dabcaf` (the files above)". Before each flash run `git -C "<repo>" hash-object UK-Arduino/<file>` and compare with the table (the file must be the committed one, not an edited working copy).
Note on a stale doc: `GETTING_STARTED.md` L182-184 says the base `STOP_ALL` → `!` "returns before writing PWM, so the last wheel command stays applied". MEGA L143-153 does write zeros (`applyWheelRPM(zeros)`, L150-151). Either the doc is stale or the board runs older code. This flash decides it; item 25a tests the result.

**Steps**
1. [ ] Record the toolchain **before** flashing (NOT VERIFIED anywhere in the files read; TRACKER Q-T5a says the lab's installed AccelStepper is not verified):
   Arduino IDE / CLI version ____; Arduino AVR core ____; ESP32 core ____; Teensyduino ____; AccelStepper ____; Encoder ____; boards: Mega model ____, ESP32 model ____, Teensy 4.1 (thesis p. 8-9) ____.
2. [ ] Flash the Mega (MEGA + LIB). Date/time ____, file hashes checked ☐, tool output saved as `F23_flash_mega.txt`.
3. [ ] Flash the ESP32 (ESP). ____ ☐ `F23_flash_esp32.txt`.
4. [ ] Flash the Teensy (TNSY). ____ ☐ `F23_flash_teensy.txt`.
5. [ ] Smoke tests, each from behaviour named in the code:
   - ESP32: the access point (SSID at ESP L6) appears; the MATLAB PC joins it and `tcpclient("192.168.4.1", 3333)` connects (port, ESP L9); the first line received is `#CONNECTED` (L82). Result ____.
   - Mega: with the base jacked up, run `testGV` (it sends `START_GV` then 15 rpm on all four wheels for 5 s, `testGV.m` L11, L24, and prints `[MEGA RX] GV,…` lines, L31-33). Expect `GV,` lines and four spinning wheels; ends with `0.0,0.0,0.0,0.0` (L46). Result ____.
   - Teensy: send `STATUS`; expect `UNCALIBRATED` after a power-up (TNSY L52, L193). Result ____.
6. [ ] Write the identity record: board, date, commit `9867ca4d…`, file blob hashes, who ____.

**Pass:** three boards flashed from files whose hashes equal the table; smoke tests pass; toolchain versions recorded. **Result:** ____ **Initials/date:** ____

---

## Item 25 — Stop test
**What the code does** (read 19 Nov):
- The ESP32 acts on the exact line `STOP_ALL` only (ESP L141-147): it writes `!` to the Mega and `STOP_ALL` to the Teensy. **A bare `!` sent from MATLAB is not forwarded**: it contains no comma, so it reaches "Unknown Command" (ESP L156-165). The Mega's own `!` branch (MEGA L143) is reachable only from the ESP32. **Send `STOP_ALL`.**
- Mega, on `STOP_ALL` or `!` (MEGA L143-153): `started = false`, desired and corrected rpm and integral set to 0, `applyWheelRPM(zeros)`. While `started` is false the main loop writes zeros every 50 ms (L100-107) and speed lines are ignored (L157). **It stays stopped until `START_GV`** (L136-140). `applyWheelRPM(zeros)` ends in `motorOn(0)` = `analogWrite(pwm_pin, 0)` (LIB `ROME.cpp` L13-20) and the direction pin set forward (L22-24). Whether duty 0 means free-wheeling or braking at the driver: **NOT VERIFIED** (driver not identified) → record the stopping time.
- Teensy, on `STOP_ALL` (TNSY L181-188): `steppers[i].move(0)`, `desiredJointDeg[i] = actualJointDeg[i]`. In READY the arm then holds where the encoders say it is. Not latched (see Safety). Does not apply during `CAL_ARM`.
- Mega watchdog: 250 ms without a command line → desired rpm = 0, integral = 0 (MEGA L10, L180-190); it is a target of 0 through the PID, not a hard PWM cut (L100-102, L213-224). Teensy watchdog: 500 ms without a command → desired = actual in READY (TNSY L14, L269-282).
- `ROMECommand` sends `STOP_ALL` when the model's block is released (`ROMECommand.m` L194-199), so a normal end of a run ends with a stop.
- `jogROMEArm`/`returnHomeROMEArm`/`testLimitSwitches` send `STOP_ALL` on exit (L155-163, L115-124, L87-97). **`jog_joint` does not** (it only flushes, L74-76); it relies on the 500 ms hold.

**Helper snippets (NOT RUN, NOT VERIFIED: MATLAB was not run when this was written; the patterns are from `testGV.m` L6-11, L24-33, L46 and `jog_joint.m` L18-27, L33).** Put the real values in the blanks. Keep `c` open so telemetry is visible.

*H1 — wheels, one entry in the command is non-zero:*
```matlab
c = tcpclient("192.168.4.1", 3333, "ConnectTimeout", 5);  pause(0.5);  flush(c);
write(c, "START_GV" + newline, "char");  pause(0.5);
cmd = "15.0,0.0,0.0,0.0" + newline;      % wheel k = the position holding 15.0
t0 = tic;  last = tic;
while toc(t0) < 5                         % must resend faster than the 250 ms watchdog
    if toc(last) > 0.1, write(c, cmd, "char");  last = tic; end
    while c.NumBytesAvailable > 0
        m = strtrim(readline(c));  if startsWith(m, "GV,"), disp(m); end
    end
    pause(0.02);
end
write(c, "STOP_ALL" + newline, "char");   % NOT "!" (see above)
```
*H2 — arm, stop in the middle of a move (the arm must be READY, i.e. calibrated; clear zone empty; hand on the cut-off):*
```matlab
j = ____;  delta = ____;                  % e.g. J1, +25 deg: 25*44.44 = 1111 steps at runSpeed 750 steps/s (TNSY L26, L20) is >= 1.5 s
c = tcpclient("192.168.4.1", 3333, "ConnectTimeout", 10);  pause(0.3);  flush(c);
% read the current ARM, line as jog_joint's read_arm does -> before (1x6, firmware deg)
target = before;  target(j) = target(j) + delta;
cmd = sprintf('ROME,0,0,0,0,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', target);
t0 = tic;  last = tic;
while toc(t0) < 0.5, if toc(last) > 0.1, write(c, uint8(char(cmd)));  last = tic; end, pause(0.02); end
write(c, "STOP_ALL" + newline, "char");   % then stop sending; print every ARM, line for 3 s
```

### 25a — Wheels stop (base jacked up)
1. [ ] Run H1 with 15 rpm on one wheel (or `testGV` for all four), then `STOP_ALL` during the run.
2. [ ] Expected: the next `GV,` line has `cmd1..cmd4 = 0.0` (MEGA L197-207); wheels stop. Time from sending to the wheel standing still (phone video, slow-motion): ____ s. Telemetry `act1..4` after 1 s: ____ .
3. [ ] Send `15.0,…` again **without** `START_GV` for 2 s. Expected: the wheels do not turn (MEGA L157). Observed: ____.
4. [ ] Send `START_GV`, then a command. Expected: they turn again. Observed: ____.
**Pass:** all four wheels stop on `STOP_ALL`; `cmd` columns 0.0 on the next telemetry line; no motion without `START_GV`; actual rpm < 1.0 after 1 s (**[Claude's call — author to confirm]**, no basis in the files; quantisation of the speed estimate is one pulse in 50 ms, MEGA L255-285). **Result:** ____ **Initials:** ____

### 25b — Arm stops (after calibration)
1. [ ] Calibrate (`calibrateROMEArm()`; **power cut at hand, nobody in the clear zone**; it takes up to 120 s, `calibrateROMEArm.m` L11). Arm state READY: ____ (`STATUS` or `ARM,…,READY`).
2. [ ] Run H2 on J1 (a joint with an encoder, large steps per degree). Expected: the joint stops within the next fraction of a second and the readings stay constant. Stop distance: ____ deg; time to stand still: ____ s.
3. [ ] Send **one more** joint packet at 10 Hz for 1 s after the stop. Expected from the code: the joint moves again (not latched). Observed: ____ . (Record it, it is a fact for the procedure: after a stop, nothing may keep streaming.)
4. [ ] J4 (no encoder): repeat H2 with `j = 4`, a delta of ≥ 20° (J4: 46.67 steps/deg, TNSY L26). Expected from the code: the move **continues to the old target** after `STOP_ALL`. Observed: ____ (**NOT VERIFIED until observed**).
5. [ ] Watchdog: stop sending (leave the socket open) in the middle of a move. Expected: holds within 0.5 s (TNSY L14, L269-276). Observed: ____ s.
**Pass:** J1 stops after `STOP_ALL` and holds; behaviours 3 and 4 observed and written down (they are not pass/fail, they are documented); watchdog hold ≤ 0.5 s + one telemetry period (0.05 s, TNSY L338). **Result:** ____ **Initials:** ____

### 25c — Physical power cut-off
1. [ ] Location photographed (`F_01_estop.jpg`) and written in Safety.
2. [ ] With the base jacked up and wheels turning at 15 rpm (H1), the arm READY and holding (clear zone empty, arm supported if it may fall), operate the cut-off. Record what stops: wheels ____; arm steppers ____; Mega ____; ESP32 (does the access point disappear?) ____; Teensy ____; the arm sags or falls: ____.
3. [ ] Restore power. Expected from the code: the Teensy boots `UNCALIBRATED` (TNSY L52) and the arm must be calibrated again (the thesis p. 9 says the encoders are relative). Observed: ____.
**Pass:** everything that moves stops; what the cut removes is written down; the arm's behaviour on power loss is written down. **Result:** ____ **Initials:** ____

### 25d — Comms-loss hold (Plan 3 Review Focus 2)
1. [ ] Base jacked up. Run H1 at 15 rpm on all four wheels; at a marked moment **stop sending but keep reading**. Expected: `cmd1..4` fall to 0.0 within about 0.3 s (250 ms watchdog, MEGA L10, plus one telemetry period of 50 ms, L194); the wheels then slow through the PID (L213-224). Observed: cmd 0.0 after ____ s; wheel standing still after ____ s.
**Pass:** `cmd` columns 0.0 within 0.3 s of the last command. **Result:** ____ **Initials:** ____

---

## Item 24 — Joint sign and direction test
**Tool:** `jog_joint(j, delta_deg)` (`jog_joint.m` L1), `j` = 1…6 as the firmware numbers them, `delta_deg` in **firmware degrees**. It reads the arm's `ARM,` line, sets `target(j) = reading + delta_deg`, caps it to the firmware range minus the 2° margin (L24-26; `arm_fw_margin_deg`, `define_constants.m` L130), sends `ROME,0,0,0,0,<six joint angles>` at 5 Hz for up to 15 s (L27-40) and prints the reading before and after (L41-42). Wheel speeds are zero, and the Mega is not started, so wheels do not move. `jogROMEArm()` (GUI, default step 5°, `jogROMEArm.m` L11, L27-29) is **not** used here: without a telemetry sync it sends all-zero targets (L95-97, L126).
**Preconditions:** item 23 done; arm calibrated (READY: the Teensy accepts joint packets only then, TNSY L241); clear zone empty; hand on the cut-off; `jog_joint` does not send `STOP_ALL` when it ends (L74-76).

**Model-side predictions (one-time, desk or bench PC; NOT RUN, UNTESTED snippet).** Firmware map in the code: `firmware = arm_sign .* model + arm_offset_deg`, `arm_sign = [1 -1 -1 1 1 1]`, `arm_offset_deg = [0 180 90 0 0 0]` (`define_constants.m` L102-103). Candidate physical maps: `romefw_to_arcs_deg(fw, 'rome_dirpin')` and `(…, 'arcs_caldir')` (`analysis/romefw_to_arcs_deg.m`; TRACKER Q-T5a). Per joint, the displacement of the flange for +5° firmware under each map, and under the model:
```matlab
cd ROME_merged;  addpath(genpath('analysis'));  evalc('define_constants');
fw = [____ ____ ____ ____ ____ ____];          % the firmware reading before the jog
for j = 1:6
    d = zeros(6,1);  d(j) = 5;
    for map = ["rome_dirpin","arcs_caldir"]
        [T1,~] = ar3_fk(romefw_to_arcs_deg(fw(:),   char(map)));
        [T2,~] = ar3_fk(romefw_to_arcs_deg(fw(:)+d, char(map)));
        dp = T2(1:3,4) - T1(1:3,4);                          % flange displacement, AR3 frame 0 (m)
        dR = T2(1:3,1:3) * T1(1:3,1:3)';
        w  = [dR(3,2)-dR(2,3); dR(1,3)-dR(3,1); dR(2,1)-dR(1,2)]/2;   % small-angle rotation, frame 0 (rad)
        fprintf('J%d %-12s dflange [%+.4f %+.4f %+.4f] m  rot [%+.3f %+.3f %+.3f]\n', j, map, dp, w);
    end
    dq = zeros(6,1);  dq(j) = deg2rad(5) * arm_sign(j);          % model joint change that +5 fw implies
    pe1 = ik9_fk([0;0;0; arm_home(:)]);  pe2 = ik9_fk([0;0;0; arm_home(:) + dq]);
    fprintf('J%d model (ik9_fk at arm_home)  dflange [%+.4f %+.4f %+.4f] m\n', j, pe2 - pe1);
end
```
Frames: AR3 frame 0 and the model's base frame differ by the arm's mount yaw, which is **unmeasured** (TRACKER §0 row 1 of 19 Nov; `define_constants.m` L98-100 says the arm faces +y). Compare vertical components and the in-plane sense; do not compare x, y directly.

### 24a — Jog table (one row per joint)
Run `jog_joint(j, 5)` for j = 1…6, one at a time. Each prints "J<j>: reading <a> -> commanding <b>" and "reading after … (moved …, asked …)" (L28, L42).

| J | Reading before (fw deg) | Reading after | Moved (printed) | Followed command? (`jog_joint` verdict, L47-56) | Physical direction observed (words + sketch) | Matches `rome_dirpin` / `arcs_caldir` / model prediction |
|---|---|---|---|---|---|---|
| 1 | ____ | ____ | ____ | ____ | ____ | ____ |
| 2 | ____ | ____ | ____ | ____ | ____ | ____ |
| 3 | ____ | ____ | ____ | ____ | ____ | ____ |
| 4 | ____ (no encoder: the reading is the command, TNSY L296; judge by eye, `jog_joint.m` L47-48) | ____ | ____ | n/a | ____ | ____ |
| 5 | ____ | ____ | ____ | ____ | ____ | ____ |
| 6 | ____ | ____ | ____ | ____ | ____ | ____ |

Rules: if "reading did NOT follow the command" (L55), stop: it is the encoder-loop or old-firmware signature; report, do not continue to the next joint. Moved ≠ asked by more than 2° is the same flag (L49). A step-table scale error (TRACKER Q-T5b: J4 1.092, J5 0.457, J6 0.937 vs the vendor) would show as an arm that moves a different angle than reported: measure the physical angle of J5 and J6 with the inclinometer for the +5° step: J5 ____ deg, J6 ____ deg (otherwise this stays NOT VERIFIED).
**Pass:** for each joint the direction is written down and agrees with exactly one of the candidate rows of the prediction output, or the disagreement is written down; no joint flagged "did NOT follow".

### 24b — J2 map question (TRACKER §0 rows of 19 Nov; Q-T5a, Q-B2)
Two candidate maps, `rome_dirpin` (default) and `arcs_caldir`, differ on J2 in sign and offset (`romefw_to_arcs_deg.m` L47-50: ARCS q2 = fw − 132 versus 2.4 − fw). A sign test alone is not enough (TRACKER §0 ruling): record the absolute angle too.
Derived here (not measured), from the J2 row of that table and `analysis/ar3_fk.m` L15-16 (vendor posture q = [0 −90 0 0 0 0] is the upper arm vertical, q2 = 0 is the upper arm horizontal along +x0): the **lean of the upper arm from vertical, positive toward the arm's reach direction (AR3 +x0)** is `lean = q2_ARCS + 90`, so
- `rome_dirpin`: lean = fw_J2 − 42° (a +5° jog **increases** lean by 5°);
- `arcs_caldir`: lean = 92.4° − fw_J2 (a +5° jog **decreases** lean by 5°).
Cross-check with TRACKER Q-B2: at fw J2 = 51.4° these give 9.4° and 41.0° from vertical, the values recorded there.
1. [ ] Mount the inclinometer on the upper arm (J2 to J3 link), zero it on a vertical reference ____.
2. [ ] At the reading before the J2 jog (24a), measure lean (positive toward reach): ____ °. After `jog_joint(2, 5)`: ____ °. Δlean = ____ °.
3. [ ] Repeat at two more J2 readings (`jog_joint(2, ±10)` each; stay inside the 2° margin of 0…132): fw ____ → lean ____ ; fw ____ → lean ____ .
4. [ ] Fill in: predicted lean `rome_dirpin` ____ / `arcs_caldir` ____ at each reading.
**Decision:** the map whose Δlean sign and absolute lean agree at all three readings = ____ . If neither: write the measured lean(fw) line and report (the map is then wrong in offset, not only in sign).
Check that agrees independently: **where is the J2 limit switch?** At fw J2 = 132 (switch side, TNSY L29) the upper arm is horizontal forward (lean 90°) under `rome_dirpin` and 39.6° behind vertical (lean −39.6°) under `arcs_caldir` (same formulas). Photograph (24c) and write where the switch is pressed: ____ .
Same sign question for J5 (maps differ: s = +1 vs −1) and J6 (−1 vs +1): the prediction output gives the rotation of the flange for +5°. Observed J5 ____ / J6 ____ → agrees with ____ .
**Pass:** J2 map decided from measurements and written in the box above, with the three readings. **Result:** ____ **Initials:** ____

### 24c — Limit switches (J2, J5, J6, and the rest)
1. [ ] Identify the switch of each joint: run `testLimitSwitches()` (`testLimitSwitches.m` L1; it sends `TEST_LIMITS`, L46; lamps are numbered Joint 1…6, L17-29; 1 = pressed, TNSY L345). Press each switch by hand, one at a time, and write which lamp lights: switch at the J1 base ____ lamp ____ ; J2 ____ ; J3 ____ ; J4 ____ ; J5 ____ ; J6 ____ . (Teensy pins 26 + i, TNSY L120.)
2. [ ] Photograph **J2, J5 and J6** switches: the switch, its trigger (cam or lever), and the joint, in two views, with the arm in a stated posture (`F24_J2_switch_a.jpg`, `_b.jpg`, `F24_J5_…`, `F24_J6_…`). For each: which direction of the joint's rotation moves the trigger toward the switch (forward / back / up / down, per the lean definition above for J2): J2 ____ ; J5 ____ ; J6 ____ .
3. [ ] During `calibrateROMEArm` (item 25b step 1) watch J2, J5, J6 drive toward their switch: direction ____ . **Trap:** the console prints "Joint <i> hit its limit switch" with the Teensy's **0-based** index (`calibrateROMEArm.m` L90-95, TNSY L402-403): "Joint 1" there is J2.
**Pass:** each switch matched to its joint and photographed, with the end of travel it sits at.

### 24d — Mount and tool measurements (TRACKER §0 row 1 of 19 Nov, Q-T6a, Q-T7a)
The model has the arm base (AR3 frame 0) 0.20 m above the base plate, at the base origin, yaw 0 (`ik9_fk.m` L13, L19-21; TRACKER row says L16; line numbers differ in this copy), and no tool length (end effector = frame-6 origin, Q-T6a). Measure, with a tape or caliper, a level and a plumb line; sketch each:

| What | How | Measured | Model value | Source of the model value |
|---|---|---|---|---|
| Base **deck** top height above the **floor** | tape from floor to the plate top at the plate centre | ____ m | none (no deck height in the sources, TRACKER §0 row 19 Nov "floor-clearance gate") | — |
| Arm mount: height of the AR3 base/J1 mounting plane above the deck | tape | ____ m | 0.20 m (frame 0 above the base plate) | `ik9_fk.m` L19-21 |
| Arm mount height above the floor (sum of the two rows above) | | ____ m | — | — |
| J2 shoulder axis height above the floor | tape to the J2 axis | ____ m | — | cross-check: vendor d1 = 169.77 mm above frame 0 (`analysis/ar3_fk.m` L21 `d_m`) |
| Horizontal offset J1 axis → J2 axis | tape | ____ m | vendor a1 = 64.2 mm (`ar3_fk.m` L20) | check the vendor value |
| **J1 axis position on the plate**, x, y from the **plate centre** (intersection of the diagonals of the 0.45 × 0.45 m plate, thesis Table 1, p. 15), along the base axes x_B, y_B of thesis Fig. 6 (marked on the chassis: item 26 step 1) | tape | x ____ m, y ____ m | 0, 0 | `ik9_fk.m` L19 |
| **Arm forward direction** on the plate (the direction the arm reaches at ARCS J1 = 0, i.e. fw J1 = −10° by the map offset c₁ = 10 in `romefw_to_arcs_deg.m`; that +x0 is the physical reach direction is NOT VERIFIED) as an angle from +x_B, counter-clockwise from above | protractor / plumb | ____ deg | 0 (model); `define_constants.m` L98-100 says the arm was turned 180° and faces +y | `ik9_fk.m` L19 |
| Offset between the plate centre and the Motive pivot | Part C item C9.5 | dx ____ dy ____ | — | — |
| **Length past the flange**: from the flange face (frame-6 origin) to the farthest point of whatever is mounted (mount plus the simulator), along the tool axis | caliper | ____ m | 0 (no tool); author's estimate "a couple cm", ~0.02 m, **not measured** (TRACKER Q-T6a) | `ik9_fk.m` L12 |
| Largest radial extent of the mounted object from the tool axis | caliper | ____ m | — | — |
| What is mounted (name, mass) | read / scale | ____ , ____ kg | — | thesis p. 9-10: a 3D-printed Space Shuttle Orbiter mount |
| Fully-extended reach from the J1 axis (also needed for the safety zone) | tape | ____ m | — | — |

**Pass:** every cell measured and written. For the tool length: ≤ 0.05 m means it is covered by the current `clearance_margin = 0.05 m` (`define_constants.m` L133, TRACKER Q-T7a: it covers ~0.02 m with 0.03 m to spare); > 0.05 m is written as a FINDING for the author, not as a pass. **Result:** ____ **Initials:** ____

---

## Item 26 — Wheel-order test (base jacked up)
**What the code says.**
- Command order: `ROME,w1,w2,w3,w4,j1..j6`; the ESP32 sends the first four values as `"%.3f,%.3f,%.3f,%.3f"` to the Mega (ESP L190-194), which assigns them in order to `desiredRPMs[0..3]` (MEGA L160-170) and to motors 1…4 (`motors[] = {motor1..motor4}`, L15-19). Mega constructor arguments `(6,2,9) (5,3,14) (7,20,15) (8,21,12)` are, per `ROME.cpp` L4-11, `(pwm pin, encoder pin, direction-relay pin)`; the comment on MEGA L13 names them differently (`enablePin, encoderPin, pwmPin`): the comment and the class disagree (the class decides; which wire goes to which motor: NOT VERIFIED, bench test). Forward = relay HIGH, "CCW direction" (`ROME.cpp` L22-24; for the wheel seen from which side: NOT VERIFIED).
- Wheel angles: thesis Table 1 (p. 15): α₁…α₄ = 45°, 135°, 225°, 315°. Thesis **Fig. 6 (p. 16), viewed 19 Nov by rendering the page**: x_B to the right, y_B up; ω₁ at the upper right (+x, +y), ω₂ upper left, ω₃ lower left, ω₄ lower right; α₁ … α₄ in the matching quadrants; yellow arrows on each wheel show the positive ω sense (read them from the figure: ____). This agrees with CLAUDE.md §15: α counter-clockwise from +x_B, ascending, drive direction `[−sin α, cos α]`.
- `define_constants.m` L154 lists `alphas = [315 225 135 45]°` and comments them "from positive y-axis": the comment disagrees with CLAUDE.md §15 (α from +x_B); CLAUDE.md §15 says the two forms are the same map (descending with +sin = ascending with −sin, 5.6e-15). **Not resolved here;** the bench test is the check.
- Telemetry cannot give direction: each Mega encoder is a single pin counting pulses (MEGA L58-61, L82-85) and the reported rpm is `motorDirection × |rpm|` (L283-284), i.e. the commanded sign echoed back. **Telemetry shows which channel has pulses, not which way the wheel turns.** Direction is observed by eye.

**Predicted table** (thesis Table 1 angles with the drive direction of CLAUDE.md §15; the direction the base would be pushed on the floor by wheel k alone, the bottom of the wheel moving the opposite way):

| Wheel k | α_k (thesis) | Corner (Fig. 6) | Base pushed along [−sin α, cos α] (x_B, y_B) | Bottom of the wheel moves along |
|---|---|---|---|---|
| 1 | 45° | upper right (+x_B, +y_B) | (−0.707, +0.707) | (+0.707, −0.707) |
| 2 | 135° | upper left | (−0.707, −0.707) | (+0.707, +0.707) |
| 3 | 225° | lower left | (+0.707, −0.707) | (−0.707, +0.707) |
| 4 | 315° | lower right | (+0.707, +0.707) | (−0.707, −0.707) |

(Single-wheel actuation would also rotate the base in the real world; the table gives only the translation direction of the contact.)

**Steps**
1. [ ] Mark x_B and y_B on the chassis (tape arrows, matching the thesis figure and the Motive body axes of Part C item C9.5): which physical side is +x_B ____; +y_B ____ . Photograph (`F26_axes.jpg`). Without this the test cannot be read.
2. [ ] Base jacked up, wheels free, clear of hands. Tape a mark on each wheel rim; label wheels by corner.
3. [ ] For k = 1…4: run H1 with 15.0 in position k only (others 0.0) for 5 s. Record in the table below: which corner's wheel turned; whether any other wheel turned; the direction the **bottom** of the turning wheel moved (in x_B, y_B terms); the telemetry columns `act1..act4` (which is non-zero), then `STOP_ALL`.
4. [ ] Repeat with −15.0 for k = 1 only: the opposite direction. Result ____.

| k | Corner that turned | Others turned? | Bottom of wheel moved along (x_B, y_B) | Matches prediction? | Telemetry: which `act` is non-zero | Wheel k = telemetry channel k? |
|---|---|---|---|---|---|---|
| 1 | ____ | ____ | ____ | ☐ | ____ | ☐ |
| 2 | ____ | ____ | ____ | ☐ | ____ | ☐ |
| 3 | ____ | ____ | ____ | ☐ | ____ | ☐ |
| 4 | ____ | ____ | ____ | ☐ | ____ | ☐ |

Evaluator `[T, info] = bench_wheel_order(log)` (`ROME_merged/lab/bench_wheel_order.m`, Plan 3 Task 2; tested offline on synthetic logs only, `tests/test_bench.m`) checks only channel index, not direction, for the reason above. Its log (raw `GV,` lines per trial) is not saved by any existing tool; the Plan 3 Task 3 recorder must save it. The joint-direction counterpart is `[T, M] = bench_joint_signs(log)` (item 24; `M.verdict` says which of `rome_dirpin` / `arcs_caldir` the J2/J5/J6 observations support).
**Pass:** for every k, one wheel turns, it is at the corner predicted for wheel k, the bottom moves as predicted for +15 rpm (and oppositely for −15), and telemetry channel k is the non-zero one. Any mismatch is a wiring or sign finding to report (it is not fixed at the bench). **Result:** ____ **Initials:** ____

---

## Item 27 — Logging present
**Names that must appear in a saved run** (Simulink To Workspace names, from `docs/verification/2026-11-19_plan2_interface_map.md` L79; checked there, **not re-checked against the .slx for this draft**): `q_cmd`, `q_dot_cmd`, `wheel_speeds_rpm`, `tau_arm`, `manip`, `resid`, `guard_flag`, `q_meas`, `wheel_sent`, `joint_sent_rad`, `wheel_meas`, `joint_meas_deg`, `arm_ready` (13).
Commanded base and joints: `wheel_sent`, `joint_sent_rad` (`wheel_sent` leaves the guard; `joint_sent_rad` is what enters ROMECommand, after the arm-freeze switch of focus plan F1, and the guard's joint output is `joint_guard_rad`; TRACKER §0 row "24 Sep: Nothing measured" lists five blocks added 24 Sep: which five by name is NOT VERIFIED here; by name they are probably `wheel_sent`, `joint_sent_rad`, `wheel_meas`, `joint_meas_deg`, `arm_ready`). Measured: `wheel_meas`, `joint_meas_deg` (telemetry through `ROMECommand`, `ROMECommand.m` L149-168: the joints are un-mapped to model degrees, L161-163), `arm_ready`. The base pose from the cameras goes into `q_meas` **only when `EnableMotive = 1`**: with 0 it is the simulated sensor (interface map "Model chain" and "Measurement"). Which signal carries the Motive base separately: NOT VERIFIED. Units of `joint_meas_deg`: read them from the file, not assumed (the interface map describes the delayed signal in rad). `wheel_meas` sign is the commanded sign echoed (item 26).
**Steps**
1. [ ] Base jacked up, arm clear (and `arm_freeze` as the author sets for this test; with 1 the model holds the arm at the q0 posture and the banner reads `ARM FROZEN`, focus plan F1; the parked pose is guaranteed equal only once F11 (pre-run check against the calibration record) exists; `ROMECommand.m` L54-57 still resets the workspace `arm_freeze`, which the model no longer reads): run a short hardware run per the procedure (duration ____ s, `EnableHardware = 1`, `EnableMotive = 0`), saved to ____ (`save_run(out, tag)` is planned, Plan 3 Task 3: **not yet written**; until then the saving method is: ____).
2. [ ] Check, in MATLAB (UNTESTED, `out` being the `Simulink.SimulationOutput`):
   ```matlab
   names = {'q_cmd','q_dot_cmd','wheel_speeds_rpm','tau_arm','manip','resid','guard_flag','q_meas', ...
            'wheel_sent','joint_sent_rad','wheel_meas','joint_meas_deg','arm_ready'};
   have = out.who;                                   % variable names in the output
   for k = 1:numel(names), fprintf('%-18s %s\n', names{k}, string(any(strcmp(have, names{k})))); end
   ```
   then for each present name: size ____, time vector present (timeseries/Structure-with-time/array with a time column: ____), length equals the others (the solver step is 0.05 s, `define_constants.m` `dt_9dof`), not all zeros where it should not be (`wheel_meas` non-zero while `wheel_sent` is non-zero; `arm_ready` true).
**Pass:** all 13 names present, non-empty, time-stamped, same length, `wheel_meas` follows `wheel_sent`, `arm_ready` true during the run. **Result:** ____ (list any missing: ____) **Initials:** ____

---

## Item 28 — Sign-off
**This one sign-off opens every robot run, mode 2 included** (author, 19 Nov 2026: "Full Part F"). Every item below, the joint-map test of item 24 too, is PASS and signed before ANY robot run, whatever the run mode (`run_mode` 2, 3 or 4). There is no partial sign-off for mode 2, although mode 2 keeps the arm parked. Mode 4 also needs the Part C sign-off (`PART_C_LAB_VISIT.md`, section 7).

| Item | Result (PASS / FAIL / NOT DONE) | Initials | Date |
|---|---|---|---|
| 23 firmware identity (reflash from commit `9867ca4d…`) | ____ | ____ | ____ |
| 24a jog table | ____ | ____ | ____ |
| 24b J2 map decided: ____ | ____ | ____ | ____ |
| 24c limit switches photographed | ____ | ____ | ____ |
| 24d mount and tool measurements | ____ | ____ | ____ |
| 25a wheels stop | ____ | ____ | ____ |
| 25b arm stops (J1, J4 behaviours recorded) | ____ | ____ | ____ |
| 25c physical power cut | ____ | ____ | ____ |
| 25d comms-loss hold | ____ | ____ | ____ |
| 26 wheel order | ____ | ____ | ____ |
| 27 logging present | ____ | ____ | ____ |

Open findings to carry to TRACKER: ____
Anything marked NOT VERIFIED above that stayed NOT VERIFIED: ____

I confirm the tests above were carried out as written, with the deviations listed.
Name ____ Signature ____ Date (YYYY-MM-DD) ____
The date goes into `lab_signoff.m` (Part F sign-off field, Plan 3 Task 4, not yet written); the signed copy goes to `docs/verification/`.
