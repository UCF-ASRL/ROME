# ROME lab procedure: student walk-through (19 Nov 2026)

**Sources used:** only the repo `ucf-asrl/ROME`, branch `sim-to-lab` (HEAD `72dbcab`), and `UK-Simulink-v3/docs/student_procedure/ROME_lab_procedure.pdf`. The PDF has 22 pages. I extracted its text with PyMuPDF 1.28.2 and rendered pages 1, 6, 7 and 12 to look at them. I did not read the `.tex`, and I did not open anything under `...\UCF\Research`.

**The PDF in numbers:**
- 96 numbered steps 1.1 to 10.4 (per section: 6, 8, 5, 5, 10, 10, 9, 32, 7, 4), plus 46 troubleshooting entries T1-T46, and A1-A4 and G1-G2 in the appendix. My first regex count gave 97 because it also matched the printed line `2.10 x 2.10 meters`; I corrected that by hand.
- 53 `[BLOCKED ...]` notes and 6 `[NOT VERIFIED ...]` notes.
- About 786 words of red text. I counted spans with red colour using PyMuPDF.
- The PDF calls itself "DRAFT ... Not for use on the robot" (p.1). Section 8 opens with "Do not do this section yet" (p.11).

Step-type key: **C** = command typed, **E** = file edited, **B** = button or GUI click, **J** = judgement or reading output, **P** = physical action in the lab.

---

## 1. Student path per mode (counts)

**How I counted:**
- Each numbered step a student carries out counts once.
- A step that only points elsewhere ("Do the steps of ... above", "Place the robot (Section 7) and calibrate (Section 6)") is counted through the steps it points to. The pointer itself counts as 0. These pointers are listed separately, because each one makes the reader jump to another page.
- Section 9 (Stopping) and step 6.10 are conditional and are not counted.

### Mode 1 (simulation)

| # | Step (p.) | Type | Note |
|---|---|---|---|
| 1 | 5.1 Start MATLAB, type `version` (p.6) | C | |
| 2 | 5.2 `cd` to the model folder (p.6) | C | **BLOCKED**: path on the lab PC is not given |
| 3 | 5.3 `edit define_constants`, set `scenario` (p.6) | C+E | Same action as 4.1/4.2 |
| 4 | 5.4 Set `run_mode = 1` (edit), or `rome_mode(1)`, or the Mode 1 button (p.6) | E or C or B, plus J | Three ways to do one thing |
| 5 | 5.5 Save, type `define_constants`, read the mode line (p.6) | B+C+J | Same action as 4.4 |
| 6 | 5.6 Check Fast Restart is off in the toolstrip (p.6) | B/J | The model is never opened in Section 5, so there is no toolstrip to look at |
| 7 | 5.7 `check_9dof`, then read about 12 expected lines (p.6-8) | C+J | |
| 8 | 5.8 Compare with Table 3 (p.8) | J | **BLOCKED**: Table 3 is empty |
| 9 | 5.9 `plot_9dof`, then read 13 figures (p.8) | C+J | Runs the simulation a second time |
| 10 | 5.10 Compare the path with Figure 2 (p.8) | J | **BLOCKED**: Figure 2 is empty |
| 11 | Save a figure with File > Save As (Section 10, p.15) | B | |

- **(a) Student steps: 11.** Of these, 3 are BLOCKED or empty (5.2, 5.8, 5.10) and 1 cannot be done as written (5.6). The real minimum is 7 actions: 5.1, 5.2, 5.3, 5.5, 5.7, 5.9 and save.
  - Page 1 says "Do the sections in order". Read literally, a mode-1 student would also do Section 1 (6 safety steps), Section 3 (5 floor-tape steps that are irrelevant to mode 1) and Section 4 (5 steps that duplicate 5.3/5.5). That gives **27** steps.
- **(b) TA or one-time: 3.** 2.1 install R2026a, 2.2 copy the folder (BLOCKED: branch), 2.3 test suite (BLOCKED on the lab PC).
- **(c) Repeated every run: 6.** 5.3 (only if the case changes), 5.5, 5.7, 5.9, save, plus the 5.6 look.

### Mode 2 (base moves, arm frozen, fake camera)

| Block | Steps | Count | Types |
|---|---|---|---|
| Safety, once per session | 1.1-1.6 | 6 | J, P (1.2, 1.4, 1.5 BLOCKED) |
| Mode-1 simulation first (8.2 → Section 5) | 5.1-5.7 | 7 | C, E, J |
| "Before every robot run" | 8.1, 8.3, 8.4 | 3 | E (8.1 BLOCKED: value), J+P, B/J |
| Mode 2 proper | 8.5, 8.7, 8.9, 8.10, 8.11, 8.12, 8.13, 8.15 | 8 | J (TA), E/C/B+J, C/B, P, B+J, J, J, E/C/B+J |
| Placement (8.8 → Section 7) | 7.1-7.4 | 4 | C, P (7.2 BLOCKED), P, P |
| Calibration (8.8 → Section 6) | 6.2-6.9 | 8 | P/J, B (6.3 BLOCKED: network), C, J, J, J, J (6.8 BLOCKED), file kept |
| Save (8.14 → Section 10) | 10.1-10.4 | 4 | C (10.1 BLOCKED: `save_run` not wired), C (10.2 BLOCKED: `compare_hw_sim` missing), file kept, notebook |

- **(a) Student steps: 40.** On top of that the student follows 4 pointer jumps: 8.6 → 8.1-8.4, 8.8 → Sections 7 and 6, 6.1 → Section 7, 8.14 → Section 10. **12 of the 40 are BLOCKED.**
- **(b) TA or one-time: 6 numbered steps** (2.1, 2.2, 2.3, 2.5, 2.6, 2.7). Step 2.5 is the whole Part F bench checklist: `docs/lab/PART_F_BENCH_CHECKLIST.md` has 39 check boxes and 215 blanks to fill in. Step 2.6 is BLOCKED because `lab_signoff.m` does not exist.
- **(c) Repeated every robot run: 28.** These are 5.5, 5.7, 8.1, 8.3, 8.4, 8.7, 7.1-7.4, 6.2-6.9, 8.9-8.13, 10.1-10.4 and 8.15. Each robot run includes a mode-1 simulation, two mode switches, and a full arm calibration (≤120 s, every joint driven into its limit switch). p.12 says to calibrate "also when nothing changed since the last one".

### Mode 3 (base and arm move, fake camera)

- **(a) Student steps: 40.** The structure is the same as mode 2: 8.16, 8.18, 8.20 (3 actions: open model, tell partner, Run), 8.21, 8.22 and the mode-back in 8.23, plus 7.1-7.4, 6.2-6.9, 10.1-10.4, 5.1-5.7, 8.1/8.3/8.4 and 1.1-1.6.
  - 8.16 also requires "a mode 2 run of the same settings has passed". In practice a first mode-3 session is the mode-2 path plus the mode-3 path, about **68 actions**.
  - Whether a mode-2 run "passed" cannot be decided: the pass condition is BLOCKED because `compare_hw_sim` is not built.
  - **12 of the 40 are BLOCKED:** 1.2, 1.4, 1.5, 5.2, 8.1, 8.16, 8.18 (pre-run check), 7.2, 6.3, 6.8, 10.1 and 10.2.
  - Types for the steps that differ from mode 2: 8.16 J (ask the TA, judge whether mode 2 passed), 8.18 E/C/B+J (set mode 3, read the banner), 8.20 C/B+P+B+J (open the model, tell the partner, Run, read `*** ARM WILL MOVE`), 8.21 J (watch, stop if needed), 8.22 J, 8.23 E/C/B+J (mode back to 1). The rest are typed as in mode 2.
- **(b) TA or one-time:** as mode 2, plus the J2/J5/J6 maps of Part F item 24 (BLOCKED).
- **(c) Repeated every run: 28.**

### Mode 4 (base and arm move, OptiTrack cameras)

- **(a) Student steps: 46.**
  - 1.1-1.6 (6), 5.1-5.7 (7), 8.1/8.3/8.4 (3).
  - **Section 3 floor marks 3.1-3.5 (5; 3.1-3.4 are BLOCKED on photographs).**
  - 8.24, 8.25, 8.27, 8.29 (3 actions), 8.31, and the mode-back in 8.32 (8).
  - Taped-mark placement 7.5-7.9 (5). It includes a tape-and-protractor measurement against a tolerance that is BLOCKED.
  - 6.2-6.9 (8) and 10.1-10.4 (4).
  - It also requires a passed mode-3 run.
  - **17 of the 46 are BLOCKED:** 1.2, 1.4, 1.5, 5.2, 8.1, 3.1-3.4, 8.24, 8.29, 7.7, 7.8, 6.3, 6.8, 10.1 and 10.2.
  - Types for the steps that differ from mode 2: 3.1-3.4 J+P (find the tape), 3.5 J, 8.24 J (ask the TA), 8.25 P (keep the camera area clear), 8.27 E/C/B+J (set mode 4, read the banner or the refusal), 7.5 C, 7.6 J (compare the card with the mark label), 7.7 P (push the base onto the mark), 7.8 P+J (tape and protractor against a tolerance), 7.9 P, 8.29 C/B+P+B+J (open the model, tell the partner, Run, read the banner), 8.30 J (conditional), 8.31 J, 8.32 E/C/B+J. The rest are typed as in mode 2.
- **(b) TA or one-time:** as mode 3, plus 2.4 (Part C lab visit: 10 check boxes, 160 blanks, camera calibration), 2.7 (one PC on two networks: BLOCKED) and 2.8 (taping). After Part C, someone must also type the measured `motive_cal` into the student's "EDIT THIS BLOCK" and set `measured = true`. This is what the refusal text in `motive_cal_check.m:86-90` tells the user to do.
- **(c) Repeated every run: about 31.**

---

## 2. Mode 1, done for real (-batch, one attempt)

**Before the run:**
- `tasklist /FI "IMAGENAME eq MATLAB.exe"` returned "No tasks are running" on both checks.
- Baseline `git status --short --untracked-files=all` had 63 lines. Nine tracked Simulink cache files were already modified, and `runs/` was already untracked. I hashed and backed up those 9 files.
- SHA-1 of `define_constants.m` was `da53dbb9...` and of `ROME_9DOF.slx` was `5b4bc5fa...`.

**Command (one attempt):**
```
"C:\Program Files\MATLAB\R2026a\bin\matlab.exe" -batch "addpath('<scratch>\student_walk'); cd('...\ROME\UK-Simulink-v3'); student_mode1_walk"
```
The script runs the procedure's steps in order. These are the -batch equivalents I used:

| Step | What the procedure says | What I ran |
|---|---|---|
| 5.1 | type `version` | `version` |
| 5.2 | `cd`, then `pwd` | `cd`, then `pwd` |
| 5.3 / 5.4 | edit the file | I could not open the editor. I printed lines 5 and 7 and kept `scenario = 1`, `run_mode = 1`. |
| 5.5 | `define_constants` | `define_constants` |
| 5.6 | look at the Simulink toolstrip | **My own substitute; the procedure gives no command:** `load_system` + `get_param(...,'FastRestart')` + `close_system` |
| 5.7 | `check_9dof` | `check_9dof` |
| 5.9 | `plot_9dof` | `plot_9dof` |
| Section 10 | File > Save As | `exportgraphics` of every figure into my scratch folder, not the repo |

**Exit code: 0.** Wall time was 71 s including MATLAB start-up. `check_9dof` took 9.2 s and `plot_9dof` took 12.0 s. This was my PC, not the lab PC: the procedure itself marks lab-PC timing as NOT VERIFIED.

### Did I get what the procedure says?

Yes. **RESULT: PASS.** I checked each expected item from 5.5, 5.7 and 5.9 against the log:

| Expected (procedure) | Seen in log? |
|---|---|
| 5.1 version contains R2026a | yes: `26.1.0.3346908 (R2026a) Update 5` |
| 5.5 `MODE 1: Simulation only (end-effector dynamics sim, pose data from sim)` | yes |
| 5.7 orbit parameters and a card headed `PLACE THE ROBOT LIKE THIS` | yes |
| `9-DOF VERIFICATION - simulation only, no hardware` | yes |
| two `static: [OK]` lines | yes, 2 |
| six result lines starting `[OK]` | yes, 6 (constraint, wheel, singularity, EE bounded, firmware range, floor) |
| mode line then `*** ARM FROZEN` when the model starts | yes |
| warning `The orbit exceeds a 2m x 2m area!` | yes (printed 5 times in one session) |
| `RESULT: PASS` | yes |
| 5.9 figure `9-DOF actuators and solve health - <case>` | yes: `... - orbit (elliptical)` |
| 5.9 "... figures from plotting.m on end-effector quantities," | yes |
| 5.8 Table 3 values | **cannot check**: Table 3 is BLOCKED (empty) |
| 5.10 path vs Figure 2 | **cannot check against Figure 2** (BLOCKED). By eye, the tracked end-effector ellipse lies on the desired one in "Desired vs. Tracked Position" (fig1). |
| 5.6 Fast Restart off | `FastRestart = off` (my substitute check) |

**Things in the output that the procedure does not prepare a student for:**
- **Unexplained warning.** `Warning: The System object name 'MotiveData' cannot be found` appeared when the model was loaded before `check_9dof` (my 5.6 substitute). `check_9dof` adds `Optitrack/` to the path itself, so it did not warn there. A student who opens the model by hand first, as 5.6 implies, would see this warning. It is not in Section 11.
- **Heading plots that look like failure.** The "Desired vs. Tracked Heading" and "θ Position Error" figures show a heading error that sweeps from about +180° to -180° over the run, with a desired heading of 0 throughout. A student would read this as failure.
  - The procedure says nothing about it. `check_9dof` does not gate the end-effector yaw.
  - **NOT VERIFIED** whether this is a real error or a convention mismatch between `2*atan2(u4,u1)` and `atan2(R21,R11)` in `plot_9dof.m:61,64`.
- **Wrong wheel limit on the plot.** The wheel plot draws the limit line at 120 rpm (`MAX_RPM`). The gate actually uses `max_rpm = 80`.
- **Repeated console output.** The start card (about 20 lines) prints 5 times in one mode-1 session: `define_constants`, the model's PostLoadFcn, and both again inside `check_9dof` and `plot_9dof`. Each copy tells the student to "run the jog test, GETTING_STARTED.md step 3, before the first robot run".
- **Two different floor-area numbers.** `Required Workspace: 2.10 x 2.10 meters` (used by 7.1/7.2) does not match `base travels to 1.27 m from the origin -- clear that space` (used by 8.3). A 1.27 m radius needs about 2.54 m of floor across.

### Repo left as found

- After the run, `git status --short --untracked-files=all` was identical to the baseline (`diff` exit 0).
- One tracked, already-modified cache file had a new hash: `UK-Simulink-v3/slprj/_sfprj/ROME_9DOF/amsi_serial.mat`. **I restored it from my pre-run backup**, and `sha1sum -c` now shows all 9 pre-modified files back at their pre-run hashes.
- `define_constants.m` and `ROME_9DOF.slx` are unchanged (SHA-1 OK).
- No `define_constants.PRISTINE` was left behind. `runs/` is unchanged (same 5 files, same 22:34 timestamps).
- The branch is still `sim-to-lab`. No commits were made.

---

## 3. Modes 2-4 on paper: existence check

Every file, function and message string the procedure names was checked with `ls` / `grep -F` in `UK-Simulink-v3`.

**Present with the exact name:**
- **Model and settings:** `define_constants.m`, `ROME_9DOF.slx`, `run_mode_table.m`, `rome_mode.m`, `set_run_mode.m`, `run_mode_label.m`, `tools/wire_run_mode_buttons.m`.
- **Checks and plots:** `assert_sim_mode.m`, `apply_pacing.m`, `arm_freeze_banner.m`, `check_9dof.m`, `plot_9dof.m`, `plotting.m`, `gate_all.m`, `check_guard.m`, `run_9dof_cases.m`, `fig_9dof_cases.m`, `verify_scenarios.m`, `verify_wheelmap.m`, `derivation_checks.m`, `plot_style/`.
- **Hardware link and cameras:** `calibrateROMEArm.m`, `jog_joint.m`, `jogROMEArm.m`, `ROMECommand.m`, `motive_cal_check.m`, `Optitrack/motive_to_floor.m`, `tools/wire_motive_emulator.m`.
- **Tests:** `tests/test_suite.m`, `tests/test_motive_emulator.m`, `tests/test_run_length.m`, `tests/test_pacing.m`.
- **Docs:** `docs/lab/PART_C_LAB_VISIT.md`, `PART_F_BENCH_CHECKLIST.md`, `PART_I_RUN_PLAN.md`, `docs/student_procedure/check_procedure.py`.
- **The model's buttons:** the four Mode buttons are in `system_root.xml` inside the `.slx`, calling `rome_mode(1)` to `rome_mode(4)`.
- **Message strings found verbatim:**
  - `PLACE THE ROBOT LIKE THIS`, `Required Workspace`, `*** ARM FROZEN`, `*** ARM WILL MOVE`, `do not match run_mode`, `EnableHardware ~= 0, so sensor_model is forced to 0`.
  - `Turn Fast Restart off`, `stop it before`, `Set run_mode = 1 (type rome_mode(1) or press Mode 1)`, `PRISTINE`, `base travels to`, `all five scenarios passed the gate`.
  - `start posture, firmware degrees`, `Connected.`, `Sending CAL_ARM`, `hit its limit switch`, `ROME arm calibration successfully completed.`, `DO NOT run a robot mode`, `Expected posture`, `calib_log_`, `STOP_ALL`.
  - Two strings exist but are built with format strings or doubled quotes, so they match in print but not as source text: `rome_mode: MODE %d REFUSED` (`rome_mode.m:47`) and `motive_source = ''optitrack'' but` (`motive_cal_check.m:86`).

**Not where the procedure says, or missing:**

| Named in PDF | Status |
|---|---|
| `UK-Arduino/ROME_*_Code/*.ino` (p.3, 8, 9, 15) | Exists at the **repo root** (`ROME/UK-Arduino/`), not inside `UK-Simulink-v3`. p.1 says "All paths are inside the model folder UK-Simulink-v3". |
| `lab_signoff.m` (2.6, 8.5) | **Missing** (marked BLOCKED in the PDF) |
| `compare_hw_sim` (10.2) | **Missing.** Only `compare_hw_sim_core.m` exists. |
| `runs_lab/` (10.1) | Missing. `save_run.m` would create it. |
| `docs/student_procedure/generated/` (A.4) | Missing |
| `figures/matlab` (A.4) | Missing |
| `allowed_ranges.m` (p.5, A.4: "not written yet") | **Exists.** The PDF is out of date. |
| pre-run check (4.5, 8.7, 8.ii: "not built yet") | `prerun_check_core.m` **exists** but is called from nowhere in the student path. "Not wired" would be accurate; "not built" is out of date. |
| `save_run` (10.1: "not wired yet") | `save_run.m` exists. Its signature is `save_run(out, snap, tag, dirOut)`, so a bare `save_run`, as 10.1 implies, would error. |
| Photos `C13_*.jpg`, `F_01_estop.jpg` | Not in the repo (BLOCKED in the PDF) |

---

## 4. Findings

### A. Order and structure

1. **p.2 "Do the sections in order" contradicts the real order (Sections 6-8).** The document order is calibrate (Section 6), then place (Section 7), then set the mode (Section 8). The required order, which 6.1 and the start card (`define_constants.m:430`) both give, is set mode → place → calibrate → open model → Run.
   - A student who obeys p.2 calibrates first and then changes `run_mode` in 8.7. That invalidates the calibration, which is exactly the hazard p.12 warns about.
   - *Fix:* put Sections 6 and 7 inside Section 8 as sub-steps, in run order. Delete the "in order" rule, or make it true.
2. **p.11-14 Section 8: at least six pointer steps.** 8.6, 8.8, 8.14, 8.17, 8.19 and 8.26 (also 8.23, 8.28 and 8.32) say "do the steps above / Section 6 / Section 7 / Section 10". One robot run sends the reader across p.6, 8, 9, 10, 11, 12 and 15.
   - *Fix:* one self-contained "Robot run card" (one page) per mode, with every command written out.
3. **p.3-4 Section 2 (TA one-time setup), Appendix A, and T12, T32, T37-T46 are TA-only material in the student document.** They take about 5 of 22 pages. T37-T46 say "Students see none of them", yet they are printed.
   - *Fix:* move them to a TA manual.
4. **p.4 Section 3 (floor marks) is placed before Section 5 but only mode 4 needs it** (Table 2, p.7: "No lab frame, no taped marks" for modes 2 and 3).
   - *Fix:* move it under "Mode 4" in Section 7.
5. **p.5 Section 4 and p.6 5.3/5.5 duplicate each other** (`edit define_constants`; save and run `define_constants`).
   - *Fix:* keep one copy.
6. **p.5-6: three ways to set the mode** (edit the line, `rome_mode(n)`, Mode button), repeated at 5.4, 8.7, 8.18 and 8.27. The button click is NOT VERIFIED (p.6), and the canvas label resets on save (p.6).
   - *Fix:* teach only `rome_mode(n)`. It edits the file, re-runs `define_constants` and prints the mode last.
7. **p.6 5.6 / p.12 8.4: "Check Fast Restart is off in the toolstrip".** In mode 1 the model is never opened, so there is no toolstrip. The code already refuses Fast Restart in every mode (`apply_pacing.m:58-59, 68-70`), with a message that says what to do.
   - *Fix:* delete both steps and keep T20.
8. **p.8 5.9: `plot_9dof` re-runs the whole simulation** (`plot_9dof.m:33-42`), and it opens 13 figures (12 from `plotting.m` plus 1). The student needs about 2 of them.
   - *Fix:* have `check_9dof` return `out` and call `plot_9dof(out)`, and show only the path figure and the actuator/solve-health figure.
9. **p.11 8.2 and p.13 8.15: every robot run needs a mode-1 simulation, so the mode goes 1 → n → 1 around every run.** Each switch is "a change" that forces recalibration (p.12).
   - *Fix:* let a `prerun` command simulate with mode-1 flags internally while `run_mode` stays at n. This removes 8.2, 8.15 and the "set back to 1" steps, and stops the mode-switch churn from interacting with the calibration rule.

### B. Unclear, or needs knowledge the procedure does not give

10. **p.6 5.2:** the lab-PC path is BLOCKED. A student cannot start. *Fix:* give the path, or say "double-click `ROME_Simulink.prj`". The project file exists in the folder but the procedure never mentions it.
11. **p.5 Table 1 is BLOCKED, so the student does not know which constants they may change** (4.2: "Change only the constants listed in Table 1").
    - The "EDIT THIS BLOCK" in `define_constants.m:1-169` holds 18 assignments. These include TA-only values: `arm_sign`, `arm_offset_deg`, `arm_fw_switch/other`, `motive_cal`, `sensor_model`, `mocap_sigma`, `enc_step_deg`.
    - Its comments contradict the procedure: line 60 says "defaults below are deliberately slow for a first run" but `speed_factor = 1.0`. Line 21 says "in the lab EnableMotive = 1 and the base pose is MEASURED, never typed".
    - *Fix:* reduce the user block to `scenario`, `run_mode`, `speed_factor`, `size_factor` and move the rest to a TA file. Table 1 then has 4 rows and can be written by hand today.
12. **p.8 5.8 / 5.10: the success criteria are BLOCKED** (Table 3 and Figure 2 are empty). The only usable criterion is `RESULT: PASS`. *Fix:* say so explicitly.
13. **p.8 5.9: plots that look like failure.** In my run, "Desired vs. Tracked Heading" and "θ Position Error" show an end-effector heading error of up to ±180°, and the wheel plot marks 120 rpm while the gate limit is 80. Nothing tells the student whether these are expected. *Fix:* explain them in the procedure, fix the plots, or drop those figures from the student set.
14. **p.6 5.6 (implied): unexplained warning.** If the model is opened before `check_9dof`, MATLAB warns `The System object name 'MotiveData' cannot be found` (seen in my run). *Fix:* add it to Section 11, or put `Optitrack/` on the path (project startup).
15. **p.10 7.1-7.2 vs p.11 8.3: two different floor areas** for the same run. "Required Workspace 2.10 x 2.10" and "base travels to 1.27 m from the origin" (about 2.54 m across); in my run both printed. For modes 2 and 3 it is also not said where the area sits relative to the start pose (BLOCKED). *Fix:* print one number, as a box drawn around the start pose.
16. **p.12 8.11: "If the mode line names another mode, or the next line is *** ARM WILL MOVE, press Stop at once".**
    - The student must read the Command Window at the moment the robot starts. The first command leaves about 0.5 s after Run (p.11), while the partner watches the robot.
    - *Fix:* the code should refuse the mismatch, as it already does for flag mismatches. Failing that, check the banner before Run: print it when the model is opened, and have the student confirm it then.
17. **p.9 6.5:** "Joint 1 means J2" (zero-based numbering in the calibration print). *Fix:* make `calibrateROMEArm.m:93` print the 1-based J number instead of documenting the off-by-one (T28).
18. **p.9 6.9: the calibration log is written "next to itself"**, that is inside the repo folder (`calibrateROMEArm.m:74`), which is a OneDrive-synced folder on the author's PC. *Fix:* write it to `runs_lab/`.
19. **p.16 10.1: `save_run` needs `(out, snap, tag)`.** The student cannot build `snap` without `collect_run_snapshot`. *Fix:* a zero-argument `save_run` wired to the last run.
20. **p.12-13 8.16 and p.14 8.24: preconditions that cannot be met.** They require that "a mode 2 (resp. 3) run of the same settings has passed", but no pass condition exists (BLOCKED, `compare_hw_sim` missing). *Fix:* define pass by eye for now (for example "base ends within X cm of the card's end pose, arm did not move"), or drop the precondition.

### C. Jargon not explained

21. These terms are used without definition:
    - **Orbit and reference:** radial / in-track (LVLH), V-bar, R-bar, NMC (p.6 5.3, only as case names), "base path of the full plan of base and arm" (p.7, 12).
    - **Software and settings:** pacing, Fast Restart, toolstrip, canvas, PostLoadFcn (A.1), "the card" / start card, mode line, "fake camera" vs "emulator" (both used).
    - **Robot and arm:** firmware degrees, limit switch, joint map (J2/J5/J6), "reference point (camera pivot)", STOP_ALL, rigid body, Part C / Part F / run plan.
    - **Check output:** residual, RMSE, min s, manipulability.
    - **The author's plan:** "Step 0 / Step 1 / Step 4" (p.5, 8, 22) are steps of the author's plan, not of this procedure.
    - *Fix:* a half-page glossary. Remove the Step-n references from the student copy.
22. **The source brackets clutter every step** (`[define_constants.m:183]` and similar: 138 brackets citing a .m/.md/.ino/.slx file, counted with a regex on the extracted text). A.4 already gives `\showsrcfalse`. *Fix:* build the student copy with it.

### D. Places that send you outside the repo

23. **p.16 Section 11 intro:** "Every entry comes from the project's known-issues record, which the authors keep outside this repository". A.4 says the same (p.21-22).
24. **p.22 G2:** "The paper's own runs are kept by the authors, not in this repository". The README also says the merge record is "kept with the paper's review files, not in this repository".
25. **The docs the procedure sends you to (PART_C, PART_F, PART_I) cite files that are not in the repo:**
    - `TRACKER §0/§2/§3` (dozens of times), `docs/2026-11-18-rome-sim-to-lab-plan.md`, (review plan documents), `CLAUDE.md §20.4`, and the thesis.
    - `ConstrainedMultibodySubmission/ROME_merged/` (PART_F l.8, PART_I l.6, PART_C l.49 "from `ROME_merged`"). This is a different folder name from `UK-Simulink-v3`.
    - `docs/verification/` (PART_F l.5) does not exist yet.
    - *Fix:* rewrite those docs against repo paths only, or mark them TA-internal.
26. **p.1 vs p.3/8/9/15:** "All paths are inside the model folder UK-Simulink-v3", but every `UK-Arduino/...` citation lives one level up. *Fix:* say "repo root" for those.
27. **Stale, contradictory documents in the same folder.**
    - **The start card (printed by every `define_constants`) sends students to `GETTING_STARTED.md` step 3.** That file (last changed 22 Sep, `6ef891d`) tells them to type `EnableHardware = 0/1`, `EnableMotive`, `arm_freeze = 1` and `max_rpm = 60`, and says "First 10 s nothing moves (hold)".
    - All of that contradicts the procedure: 4.3 says never type those flags, and the README says the hold was replaced.
    - `README.txt` (3 Sep) reports "check_9dof 4/4 PASS ... 92.8 rpm" (today: 6 checks, 42.9 rpm). `README.md` item 5 says "Fast Restart is refused on hardware" (procedure: in every mode).
    - *Fix:* delete or archive `GETTING_STARTED.md` and `README.txt`, and change the card's pointer to the procedure section.

### E. Edits beyond the one `run_mode` line

28. The procedure needs these edits on top of `run_mode`:
    - **Student:** `scenario` (5.3; acceptable), and `speed_factor` (8.1, value BLOCKED).
    - **Unknown:** whatever Table 1 will list (BLOCKED).
    - **TA/author, but in the student's edit block:** `motive_cal.R/t/scale/measured` for mode 4 (the refusal text tells the user to "Enter the measured calibration in define_constants.m"), and `arm_sign` / `arm_offset_deg` after the Part F J2 test (the start card and GETTING_STARTED tell the reader to flip `arm_sign(j)`).
    - **A new file:** `lab_signoff.m` (2.6, not yet written).
    - *Fix:* see 11. Keep TA values out of the student block, and load measured calibrations from a signed file.

### F. Safety steps that are easy to skip

29. **The recalibration rule is a paragraph, not a step (p.12).** "Calibrate after the last change of define_constants.m... switching to another mode, or back to mode 1, is such a change". The code check that would enforce it is BLOCKED (p.12). Combined with finding 1, the natural reading order skips it. *Fix:* make it a numbered step right before Run, and wire `prerun_check_core` / `lab/arm_park_record.m`, which already exist.
30. **The stop ladder is missing its middle step.** p.3 (b) / 9.2 "Send STOP_ALL" is BLOCKED (no tested command), so students have Stop in Simulink, then the power cut. The power cut-off location is also BLOCKED (1.2), as are the clear-zone radius (1.4) and the arm-falls-on-power-loss behaviour (1.5). *Fix:* none of the robot sections should open until these four are filled in. Put them on a one-line "pre-flight" box at the top of each run card, not on p.2-3 only.
31. **p.9 6.2 / 9.4: only the power cut stops a running calibration.** This is stated in Section 6's warning, Section 9 and T2, but not in step 6.4 itself, which is where the student presses Enter. *Fix:* put "Hand on power cut-off NOW, nothing typed stops this" in 6.4, and have `calibrateROMEArm` ask for a typed confirmation.
32. **p.13 8.12: STOP_ALL does not latch the arm.** "If the arm moves... stop" is followed by 9.5 "start nothing", but Section 9 says the arm moves again on the next joint command. *Fix:* in 8.12/8.21, tell the student to stop Simulink first, so no further packets are sent.

---

## 5. Verdict

Only the compute times are measured (71 s total, 9.2 s for `check_9dof`, 12.0 s for `plot_9dof`, on the author's PC, not the lab PC). Every other time below is my estimate, NOT VERIFIED.

| Mode | Achievable in one lab session today? | Once the BLOCKED items are filled |
|---|---|---|
| 1 simulation | **Yes.** I reproduced `RESULT: PASS` in about 1 min of compute. My estimate (NOT VERIFIED) is 20-30 min for a first-time student, mostly reading 2.5 pages of expected output and 13 figures. Lab-PC path (5.2) BLOCKED; Table 3 and Figure 2 empty. | Yes, easily. |
| 2 base only | **No.** Section 8 says "do not do this section yet". Blocked: Part F sign-off, power cut-off location, network, `speed_factor` value, pre-run check, `save_run` wiring. | Yes, for a pair with a TA present. My estimate (NOT VERIFIED) is 15-20 min per run: mode-1 simulation, mode switch, placement, calibration up to 2 min, 31 s run, save, mode back. 40 actions the first time, 28 per repeat. |
| 3 base and arm | **No.** As mode 2, plus the J2/J5/J6 maps and a "mode 2 passed" precondition that has no definition. | Borderline. Modes 2 and 3 back to back is about 68 actions in one session, which is feasible but tiring. |
| 4 with cameras | **No.** Refused in code (`motive_cal.measured = false`). Part C, the two-network question and the camera-frame wait are BLOCKED. | Only if modes 2 and 3 were done in earlier sessions. Taped-mark placement with tape and protractor adds an estimated 10 min (NOT VERIFIED). |

**Is it convoluted? Yes, for the robot modes.** The student actions themselves are reasonable: about 7 for mode 1 and about 28 per robot run. What makes the document hard is the framing:
- the sections are in the wrong order (finding 1);
- at least six pointer jumps per run;
- three ways to set the mode;
- TA material mixed into the student text;
- 138 source brackets and 53 BLOCKED notes;
- out-of-date companion docs that contradict it.

### Top 5 cuts or simplifications

1. **Split the document.** Make a ≤6-page student sheet, built with `\showsrcfalse`: a glossary, plus one run card per mode written in execution order (set mode → place → calibrate → open → Run → save). Move Section 2, Appendix A and T12/T32/T37-T46 to a TA manual.
2. **One way to set the mode (`rome_mode(n)`), and let code replace the student checks.** Delete 5.4's alternatives, 5.6/8.4 (Fast Restart is refused by code), the banner-reading race in 8.11, and the 1 → n → 1 mode cycling: a `prerun` command simulates mode 1 internally and runs the existing `prerun_check_core` (ranges, park record, freshness of the calibration).
3. **Shrink "EDIT THIS BLOCK" to `scenario`, `run_mode`, `speed_factor`, `size_factor`.** Move `arm_sign`/`arm_offset_deg`/`arm_fw_*`/`motive_cal`/sensor values to a TA-signed file. Table 1 then exists today with 4 rows.
4. **Delete or archive `GETTING_STARTED.md` and `README.txt`**, and remove the start card's "GETTING_STARTED.md step 3" line. Rewrite `docs/lab/*` against repo paths: no TRACKER, `ROME_merged`, `ConstrainedMultibodySubmission` or review-plan references.
5. **Make the end of a run one command and one criterion.**
   - Wire `save_run` with no arguments to the last `out` and snapshot, and write calibration logs into `runs_lab/`.
   - For mode 1, the pass criterion is `RESULT: PASS` plus the path figure. Have `check_9dof` return `out` so `plot_9dof(out)` does not simulate again, and show 2 figures, not 13. Explain or fix the ±180° heading plot and the 120-vs-80 rpm line.
   - Define a by-eye pass rule for modes 2 and 3, so that 8.16 and 8.24 can be satisfied.

---

## Appendix: full MATLAB output for mode 1

Script: `<scratch>\student_walk\student_mode1_walk.m`. Exported figures: `...\scratchpad\student_walk\figs\`. Start time: Thu 19 Nov 2026 22:52:18.

```

### STEP 5.1  version
26.1.0.3346908 (R2026a) Update 5

### STEP 5.2  pwd
<repo>\UK-Simulink-v3

### STEP 5.3  (edit define_constants: no change, case 1) line 5 reads:
scenario = 1;          % 1 ellipse | 2 V-bar | 3 R-bar | 4 NMC | 5 circle

### STEP 5.4  (keep run_mode = 1) line 7 reads:
run_mode = 1;          % THE RUN MODE, 1 to 4 (the author's four modes, 19 Nov 2026):

### STEP 5.5  define_constants
MODE 1: Simulation only (end-effector dynamics sim, pose data from sim)

==========================================
      ORBITAL TRAJECTORY PARAMETERS       
      Date: 19-Nov-2026 22:52:22                
==========================================
Semi-major Axis (a):    1.400 units
Eccentricity (e):       0.500
Inclination (i):        0.00 degrees
Initial Anomaly (nu0):  0.00 degrees
------------------------------------------
TIME ANALYSIS:
  Normalized Period:    10.41 s
  MAX REAL TIME (Lap):  26.02 s
------------------------------------------
SPATIAL ANALYSIS:
  Distance Scale:       0.50
  MAX REAL DISTANCE:    1.05 meters
  Required Workspace:   2.10 x 2.10 meters
==========================================

==========================================
      PLACE THE ROBOT LIKE THIS           
==========================================
  Base    x  +0.678 m   y  -0.008 m   yaw -179.16 deg
  Arm     -2.16 +128.58  -14.82   -0.00  -23.75   -1.32 deg
  Rates   all zero. Let it sit still before you start.

  Order: set run_mode FIRST (rome_mode(n)) -> place base -> calibrateROMEArm -> Run.
  calibrateROMEArm parks the arm at the angles above (mapped to
  firmware degrees). arm_sign / arm_offset_deg are derived from
  the AR3 files, not measured: run the jog test, GETTING_STARTED.md
  step 3, before the first robot run. Joint commands are capped to
  the firmware ranges, wheels to max_rpm.
==========================================

[Warning: The orbit exceeds a 2m x 2m area! Check dist_scale or a.] 
[> In define_constants (line 440)
In student_mode1_walk (line 11)] 

### STEP 5.6  Fast Restart (batch equivalent: load_system + get_param)
MODE 1: Simulation only (end-effector dynamics sim, pose data from sim)

==========================================
      ORBITAL TRAJECTORY PARAMETERS       
      Date: 19-Nov-2026 22:52:28                
==========================================
Semi-major Axis (a):    1.400 units
Eccentricity (e):       0.500
Inclination (i):        0.00 degrees
Initial Anomaly (nu0):  0.00 degrees
------------------------------------------
TIME ANALYSIS:
  Normalized Period:    10.41 s
  MAX REAL TIME (Lap):  26.02 s
------------------------------------------
SPATIAL ANALYSIS:
  Distance Scale:       0.50
  MAX REAL DISTANCE:    1.05 meters
  Required Workspace:   2.10 x 2.10 meters
==========================================

==========================================
      PLACE THE ROBOT LIKE THIS           
==========================================
  Base    x  +0.678 m   y  -0.008 m   yaw -179.16 deg
  Arm     -2.16 +128.58  -14.82   -0.00  -23.75   -1.32 deg
  Rates   all zero. Let it sit still before you start.

  Order: set run_mode FIRST (rome_mode(n)) -> place base -> calibrateROMEArm -> Run.
  calibrateROMEArm parks the arm at the angles above (mapped to
  firmware degrees). arm_sign / arm_offset_deg are derived from
  the AR3 files, not measured: run the jog test, GETTING_STARTED.md
  step 3, before the first robot run. Joint commands are capped to
  the firmware ranges, wheels to max_rpm.
==========================================

[Warning: The orbit exceeds a 2m x 2m area! Check dist_scale or a.] 
[> In define_constants (line 440)
In pLoadSystem (line 40)
In load_system (line 17)
In student_mode1_walk (line 13)] 
[Warning: The System object name 'MotiveData' cannot be found. Enter a class name of a System object that is on the path, uses correct capitalization, and does not include the file
extension.] 
[> In pLoadSystem (line 40)
In load_system (line 17)
In student_mode1_walk (line 13)] 
FastRestart = off

### STEP 5.7  check_9dof
MODE 1: Simulation only (end-effector dynamics sim, pose data from sim)

==========================================
      ORBITAL TRAJECTORY PARAMETERS       
      Date: 19-Nov-2026 22:52:31                
==========================================
Semi-major Axis (a):    1.400 units
Eccentricity (e):       0.500
Inclination (i):        0.00 degrees
Initial Anomaly (nu0):  0.00 degrees
------------------------------------------
TIME ANALYSIS:
  Normalized Period:    10.41 s
  MAX REAL TIME (Lap):  26.02 s
------------------------------------------
SPATIAL ANALYSIS:
  Distance Scale:       0.50
  MAX REAL DISTANCE:    1.05 meters
  Required Workspace:   2.10 x 2.10 meters
==========================================

==========================================
      PLACE THE ROBOT LIKE THIS           
==========================================
  Base    x  +0.678 m   y  -0.008 m   yaw -179.16 deg
  Arm     -2.16 +128.58  -14.82   -0.00  -23.75   -1.32 deg
  Rates   all zero. Let it sit still before you start.

  Order: set run_mode FIRST (rome_mode(n)) -> place base -> calibrateROMEArm -> Run.
  calibrateROMEArm parks the arm at the angles above (mapped to
  firmware degrees). arm_sign / arm_offset_deg are derived from
  the AR3 files, not measured: run the jog test, GETTING_STARTED.md
  step 3, before the first robot run. Joint commands are capped to
  the firmware ranges, wheels to max_rpm.
==========================================

[Warning: The orbit exceeds a 2m x 2m area! Check dist_scale or a.] 
[> In define_constants (line 440)
In run (line 100)
In check_9dof (line 50)
In student_mode1_walk (line 17)] 

=======================================================
  9-DOF VERIFICATION  -  simulation only, no hardware
=======================================================

  static: [OK] reference starts from rest and is continuous through the onset
  static: [OK] at rest on the seed the solve stays put  (1 step: 2.2e-07 m, 0.73 rpm)
MODE 1: Simulation only (end-effector dynamics sim, pose data from sim)

==========================================
      ORBITAL TRAJECTORY PARAMETERS       
      Date: 19-Nov-2026 22:52:31                
==========================================
Semi-major Axis (a):    1.400 units
Eccentricity (e):       0.500
Inclination (i):        0.00 degrees
Initial Anomaly (nu0):  0.00 degrees
------------------------------------------
TIME ANALYSIS:
  Normalized Period:    10.41 s
  MAX REAL TIME (Lap):  26.02 s
------------------------------------------
SPATIAL ANALYSIS:
  Distance Scale:       0.50
  MAX REAL DISTANCE:    1.05 meters
  Required Workspace:   2.10 x 2.10 meters
==========================================

==========================================
      PLACE THE ROBOT LIKE THIS           
==========================================
  Base    x  +0.678 m   y  -0.008 m   yaw -179.16 deg
  Arm     -2.16 +128.58  -14.82   -0.00  -23.75   -1.32 deg
  Rates   all zero. Let it sit still before you start.

  Order: set run_mode FIRST (rome_mode(n)) -> place base -> calibrateROMEArm -> Run.
  calibrateROMEArm parks the arm at the angles above (mapped to
  firmware degrees). arm_sign / arm_offset_deg are derived from
  the AR3 files, not measured: run the jog test, GETTING_STARTED.md
  step 3, before the first robot run. Joint commands are capped to
  the firmware ranges, wheels to max_rpm.
==========================================

[Warning: The orbit exceeds a 2m x 2m area! Check dist_scale or a.] 
[> In define_constants (line 440)
In load_system (line 17)
In check_9dof (line 96)
In student_mode1_walk (line 17)] 
MODE 1: Simulation only (end-effector dynamics sim, pose data from sim)
*** ARM FROZEN at arm_freeze_deg: q0_9dof(4:9) mapped and clamped = [-2.160 51.422 104.825 -0.000 -23.754 -1.320] (firmware deg); arm_freeze_user = 1, ROME_9DOF sends this start posture for the whole run (EnableHardware = 0) ***
  run flags (read back after the sim): EnableMotive 1, motive_source emulator, motive_emulate_noise 1, sensor_model 1

  ran 31.00 s, 621 steps at dt = 0.050 s (20 Hz)
  [OK] constraint satisfied      max residual 4.29e-12
  [OK] wheel speed under ceiling peak 42.9 rpm (max_rpm 80, firmware 120)
  [OK] away from singularity     min s 0.6773 (screening at 0.30), manip 0.3036
  [OK] end effector bounded      same-sample RMSE 8.011e-03 m (limit 2.196e-02; sampling lead 1.464e-02, noise response 1.145e-04)
       lead-corrected steady RMSE 6.045e-04 m (command vs reference one sample ahead)
       measured-pose RMSE 8.059e-04 m (what the controller read, against the reference)
  [OK] arm inside firmware range  closest approach 36.3 deg to a limit (margin 2)
       fw min [-2 51 92 0 -32 -115]
       fw max [122 55 105 0 -24 -1]
  [OK] arm clear of the floor    z_min 0.3200 m at t = 0.00 s (margin 0.05 m, whole run, paper geometry)
       xy_min -0.0489 m (signed distance to the base square, not gated); ar3 logged, not gated: z_min 0.3698 m, xy_min -0.1307 m
       peak arm torque 6.93 N m  (no actuator limit on record)
       base travels to 1.27 m from the origin -- clear that space,
       it is NOT the 4.2 m x 4.2 m of the 3-DOF orbit 

=======================================================
  RESULT: PASS
=======================================================

### check_9dof returned ok=1 after 9.2 s

### STEP 5.9  plot_9dof
MODE 1: Simulation only (end-effector dynamics sim, pose data from sim)

==========================================
      ORBITAL TRAJECTORY PARAMETERS       
      Date: 19-Nov-2026 22:52:40                
==========================================
Semi-major Axis (a):    1.400 units
Eccentricity (e):       0.500
Inclination (i):        0.00 degrees
Initial Anomaly (nu0):  0.00 degrees
------------------------------------------
TIME ANALYSIS:
  Normalized Period:    10.41 s
  MAX REAL TIME (Lap):  26.02 s
------------------------------------------
SPATIAL ANALYSIS:
  Distance Scale:       0.50
  MAX REAL DISTANCE:    1.05 meters
  Required Workspace:   2.10 x 2.10 meters
==========================================

==========================================
      PLACE THE ROBOT LIKE THIS           
==========================================
  Base    x  +0.678 m   y  -0.008 m   yaw -179.16 deg
  Arm     -2.16 +128.58  -14.82   -0.00  -23.75   -1.32 deg
  Rates   all zero. Let it sit still before you start.

  Order: set run_mode FIRST (rome_mode(n)) -> place base -> calibrateROMEArm -> Run.
  calibrateROMEArm parks the arm at the angles above (mapped to
  firmware degrees). arm_sign / arm_offset_deg are derived from
  the AR3 files, not measured: run the jog test, GETTING_STARTED.md
  step 3, before the first robot run. Joint commands are capped to
  the firmware ranges, wheels to max_rpm.
==========================================

[Warning: The orbit exceeds a 2m x 2m area! Check dist_scale or a.] 
[> In define_constants (line 440)
In run (line 100)
In plot_9dof (line 34)
In student_mode1_walk (line 25)] 
MODE 1: Simulation only (end-effector dynamics sim, pose data from sim)

==========================================
      ORBITAL TRAJECTORY PARAMETERS       
      Date: 19-Nov-2026 22:52:40                
==========================================
Semi-major Axis (a):    1.400 units
Eccentricity (e):       0.500
Inclination (i):        0.00 degrees
Initial Anomaly (nu0):  0.00 degrees
------------------------------------------
TIME ANALYSIS:
  Normalized Period:    10.41 s
  MAX REAL TIME (Lap):  26.02 s
------------------------------------------
SPATIAL ANALYSIS:
  Distance Scale:       0.50
  MAX REAL DISTANCE:    1.05 meters
  Required Workspace:   2.10 x 2.10 meters
==========================================

==========================================
      PLACE THE ROBOT LIKE THIS           
==========================================
  Base    x  +0.678 m   y  -0.008 m   yaw -179.16 deg
  Arm     -2.16 +128.58  -14.82   -0.00  -23.75   -1.32 deg
  Rates   all zero. Let it sit still before you start.

  Order: set run_mode FIRST (rome_mode(n)) -> place base -> calibrateROMEArm -> Run.
  calibrateROMEArm parks the arm at the angles above (mapped to
  firmware degrees). arm_sign / arm_offset_deg are derived from
  the AR3 files, not measured: run the jog test, GETTING_STARTED.md
  step 3, before the first robot run. Joint commands are capped to
  the firmware ranges, wheels to max_rpm.
==========================================

[Warning: The orbit exceeds a 2m x 2m area! Check dist_scale or a.] 
[> In define_constants (line 440)
In load_system (line 17)
In plot_9dof (line 38)
In student_mode1_walk (line 25)] 
MODE 1: Simulation only (end-effector dynamics sim, pose data from sim)
*** ARM FROZEN at arm_freeze_deg: q0_9dof(4:9) mapped and clamped = [-2.160 51.422 104.825 -0.000 -23.754 -1.320] (firmware deg); arm_freeze_user = 1, ROME_9DOF sends this start posture for the whole run (EnableHardware = 0) ***

  orbit (elliptical): 12 figures from plotting.m on end-effector quantities,
  plus one with the wheel, joint, health and torque traces.

### plot_9dof returned ok=1 after 12.0 s

### STEP 10 (Save As equivalent): figures written to the scratch folder
  fig 16 "9-DOF actuators and solve health - orbit (elliptical)" -> <scratch>\student_walk\figs\16_9-DOF_actuators_and_solve_health_-_orbit_elliptical_.png
[Warning: UI components will not be included in the output. To include UI components, use the 'exportapp' function.] 
  fig 15 "ROME Robot Performance Metrics" -> <scratch>\student_walk\figs\15_ROME_Robot_Performance_Metrics.png
  fig 14 "Theta Velocity Error" -> <scratch>\student_walk\figs\14_Theta_Velocity_Error.png
  fig 13 "Y Velocity Error" -> <scratch>\student_walk\figs\13_Y_Velocity_Error.png
  fig 12 "X Velocity Error" -> <scratch>\student_walk\figs\12_X_Velocity_Error.png
  fig 11 "Theta Position Error" -> <scratch>\student_walk\figs\11_Theta_Position_Error.png
  fig 10 "Y Position Error" -> <scratch>\student_walk\figs\10_Y_Position_Error.png
  fig 9 "X Position Error" -> <scratch>\student_walk\figs\09_X_Position_Error.png
  fig 8 "Theta Velocity Comparison" -> <scratch>\student_walk\figs\08_Theta_Velocity_Comparison.png
  fig 7 "Y Velocity Comparison" -> <scratch>\student_walk\figs\07_Y_Velocity_Comparison.png
  fig 6 "X Velocity Comparison" -> <scratch>\student_walk\figs\06_X_Velocity_Comparison.png
  fig 5 "Theta Comparison" -> <scratch>\student_walk\figs\05_Theta_Comparison.png
  fig 4 "Y Position Comparison" -> <scratch>\student_walk\figs\04_Y_Position_Comparison.png
  fig 3 "X Position Comparison" -> <scratch>\student_walk\figs\03_X_Position_Comparison.png
  fig 2 "fig2" -> <scratch>\student_walk\figs\02_fig2.png
  fig 1 "fig1" -> <scratch>\student_walk\figs\01_fig1.png

### total wall time 52.9 s
EXIT=0
Thu, Nov 19, 2026 10:53:29 PM
```
