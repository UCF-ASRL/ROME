# ROME student lab procedure (Plan 3 Task 5, spec Part G)

DRAFT of 19 Nov 2026. **Compiled** on the author's PC on 19 Nov (night, doc pass) with MiKTeX `pdflatex` (`%LOCALAPPDATA%\Programs\MiKTeX\miktex\bin\x64`, not on PATH), three passes in a copy of this folder (the second still asked for a Rerun): no LaTeX error; the PDF is `ROME_lab_procedure.pdf` beside the `.tex`. Rebuild it after every edit (pdflatex twice in a copy, once more if the log says Rerun) or on Overleaf as below.

## Files

| File | What it is | Edited by |
|---|---|---|
| `ROME_lab_procedure.tex` | The procedure. 11 sections plus the TA appendix. Packages: article, graphicx, amsmath, siunitx, enumitem, booktabs, hyperref, tikz only. | hand |
| `figures/floor_frame.tex` | TikZ top view of the floor frame (x radial, y in-track, z up, heading from +x), schematic. `\input` by the procedure. | hand |
| `figures/fig_9dof_case_<n>_path.pdf` (n = 1..4) | Expected path figures. **Generated** by `fig_9dof_cases` once Task 5 Step 0 makes it self-contained. Not present yet. | MATLAB |
| `generated/constants_table.tex` | Table of the constants a student may change (name, meaning, unit, default, allowed range, source). **Generated** by the Task 5 Step 1 script from `allowed_ranges.m`. Not present yet. | MATLAB |
| `generated/expected_results.tex` | Expected simulation results per case (the paper's Table 3 values). **Generated** by the Task 5 Step 1 script. Not present yet. | MATLAB |
| `number_allowlist.tsv` | The numbers allowed in the student part, each with a file:line citation. | hand |
| `check_procedure.py` | Static checker. Run `python -I check_procedure.py` from this folder after every edit. | hand |
| `citation_lock.tsv` | The text of every line cited with `\src{file:line}` as it was when the citation was checked by hand. The checker fails when a cited line moves or changes. | `check_procedure.py --update-lock`, after re-reading |

Every generated input is wrapped in `\IfFileExists`, so the document compiles without it and shows a red `[BLOCKED: ...]` note in its place.

## Keeping the citations right

The procedure cites code by line number, and the code keeps changing. Use this sequence:
1. `python -I check_procedure.py` reads the cited files from the working tree. `--rev HEAD` reads them from the last commit. The lock was first built against commit `77a1239` (19 Nov; the first draft was locked at `a9d9acf` and 13 citations plus 3 allow-list rows were re-pointed after the final-review commits `faf9185`, `77a1239`). F9 (19 Nov, three modes of case 1) rebuilt it from the WORKING TREE, at a time when F1 had uncommitted edits in `ROME_merged/tests/test_suite.m` and `define_constants.m`: the citations `tests/test_suite.m:1-34, 30-32, 47-48, 113` rest on F1's edit, so `--rev HEAD` fails on them until F1 commits, and they drift again if F1 changes those lines.
   On Windows the checker's output can contain characters the console code page cannot print (UnicodeEncodeError); run it as `python -I -X utf8 check_procedure.py`.
2. After a commit that touches a cited file, run without `--rev`. Every cited line that moved or changed is listed with its old and new text.
3. Fix each `\src{...}` and each `number_allowlist.tsv` row by hand, re-read the new lines, then run `python -I check_procedure.py --update-lock` and check again.

**Printed versus review citations (proc4, 19 Nov).** The printed text (`\src{...}`) cites only files of the lab repository (paths relative to `UK-Simulink-v3/`, or `UK-Arduino/...`). A source that is a review document (TRACKER.md, the plans, run logs, CLAUDE.md, the manuscript) goes on a LaTeX comment line of its own, `% review-src: path:lines; path:lines`, never after text on the same line (a mid-line `%` comments out the rest of the paragraph). In this review folder the checker resolves and locks those entries like `\src`; in the repo (`--repo`, or detected) it lists them as "review-only, not checkable from the repo". A printed `\src` into a review document or into `ROME_merged/`, and a mid-line `% review-src:`, FAIL in both modes (make-it-fail evidence: the proc4 rows of the review folder's test register).

## Upload to Overleaf

1. Run `python -I check_procedure.py` here. Expected: `check_procedure PASS: 6 of 6 checks` (`docs/verification/2026-11-19_plan3_t5_check_procedure.log`).
2. Make a zip of this folder with its sub-folders: `ROME_lab_procedure.tex`, `figures/`, and `generated/` if it exists. The `.py`, `.tsv` and this README may go in too. Overleaf ignores them.
3. In Overleaf choose New Project, then Upload Project, and pick the zip.
4. In the project menu set Compiler to pdfLaTeX and Main document to `ROME_lab_procedure.tex`.
5. Compile. Expected: a PDF with red `[BLOCKED: ...]` notes and no LaTeX error (the local MiKTeX build of 19 Nov had none). Report any LaTeX error back.
6. To print a copy without the small source brackets, change `\showsrctrue` to `\showsrcfalse` in the preamble.

When Step 0 or Step 1 has produced new files, upload only `figures/` and `generated/` again (Overleaf: Upload, replace).

## Still BLOCKED (each shows as a red note in the PDF)

| What | Unblocked by |
|---|---|
| Constants table (Section 4) | Plan 3 Task 4 (`allowed_ranges.m`), then Task 5 Step 1 (needs MATLAB) |
| Expected results table and its tolerances (Section 5) | Task 5 Step 1 (needs MATLAB) |
| Expected path figures (Section 5) | Task 5 Step 0 (needs MATLAB) |
| `plot_9dof` output and the run time on the lab PC (Section 5) | Task 5 Step 4 cold-read test; lab PC |
| Model folder on the lab PC, lab PC install, suite run on the lab PC (Sections 2, 5) | Plan 3 Task 7 |
| Floor origin, taped marks, all Part C photos (Sections 2, 3, 7) | Part C lab visit; author decision Q-D2 |
| Power cut-off location, clear-zone radius, arm on power loss, stop tests and stop times (Sections 1, 9) | Part F bench session, items 24d and 25 |
| J2 joint map, calibrated-arm photo (Section 6) | Part F item 24b |
| Placement tolerance per case, how to move the base, reference point (Section 7) | Plan 2 Task 7 report; Part C items C9.5, C12.0 |
| The mode selector: the `run_mode` line of define_constants, `rome_mode(n)`, the Mode buttons (Sections 1, 4, 5, 8) | DONE in software: F12 wiring 7686e2c and fix round 1 94ab2bc (MODE line at every Run, the refusals `arm_freeze_banner:mode`, `set_run_mode:locked`, `rome_mode:running`, `assert_sim_mode:notsim`, label reset at save). A button click stays NOT VERIFIED (no test can click). |
| Arm freeze (mode 2) and its banner ARM FROZEN / ARM WILL MOVE (Section 8) | DONE in software (F1, df60e12; F7 fix round 1, 317d3d6: with the arm frozen the controller reads the model's arm, the old frozen-arm refusal is gone). The parked pose is guaranteed equal only once F11 (pre-run check against the calibration record) exists. On the robot NOT VERIFIED. |
| Bench sign-off for mode 2 without the joint-map test (Sections 2, 8) | CLOSED 19 Nov: the author decided "Full Part F" (TRACKER Q-P4-a): the whole bench checklist, joint-map test included, before any robot run, mode 2 included. |
| J2, J5, J6 joint maps (Section 8 box; every robot mode, since the whole bench checklist comes first) | Part F item 24, bench jog test (TRACKER section 0, J2 row) |
| Mode 4 only: NaN-safe start and dropout stop, start procedure A (Section 8) | Focus plan F6, F8 |
| Pre-run check, sign-off file (Sections 2, 4, 8) | Plan 3 Task 4 = focus plan F2 (`prerun_check.m`, `lab_signoff.m`) |
| Mode 4 refused until the lab frame is measured (`motive_cal.measured`); modes 2 and 3 need no lab frame | In code (F5b, F7: `motive_cal_check:unmeasured` for 'optitrack' only); Part C visit sets the values |
| How much shorter the base travel is in modes 2 and 3 because of the pacing start-up lag (Section 8, NOT VERIFIED) | TRACKER section 2, row "Pacing start-up lag ACCEPTED", to-do (1), from the Task 1c recordings |
| Workspace size for modes 2 and 3, and where it lies relative to the start pose (Section 7) | The run plan `docs/lab/PART_I_RUN_PLAN.md` (the model prints 2.10 x 2.10 m for case 1). That plan was rewritten for the four modes on 19 Nov (TRACKER Q-P4-b) and gives the size the model prints; where the area lies relative to the start pose is still open. |
| Reduced speed of the first hardware run (Section 8) | Plan 2 Task 8 study, Plan 3 Task 6 run plan |
| `save_run`, `compare_hw_sim` (Section 10) | Plan 3 Task 3 |
| A tested command that sends `STOP_ALL` on its own (Sections 1, 9) | Not planned in any task yet (the Part F snippets are NOT RUN) |
| Network for the robot and the cameras (Sections 2, 6) | TRACKER Q-L2 |

`check_procedure.py` prints the current count of `BLOCKED` and `NOT VERIFIED` markers on its last lines.
