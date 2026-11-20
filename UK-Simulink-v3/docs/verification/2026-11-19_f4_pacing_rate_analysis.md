# F4: is the ROME command stream faster than real time? (2026-11-19)

Read-and-analyse only. No MATLAB was run, no code was edited. Everything below is from existing logs and source; items I could not check are in the last section.

## 1. Measured numbers (from logs, all with EnablePacing on, rate 1, T_stop 31.0203 s unless stated)

| Log (docs/verification/) | line | ROME-line span, wall | ratio | median interval | min lag (wall minus sim) / end | first ROME line after connect | wall of first 2 s sim |
|---|---|---|---|---|---|---|---|
| p2t1c_green.log | 142 | 29.103 s | 0.939 | not printed | not printed | not printed | not printed |
| p2t1c_makefail.log | 153-155 | 29.048 s | 0.937 | 0.0465 s | -1.958 at t=29.65 / -1.952 | 0.521 s | 1.820 s |
| p2t1c_makefail.log | 507-509 | 30.480 s | 0.983 | 0.0486 s | -0.527 at 24.35 / -0.520 | 0.519 s | 1.806 s |
| p2t1c_makefail.log | 726-728 (T_stop 18.01) | 17.464 s | 0.970 | 0.0477 s | -0.538 at 10.80 / -0.536 | 0.523 s | 1.822 s |
| p2t1c_makefail.log | 903-905 | 30.471 s | 0.983 | 0.0487 s | -0.530 at 12.55 / -0.529 | 0.513 s | 1.816 s |
| p2t1c_makefail.log | 1260-1262 | 30.467 s | 0.983 | 0.0487 s | -0.534 at 22.15 / -0.533 | 0.518 s | 1.822 s |
| p2t1c_makefail_iv_motive_rerun2.log | 165-167 | 29.008 s | 0.936 | 0.0465 s | -1.993 at 28.45 / -1.992 | 0.514 s | 1.818 s |
| p2t1c_test_suite.log | 2495-2497 | 30.468 s | 0.983 | 0.0487 s | -0.532 at 18.80 / -0.532 | 0.522 s | 1.820 s |

Whole `sim()` wall time (green.log:93, makefail.log:103/457, rerun2, test_suite.log:2495):
33.22, 33.24, 32.74, 33.00, 32.82 s for 31.02 s simulated. So the whole run is about 1.7 to 2.2 s longer than the simulated time, never shorter. The line span varies by 1.5 s (29.0 to 30.5 s) while the whole-run time varies by 0.5 s.

Task 4b (task4b_callback_order_experiment.log:57,113,169,225), 20 s simulated, no ROMECommand: off 5.48 s, InitFcn-on 22.05 s, StartFcn-on 21.74 s, pacing-on-before-sim 21.72 s. Overhead 1.7 to 2.0 s over 20 s, the same 1.7 s as the paced hardware runs; pacing itself is not fast in that experiment (no per-line stamps there, only total).

Every paced run has exactly 1 of 620 intervals under half a step, at t = 0 (min 0.0053 to 0.0166 s). There is no burst of short intervals after it.

## 2. How timing was measured

- Stamp: `tests/stubs_hw/RobotLinkRecorder.m:28` (constructor) and `:37` (`write`): `twall = double(java.lang.System.nanoTime())*1e-9`, with sim time from `get_param('ROME_9DOF','SimulationTime')` at `:27`/`:36`. nanoTime is monotonic and unaffected by clock adjustment. Differences are valid.
- Where in the step: `write` is called from `stepImpl` (`tests/stubs_hw/ROMECommand.m:127` for the ROME line), after the sim-time `get_param` and before the file append. Each stamp is taken in the step, once per step, before its own file I/O. The file I/O adds jitter but cannot bias a sum of intervals.
- First stamp: connect and START_GV happen inside step 0: `ROMECommand.m:90` constructs the link (C record), `:91` is `pause(0.5)`, `:92` writes START_GV, then the first ROME line at `:127`. The first ROME line is therefore 0.513 to 0.523 s after the connect record (matches the log column above).
- Statistics: `tests/test_hw_nocam.m:327-346`. Span is `twR(end) - twR(1)` (`:330`), ratio `spanWall/spanSim`; lag is `(twR - twR(1)) - (tR - tR(1))` (`:337`); intervals `diff(twR)` (`:339`). The span starts at the first ROME line, so compile, setup and the 0.5 s pause are excluded from the span. Whole-sim wall is `toc(t0)` around `sim()` (`:250-261`), which includes compile.
- The stub equals the real `ROMECommand.m` with only the transport constructor replaced (log line 2 of each run: "byte-identical", 2 replacements), so the send logic below applies to the real file.

## 3. What the real ROMECommand and firmware do

- Real `ROME_merged/ROMECommand.m` (identical to the stub except transport): a ROME line is written in every `stepImpl` call (`:100-128`), no rate limit, no timestamp in the message (format `ROME,w1..w4,j1..j6`, `:123`). One `pause(0.5)` at connect (`:91`).
- Firmware `UK-Arduino/ROME_Mega_Code/ROME_Mega_Code.ino`:
  - `:10` `#define WATCHDOG_TIMEOUT_MS 250`.
  - `:180-190` `checkWatchdog()`: if `started` and `millis() - lastCommandTime > WATCHDOG_TIMEOUT_MS`, `desiredRPMs` and the integrators are zeroed. This is a silence watchdog on arrival; there are no timestamps in the commands.
  - `:144-155` a speed packet sets `desiredRPMs[i]` and `lastCommandTime = millis()`. The setpoint is held until the next packet or the watchdog expiry. Sending every 46 ms instead of 50 ms is well inside 250 ms.
  - The wheel PID runs on its own 20 Hz timer (`:95`), independent of arrival times.
- NOT VERIFIED: this Mega parses bare `w1,w2,w3,w4` (`:144-155`), not `ROME,...` lines with joints. Something between (ESP32, TCP port 3333) must translate. I did not read that code, so I did not check whether it rate-limits or buffers.

## 4. Reasoning: real or artefact?

The wall stamps are real elapsed time, so the measured intervals are not a measurement error in themselves. What is an artefact is the reading "the pacing runs 2.6 to 7 % fast for the whole run". Evidence:

1. Pacing cannot give a steady rate below real time. MathWorks (below) says only "approximately". The settled lag is what matters.
2. In the 0.983 runs the lag reaches its minimum (-0.53 s) at t = 12.6 to 24.4 s and then stays there (end value equals min within 0.01 s). So the speed-up is a one-off offset of 0.52 to 0.54 s, then rate 1. That offset equals the `pause(0.5)` (first line 0.513 to 0.523 s after connect). The median 0.0487 is the signature of that offset being absorbed over the first third of the run (0.5 s over about 10 to 20 s is about 1.3 ms per step: 0.0487), not of a steady 2.6 % fast stream.
3. The pause sits inside step 0, before the first stamp. If the pacing clock was anchored before step 0 (NOT VERIFIED), the pause is already spent from the pacing budget, so the later lines arrive early relative to the first stamp. Consistent with this, whole-`sim()` wall is T + 1.7 to 2.2 s in all runs, while only the position of the first stamp moves.
4. In the 0.936 runs the lag min (-1.958 at 29.65 s, -1.993 at 28.45 s) is within 0.01 s of the end value, so those also look settled near the end, but the log gives only min and end, not the curve. The extra 1.4 s over the 0.983 runs is not explained by the pause (NOT VERIFIED). The task 1c report notes these were the first runs in a fresh MATLAB session; a longer one-off cost in step 0 before the first stamp (java class load, first interpreted calls) would produce this, again as an offset.
5. The first-interval check asked for: the t = 0 interval is short (0.005 to 0.017 s) but only 1 interval is, and the first 2 s span 1.806 to 1.822 s, so only 0.18 s of the deficit is in the first 2 s. The catch-up is therefore not a burst after a long first interval; it is spread over 10 to 30 s. That does not match a simple "long first step then fast steps" model, so how Simulink spreads the catch-up is NOT VERIFIED.

Conclusion: the 2.6 to 7 % "median" is mostly a start-up offset of 0.5 to 2 s in where the stream sits relative to wall time, not a persistent rate error. For 0.983 runs the evidence supports "offset then rate 1". For 0.936 runs it supports "offset settled by the end" but the per-line curve is not in the logs.

## 5. Effect on the robot

- The command count is unchanged (621 steps, 623 writes), the values are unchanged (q_cmd is bit-identical to the reference run, test (i)), and wheel and joint caps act on values, not time. `max_rpm` (80) and the guard do not change. The green run's peak wheel command was 42.9 of 80 rpm (task-1c report).
- Time: the stream finishes 0.5 to 2.0 s early on a 31 s lap (end-of-run lag in the table: -0.52 to -1.99 s), i.e. at most 6.5 %, and the median lines over-state it for the 0.983 runs. This is only an offset relative to wall time; the end shift of 0.5 to 2 s is the estimate asked for.
- Correction to the "same speeds, runs faster" picture: if each held speed lasts 3.5 to 7 % less time, an open-loop robot covers 3.5 to 7 % less distance per setpoint, not the same path sooner. But the shortening is concentrated in the first 10 to 30 s and then stops (section 4), so the open-loop distortion is at most the 0.5 to 2 s of travel at that time. Whether the hardware run is closed loop on mocap (controller reads measured pose and corrects) was not checked here: NOT VERIFIED.
- Watchdog: faster arrival only helps. No effect on the 250 ms watchdog.
- Safety: no limit is relaxed. For a first hardware test the effect is small and bounded; it does not need a fix before that test.

## 6. Is a fix needed? Options (none implemented)

No fix needed for safety. If the lap-time accuracy matters:
1. Keep check (ii) as is (span >= 0.9 x simulated, passes at 0.936 to 0.983) and add a second, informational check on the settled lag (end lag within +/- 2.5 s).
2. Measure the pacing clock properly: stamp the connect time and the first step separately, and compute the ratio from t = 5 s onward (after the offset settles) so the 0.5 s pause and first-step cost are excluded.
3. Move the `pause(0.5)` out of step 0 (into setup, or before pacing starts) so the stream is not offset; this edits Keanu's ROMECommand.m, so it needs the owner's decision.
4. Time-stamp commands, or have the sender hold each step to the wall clock (`tic/toc` in stepImpl) if exact wall timing is required. The firmware does not use timestamps, so this needs firmware or ESP32 changes; not recommended.

## 7. MathWorks documentation

- URL: https://www.mathworks.com/help/simulink/slref/simulationpacingoptions.html ("Simulation Pacing Options"). Fetched 2026-11-19 through a page-summarising tool, so the quotes below are as returned by that tool; re-check on the page.
- "Approximate ratio of simulation time to wall clock time, specified as a positive scalar number."
- "the software slows the simulation to a rate of approximately one second of simulation time per second of clock time."
- "Use simulation pacing to slow simulation to help analyze and interact with your model." (the stated purpose is analysis and interaction, not hardware timing).
- "The simulation pacing does not apply when you step through a simulation by clicking Step Forward or Step Backward."
- The page does not say whether pacing catches up after a slow step, and does not claim a guarantee. Best-effort is my reading of "approximate", not a MathWorks statement. A separate search result for the same topic (Simulation Pace block, aeroblks) says the sample time sets how often the interface synchronises with the wall clock; I did not open it.

## 8. NOT VERIFIED

- Where the Simulink pacing clock starts (before or after setupImpl and step 0), and how it distributes catch-up after a slow step.
- The per-line curve of the lag for the 0.936 runs (logs hold only min, end, median, min interval; the record file is in a temp copy and was not kept).
- Why the fresh-session runs lost an extra 1.4 s (cause of the 0.936 vs 0.983 difference).
- The ESP32/bridge between ROMECommand lines and the Mega (rate limit, buffering, translation); the Mega file parses bare wheel values only.
- Whether the hardware run closes the loop on mocap in a way that corrects any distance shortfall.
- That the real transport (tcpclient write) behaves like the recorder: the test never sends to a socket, so real write latency and Nagle/batching are untested.
- The MathWorks quotes are as returned by a summarising fetch tool.
