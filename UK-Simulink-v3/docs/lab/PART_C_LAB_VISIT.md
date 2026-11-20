# Part C — Lab-frame visit: one-visit checklist

Prepared 19 Nov 2026 (drafted by Claude for the author). **NOT EXECUTED.** Every blank below is empty on purpose.
Spec: `docs/2026-11-18-rome-sim-to-lab-plan.md` Part C steps 9-13. Plan: `docs/superpowers/plans/2026-11-19-plan3-lab-procedure.md` Task 1.

Rule for this document: a statement about hardware or code carries its file and line, or its document and printed page.
Anything not found is written **NOT VERIFIED**, with what would verify it. Paths are relative to
`ConstrainedMultibodySubmission/` unless they start with `UK-Arduino` (lab repo `ucf-asrl/ROME`, read-only).
Thesis = `HUT Thesis Complete Review Draft - Keanu Brayman.pdf`, **printed** page numbers (the PDF page is the printed number + 1).

| Visit | |
|---|---|
| Date | ____ |
| Who | ____ |
| Motive PC / MATLAB PC | ____ / ____ |
| `define_constants.m` SHA-256 on the day (`Get-FileHash`) | ____ |
| Folder for everything recorded (suggested: `docs/lab/visit_data/`) | ____ |

## Safety (short)
- Robot at the lab: e-stop / power cut-off location: **NOT VERIFIED** → fill in: ____ (Part F item 25 tests it).
- Part C moves the robot by hand or at rest only. **No `EnableHardware = 1`, no wheel or arm command in this visit.**
- If the arm is mounted, keep it unpowered or parked while you push the base. Whether the arm holds or drops when unpowered: **NOT VERIFIED** (thesis p. 9: six stepper motors; nothing found on brakes). Support it by hand or keep clear: ____.
- The base mass is 18 kg without the arm (thesis Table 1, p. 15). Two people for the push test.
- Whether the base can be pushed by hand with the motors off (50:1 gearboxes, `UK-Arduino/ROME_Mega_Code/ROME_Mega_Code.ino` L7): **NOT VERIFIED**. Record what you find in C12.0.

## Open question that blocks C11
**Q-D2 (TRACKER §3, opened 18 Nov): where is the origin (the target) in the room?** OPEN.
The thesis uses metres but defines the origin by where the robot is placed (q0 + displacement); Fig. 7 shows a white sphere at the focus, and whether it is a physical target marker is **NOT VERIFIED** (thesis Figs 7, 8, 22; TRACKER Q-D2).
Spec step 10 defines the floor frame as the paper's floor-fixed LVLH (origin = target, x radial, y in-track, z up, θ_v from +x).
**C11 cannot be completed until the author decides the origin.** Decision: ____ (author, date ____). Basis: ____.
Known about the room, from the spec's "Author's answers" row "Lab frame source" (not re-read from the thesis): four corner markers, tracked region about x −2.1 … 0.7 m, y ±1.2 m (thesis Fig. 8); no origin, axes, up-axis or volume given. `docs/HARDWARE_IMPLEMENTATION.md` L123 and L414 (unicast, 127.0.0.1) are stale; the code uses multicast (C9.1).

## What the code assumes (read this before the visit)
All from `ROME_merged/Optitrack/MotiveData.m` unless stated.
- Multicast `239.255.42.99`, port `1511` (L24-25); joins the group with no interface given (L42-46); 500 ms receive timeout (L47); 2048-byte buffer (L50).
- Origin = the first frame read (L53-55). x and y are relative to it, z is absolute (L64-66). Rotation is absolute, `quat2eul(q,'XYZ')` (L69-74).
- Rigid body at a fixed byte offset: MATLAB indices `41:68`, 7 singles `[x y z qx qy qz qw]` (L95-100). `RigidBodyID` is declared (L23) and never read (header note L14-17).
- A timeout returns x = y = z = 0, q = [0 0 0 1] (L105-109), not the last frame.
- The model adds `[x, y, −z_rot]` to `q0_9dof(1:3)` (`docs/verification/2026-11-19_plan2_interface_map.md`, "Model chain"). There is **no axis remapping and no rotation of x, y** in the code. So the code is right only if Motive's x, y are the floor's horizontal axes (z up) and z-rotation is the yaw.
- Units are not stated anywhere in the code (interface map, "Motion capture").

---

## 0. Before leaving the desk (no lab needed)
- [ ] **C0.1 Print the start-pose card of every case** (spec step 11). The card is printed by `define_constants.m` L343-349 ("PLACE THE ROBOT LIKE THIS": Base x, y, yaw; Arm angles; rates zero). It depends on `scenario`, `size_factor` (L44), `speed_factor` (L40), `z_work` (L163), `arm_home` (L169), `tool_tilt_deg` (L187) (file comment before L343), so print it with the values you will use, and write them down.
  `scenario = 3; define_constants` does **not** work, because L5 reassigns `scenario = 1`. For each N = 1 … 5:
  1. Copy `define_constants.m` to a safe place first (outside `ROME_merged`).
  2. Edit L5 to `scenario = N;`. Leave `EnableHardware = 0`, `EnableMotive = 0` (L7-8) and `base_override = [NaN; NaN; NaN]` (L25).
  3. In MATLAB (R2026a, full path; per TRACKER §0 R2025a fails in `derivation_checks`), from `ROME_merged`: `define_constants`
  4. Copy the block between the two `=====` lines headed "PLACE THE ROBOT LIKE THIS" into the table below.
  5. After N = 5 restore L5 to `scenario = 1;` and compare with the saved copy (`fc` or `git diff --no-index --word-diff`). Expected: no difference.

  | Case | scenario value | size_factor / speed_factor | Base x (m) | Base y (m) | Base yaw (deg) | Arm J1…J6 (model deg) |
  |---|---|---|---|---|---|---|
  | 1 ellipse | 1 | ____ / ____ | ____ | ____ | ____ | ____ |
  | 2 V-bar | 2 | ____ / ____ | ____ | ____ | ____ | ____ |
  | 3 R-bar | 3 | ____ / ____ | ____ | ____ | ____ | ____ |
  | 4 NMC | 4 | ____ / ____ | ____ | ____ | ____ | ____ |
  | 5 circle | 5 | ____ / ____ | ____ | ____ | ____ | ____ |

  Pass: all 5 rows filled from the printout (no value typed from memory), `define_constants.m` restored and unchanged. The poses are in the **floor frame** (origin Q-D2). The card's arm angles are model degrees; `calibrateROMEArm` maps them to firmware degrees (`calibrateROMEArm.m` L21-22).
- [ ] **C0.2 Bring:** tape measure (m, 1 mm resolution), floor tape (two colours), marker pen, phone/camera with a scale in every photo, a level, and for Part F the inclinometer. Result: ____.
- [ ] **C0.3 Software on the MATLAB PC:** `ROME_merged/lab/record_motive.m` (`rec = record_motive(seconds, outFile)`, Plan 3 Task 1) now exists (written and tested offline against a fake transport, 19 Nov; NOT yet run against the real stream: its network class `lab/MotiveUdpTransport.m` is untested). Usage `rec = record_motive(seconds, 'C9R_x.mat', struct('live', true, 'rbID', N))` (the file name must end in .mat and must not exist; without `live = true` it refuses to open a socket). If it misbehaves on the day, use C9R fallbacks A and B. Wireshark installed on the Motive PC or the MATLAB PC: **NOT VERIFIED** → check: ____.

---

## 1. Step 9 — record in Motive (and on the network)
For every item: write what is **displayed**, save a screenshot (name in the photo list, C13), then compare with the pass condition. If the value differs from what the code assumes, **do not change the code in the lab**: write FAIL and the value; the author decides.

### C9.1 Streaming settings, NatNet and Motive versions
Code expects: multicast, `239.255.42.99`, port `1511` (MotiveData L24-25). The model's block parameters are `MulticastIP` and `DataPort` (header note L19-20), so a different value can be set there without editing `MotiveData.m`.
Which NatNet version the stream is: **NOT VERIFIED**. `Optitrack/assemblypath.txt` L1 points to `NatNetML.dll` on another user's desktop (`C:\Users\ke350927\…`); the DLL's version is unknown. `MotiveData` no longer uses the SDK (header L3-4).

| Record | Value |
|---|---|
| Motive version (Help → About) | ____ |
| NatNet version shown in the streaming pane | ____ |
| Streaming enabled; transmission type (multicast / unicast) | ____ |
| Multicast address | ____ |
| Data port | ____ (command port: ____) |
| Local interface / IP of the Motive PC | ____ |
| IP of the MATLAB PC on that network | ____ |
| Streaming toggles that are ON (rigid bodies, marker sets, labelled markers, unlabelled markers, skeletons, other) | ____ |

Pass: transmission = multicast, address = `239.255.42.99`, port = `1511` (or the differences are written down for the block parameters).
Network check (new): the MATLAB PC also has to reach the robot's ESP32 access point at 192.168.4.1:3333 (`jogROMEArm.m` L5-6, `ROMECommand.m` L3-4), whose SSID is in `UK-Arduino/ROME_ESP32_Code/ROME_ESP32_Code.ino` L6 (the password is on L7; it is not copied here). `joinGroup` is called without an interface (MotiveData L46), so which network interface receives the multicast: **NOT VERIFIED**. Test: with the MATLAB PC joined to the ESP32 access point **and** to the Motive network at the same time (how: ____), run C9R-A. Pass: packets arrive.

### C9.2 Up-axis
Code needs Z up (see "What the code assumes"). Motive's default up-axis: **NOT VERIFIED** (not in any file read).
Record: up-axis setting ____ (screenshot). Check by reading the stream: lift the robot (or a marker) by a measured amount and see which of x, y, z changes (C12.4).
Pass: z is up. **If the up-axis is Y (or anything else): FAIL for the code as written** (it reads x, y as the floor and the last Euler angle as yaw, with no remapping): stop and report.

### C9.3 Ground plane and calibration state
Record: how the ground plane was set (tool, date) ____; last wanding date and result (mean error as shown by Motive) ____; screenshot of the calibration result.
Check: with the robot at rest at the first taped mark, the rigid body's displayed z ____ m versus the measured height of the pivot above the floor ____ m.
Pass: difference ≤ 10 mm. (The 10 mm and 2° bounds used throughout this document are the provisional `frame_check` bounds of Plan 3 Task 1 Step 1, **[Claude's call — author to confirm]**, basis: tape placement about 5 mm and hand alignment about 1°, doubled.)

### C9.4 Units
The code does not state units (interface map). Stream units: **NOT VERIFIED**. Decided by C12 (a measured 0.5 m push must read 0.5 in the stream, or 500 / 50 …).
Record the unit the stream shows: ____ → scale factor to metres: ____.
Pass: after the scale, C12 passes.

### C9.5 Rigid body ID, pivot, markers
Code: "rigid body 1" by fixed offset (MotiveData L14-17, L23, L95-96).

| Record | Value |
|---|---|
| Number of rigid bodies in the stream | ____ |
| Name and streaming ID of the robot's rigid body | ____ / ____ |
| Position of the robot's body in the stream order (1st, 2nd, …) | ____ |
| Number of markers on the robot (thesis p. 19: four, at the corners of the ground vehicle) | ____ |
| Pivot position (Motive Properties pane), x y z in the body frame | ____ |
| Pivot offset from the centre of the 0.45 × 0.45 m plate (thesis Table 1, p. 15), measured with a tape | dx ____ dy ____ m |
| Rigid body axes: which physical side of the base is body +x, +y | ____ |
| Other assets in the volume (other rigid bodies, marker sets) | ____ |

Pass: exactly one rigid body in the stream, or the robot's body is the first one (this is what the code reads); pivot offset recorded; if |offset| > 10 mm it goes into the calibration (C10), not into the code. If more than one rigid body is streamed: FAIL for the fixed offset until C9R shows where the robot's body sits.
Body-axis marking feeds Part F item 26 (base +x must be marked on the chassis): done / not done ____.

### C9.6 Volume size
The code warns when the orbit exceeds 2 × 2 m (`define_constants.m`, "Safety Check" after L349), and prints "Required Workspace" (L318).
Record: the tracked region corners in the stream's coordinates (walk the robot or a marker to each corner and read the stream), x from ____ to ____, y from ____ to ____ (m); camera count ____ (`define_constants.m` L12 says four; verify); camera positions photographed.
Pass: the region contains every case's required workspace (the value printed by C0.1 step 3, per case) plus the margin the author sets: ____ m.

### C9.7 Occlusion with the arm up (relates to open question C1 in the spec)
The arm is not mounted for the push test? ____. If it is, and the arm can be parked at a start posture, record with the arm parked: markers visible ____ of ____; Motive's tracking state / mean error ____.
Pass: all robot markers tracked with the arm parked; if not, write which marker is hidden and where the arm is. If not done in this visit: write "NOT DONE, needs `calibrateROMEArm` (Part F)".

---

## 2. Record a raw NatNet packet (settles the packet layout)
Why: `MotiveData` reads the rigid body at the fixed 1-based byte index 41 (L95-96) and never reads the ID (TRACKER §0 row "A Motive dropout"). The bytes before the rigid-body section depend on what is streamed (marker sets and their names, labelled and unlabelled markers), so index 41 may be right for **one** configuration only. This is a check, not a fact: **NOT VERIFIED** until the packet is decoded against the NatNet packet format of the version noted in C9.1 (that document has not been read for this draft).

Record one packet file per condition. Keep the Motive settings of C9.1 unchanged for all of them and write them next to each file.

| # | Condition | File | Seconds | Datagrams | Largest datagram (bytes) |
|---|---|---|---|---|---|
| a | Robot at rest at mark 1, only the robot streamed as in C9.1 | `C9R_a_rest.mat` | ____ | ____ | ____ |
| b | Robot being pushed slowly along +x (reuse C12.1 if the recording covers it) | `C9R_b_moving.mat` | ____ | ____ | ____ |
| c | Robot **hidden** (all markers covered) while Motive keeps streaming: the rigid body is untracked | `C9R_c_untracked.mat` | ____ | ____ | ____ |
| d | Same as (a) with one more asset streamed (if the lab uses more than one), or with unlabelled markers toggled | `C9R_d_other_setting.mat` | ____ | ____ | ____ |

How to record:
- **Preferred:** `rec = record_motive(seconds, outFile)` (Plan 3 Task 1). It saves raw packets (bytes plus receive time) and the parsed pose7 and never overwrites a file. Written 19 Nov (C0.3); `opts.rbID` must be the rigid-body Streaming ID found in C9.1, default 1; `opts.live = true` is needed for the real stream.
- **Fallback A, MATLAB, UNTESTED (MATLAB was not run when this was written):** the same socket as `MotiveData.m` L41-52, saving what arrives.
  ```matlab
  import java.net.*;
  sock = MulticastSocket(1511);  grp = InetAddress.getByName('239.255.42.99');
  sock.joinGroup(grp);  sock.setSoTimeout(500);
  buf = zeros(1,2048,'int8');  pkt = DatagramPacket(buf, length(buf));
  raw = {};  tr = [];  t0 = tic;
  while toc(t0) < 5                      % seconds: set per the table
      try
          sock.receive(pkt);  n = pkt.getLength();  d = pkt.getData();
          raw{end+1} = typecast(int8(d(1:n)),'uint8');  tr(end+1) = toc(t0); %#ok<AGROW>
      catch, end
  end
  sock.leaveGroup(grp);  sock.close();
  save('C9R_a_rest.mat','raw','tr');    % pick a file name that does not exist yet
  ```
- **Fallback B:** Wireshark capture with the display filter `udp.port == 1511`, saved as `.pcapng`. Wireshark availability: **NOT VERIFIED**.

What to check offline (author, desk), per file; write the result in the table:
1. Does the receive loop see more than one kind of datagram (different lengths, different first bytes)? ____
2. `b = typecast(int8(raw{k}(41:68)),'single')` (what MotiveData does, L96): are `b(1:3)` the position Motive displays (to 1 mm, in the unit of C9.4) and `norm(b(4:7))` = 1 within 1e-3? (a) ____ (b) ____
3. In (c) (untracked), what is at 41:68? Zeros, the last pose, or something else? ____ (This decides what the model sees when markers are lost while Motive is still streaming; the code only handles a silent stream, L105-109.)
4. In (d), does the rigid body move away from index 41? ____ (Shows the layout depends on the streaming toggles.)
5. Is every datagram shorter than 2048 bytes (L50)? ____ Is one frame one datagram? ____
6. Where in the packet is the rigid-body ID, and which value is the robot's? ____ (Needed to stop relying on a fixed offset: Plan 2 Task 1.)
7. Tracked flag (the parser rejects a frame whose params bit 0x01 is clear; MotiveData's default Layout reads params at bytes 73-74, `natnet_parse_rigidbody.m`). Decode bytes 73-74 (`typecast(raw{k}(73:74),'uint16')`) of a captured packet: is bit 0x01 set while the body is tracked (a) and clear while its markers are covered (c)? (a) ____ (c) ____ Record the NatNet version (C9.1): ____ Is it >= 3.0? ____ If it is below 3.0, the params position moves by 4 + 20 n bytes (n = markers of the body; NatNetClient.py L107-131) and camera runs stay REFUSED until the layout is fixed.

Pass: items 2 and 5 are yes for the configuration that will be used in every run, and items 3, 4 and 6 are answered. Otherwise: FAIL, report to the author (the reader must parse by ID, not by offset).

---

## 3. Step 10 — floor frame (desk, after the visit)
Not a lab action. After C11 and C12, the Motive → floor transform comes from `cal = fit_motive_cal(recs, marks)` (Plan 3 Task 1, `ROME_merged/lab/fit_motive_cal.m`, written and tested offline on synthetic data; Procrustes/Kabsch least-squares with scale; the fit returns `fit_ok` (residual gate, provisional) and never sets `measured` (only the sign-off does); Umeyama 1991, IEEE TPAMI 13(4):376-380, DOI 10.1109/34.88573: bibliographic data checked at Crossref on 19 Nov, the paper text itself NOT opened). `marks` is a table with variables `x`, `y` (m), `heading` (rad), optional `z` (pivot height, m). It needs ≥ 6 taped marks with known floor coordinates and headings (the planned offline test uses 6), and a recording of the rigid body at each mark. So record the marks in C11 with that use in mind.

## 4. Step 11 — tape the floor
Blocked by Q-D2 (above). Origin decision: ____.

| # | Item | Floor coordinates in the floor frame (m, deg) | Measured how | Done |
|---|---|---|---|---|
| T1 | Origin mark | (0, 0) by definition | ____ | ☐ |
| T2 | +x axis line (length ____ m) | along +x | ____ | ☐ |
| T3 | +y axis line (length ____ m) | along +y; check 90° to x with the 3-4-5 triangle or a square | angle error ____ deg | ☐ |
| T4-T8 | Start pose, cases 1 to 5: tape the base **pivot** position and the heading line | from the table in C0.1 | ____ | ☐ ☐ ☐ ☐ ☐ |
| T9+ | Extra calibration marks so that, with T1 and T4-T8, there are ≥ 6 well-spread marks (name the floor coordinates and heading of each) | ____ | ____ | ☐ |

Rules:
- Tape the **pivot** (C9.5), not the geometric centre, or write the pivot offset for each mark.
- The start poses come only from the C0.1 printout. Do not type values from memory or from a previous version of the card.
- With `EnableMotive = 1` the start base pose is `q0_9dof(1:3)` plus the displacement since the first frame (`MotiveData.m` L53-55; `define_constants.m` L291 re-solves the arm only when `~EnableMotive`), so the robot must really stand at the card's pose: a placement error is invisible to the model (spec "What we know", row "Camera path").
Pass: every mark placed within 5 mm and 1° of its coordinates (provisional, C9.3) and measured twice; marks lie inside the tracked region of C9.6.

## 5. Step 12 — push test
Purpose: check that the stream, after the transform, matches signs and sizes. Planned evaluator: `rep = frame_check(recPush, cal)` (Plan 3 Task 1, `ROME_merged/lab/frame_check.m`, written and tested offline; PASS bounds 10 mm and 2 deg, provisional; it expects the whole push test in ONE recording with four rests of at least 3 s); the table below can still be filled by hand.

- [ ] **C12.0 How the robot is moved:** by hand / jacked on a trolley / other ____. Observation on pushing with the motors off (C Safety): ____.
- [ ] **C12.1 Set up:** robot with its pivot on mark 1, heading ____. Start the recording (`record_motive` or C9R-A), **let it sit 10 s** (rest).
- [ ] **C12.2 Push +0.5 m along floor +x**, using the tape (axis T2) as the guide. Hold 10 s at rest. Measured distance on the tape: ____ m.
- [ ] **C12.3 Push +0.5 m along floor +y** (T3). Hold 10 s. Measured: ____ m.
- [ ] **C12.4 Lift check (only if C9.2 was not decided by a marker test):** lift one side by about ____ m (measured) and read which stream axis changed ____.
- [ ] **C12.5 Rotate +90°** about the pivot, counter-clockwise seen from above (z up), using the heading line. Hold 10 s. Measured angle: ____ deg.
- [ ] **C12.6 Stop the recording**; file: ____.

Record from the stream (position in the unit of C9.4; rotation in the unit the code outputs, radians, `MotiveData.m` L68 comment):

| Step | Raw Δ in Motive axes (x, y, z) | After the Motive → floor transform (x, y) | Expected | Difference | Pass |
|---|---|---|---|---|---|
| +0.5 m x | ____ ____ ____ | ____ ____ | +0.5, 0 | ____ | ☐ |
| +0.5 m y | ____ ____ ____ | ____ ____ | 0, +0.5 | ____ | ☐ |
| +90° | Δx_rot ____ Δy_rot ____ Δz_rot ____ | yaw change ____ | +90° | ____ | ☐ |

Pass conditions:
1. Signs: the transformed Δ has the expected sign on each axis (x push → +x, y push → +y), and the sizes are within 10 mm; the yaw within 2° (provisional bounds, C9.3).
2. If the transform is the identity (Motive x, y already are the floor axes), the raw Δ in the table must match as it is.
3. **Yaw sign as the model uses it.** The model chain adds `−z_rot` to the start yaw (`[x, y, −z_rot]`, interface map). So for the model's yaw to increase by 90°, `MotiveData`'s `z_rot` must change by **−90°** for the counter-clockwise rotation. Record Δz_rot ____. Pass: Δz_rot = −90° within 2°, **and** x_rot and y_rot do not change (that also proves z is up). If Δz_rot = +90°: FAIL for the code as written (yaw sign); report.
4. The sizes before any scale are consistent with C9.4 (factor ____).
5. Rest periods: the stream is steady at rest (spread ____ mm). Note any drift: ____.

## 6. Step 13 — photos and floor plan
Take every photo with a tape measure or a ruler in view. Folder: ____. Fill the "Taken" column.

| File name | Content | Taken |
|---|---|---|
| `C13_01_lab_overview_wide.jpg` | Whole room from the door | ☐ |
| `C13_02_floorplan_topview.jpg` | Top view of the taped floor (or a hand-drawn plan with dimensions, photographed) showing origin, axes, all marks | ☐ |
| `C13_03_origin_tape.jpg` | Origin mark, close | ☐ |
| `C13_04_xaxis_tape.jpg` | +x axis line with its end mark | ☐ |
| `C13_05_yaxis_tape.jpg` | +y axis line with its end mark | ☐ |
| `C13_06_case1_start_pose.jpg` … `C13_10_case5_start_pose.jpg` | Robot on each start pose (five files; name them 06, 07, 08, 09, 10 in case order) | ☐ ☐ ☐ ☐ ☐ |
| `C13_11_cameras_overview.jpg` | All cameras in view (and `C13_11b_…` per camera if needed) | ☐ |
| `C13_12_robot_markers_closeup.jpg` | Marker positions on the robot | ☐ |
| `C13_13_robot_axes_marked.jpg` | Base +x / +y marked on the chassis | ☐ |
| `C13_14_motive_streaming_settings.png` | Screenshot of the streaming pane (C9.1) | ☐ |
| `C13_15_motive_asset_rigidbody.png` | Screenshot of the robot's rigid-body properties, pivot (C9.5) | ☐ |
| `C13_16_motive_calibration_result.png` | Screenshot of the calibration result (C9.3) | ☐ |
| `C13_17_motive_axes_view.png` | Motive 3D view showing the axis gizmo and the ground plane (C9.2) | ☐ |
| `C13_18_estop_location.jpg` | The physical power cut-off and where it is (also needed by Part F) | ☐ |
| `C13_19_pc_network_settings.png` | MATLAB PC network adapters (C9.1) | ☐ |

Pass: all files exist, are named as above, and every photo shows its scale reference. The top-view floor plan carries the coordinates of every mark.

---

## 7. Results and sign-off
| Item | Result (PASS / FAIL / NOT DONE) | Note |
|---|---|---|
| C0.1 start-pose cards | ____ | |
| C9.1 streaming settings and versions | ____ | |
| C9.2 up-axis | ____ | |
| C9.3 ground plane | ____ | |
| C9.4 units | ____ | |
| C9.5 rigid body, pivot | ____ | |
| C9.6 volume | ____ | |
| C9.7 occlusion with the arm | ____ | |
| C9R raw packets (a-d) and offline checks 1-6 | ____ | |
| C11 taping (Q-D2 decided) | ____ | |
| C12 push test | ____ | |
| C13 photos | ____ | |

Open after the visit: ____.
Signed (name) ____ Date (YYYY-MM-DD) ____. The date goes into `lab_signoff.m` (the Part C sign-off date field, Plan 3 Task 4, not yet written).
