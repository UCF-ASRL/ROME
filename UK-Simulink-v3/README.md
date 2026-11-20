This model folder: the paper's 9-DOF formulation (ROME_9DOF_Integration, final paper V3) merged with the lab's hardware files from ucf-asrl/ROME UK-Simulink-v3; the merge record is kept with the paper's review files, not in this repository.
The step-by-step procedure for students (calibration, hardware runs) is docs/student_procedure/ROME_student_sheet.pdf, and for the TA docs/student_procedure/ROME_TA_manual.pdf (sources .tex beside them); do not use the hardware scripts without them. docs/student_procedure/ROME_lab_procedure.pdf (19 Nov) is superseded by these two.
To check the folder, run in MATLAB R2026a from the tests folder of this model folder (UK-Simulink-v3/tests in the lab repository): `test_suite`

## Changes vs lab main

What changes on the robot compared with lab `main @ 9867ca4` (author's review record, 19 Nov 2026):
1. `max_rpm` 60 -> 80 (author decision, 19 Nov: "80 is okay. The 60 was an old estimate, but we actually dont know for sure. I think 80 is okay. This should be tunable on the define_constants though"). Firmware hard cap 120.
2. The 10 s hard hold is replaced by the raised-cosine onset: ellipse wheel peak 77.8 -> 42.9 rpm. The hold was a safety measure against the earlier divergence.
3. StopTime 26 s -> `T_stop` = 31 s on the ellipse (55 s on scenarios 2-5): the run now covers the whole path.
4. SensorModel noise is forced off on hardware (`sensor_model = 0` when `EnableHardware ~= 0`; a run with it on is refused). Lab main has no SensorModel.
5. Real-time pacing is off only when `EnableHardware = 0`, enforced at every run start (`apply_pacing`); Fast Restart is refused on hardware.
