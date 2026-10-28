ROME 9-DOF model — everything needed to open and run it, nothing else.

Verified from this folder, 3 September 2026:

    check_9dof        4/4  PASS   residual 2.30e-12 | 92.8 rpm | manip 0.3242
                                  | EE RMSE 1.586e-02 m
    verify_wheelmap        PASS   the 9-DOF block addresses the same motors
                                  as the 3-DOF model (1.8e-15 rpm apart)
    verify_scenarios 12/12  PASS   references checked against the manuscript
                                  and against the Clohessy-Wiltshire equations


OPEN IT
-------
    open_system('ROME_9DOF')

define_constants runs automatically on open; it is the model's PostLoadFcn.


SELECT A CASE STUDY
-------------------
One line in define_constants.m:

    scenario = 1;   % 1 ellipse | 2 V-bar | 3 R-bar | 4 NMC | 5 circle

Editing the FILE is what takes effect. Assigning the variable in the
workspace does not: the PostLoadFcn re-runs define_constants on every load
and overwrites it before the first step.

Status of each:
    1  ellipse   WORKING, verified. Reproduces the path the 3-DOF model flies.
    2  V-bar     DIVERGES. Under investigation; do not run on hardware.
    3  R-bar     not yet exercised end to end
    4  NMC       not yet exercised end to end
    5  circle    not yet exercised end to end


RUNNING WITH NO HARDWARE
------------------------
Both gates default to off, so the model runs standalone:

    EnableHardware = 0    ROMECommand transmits nothing
    EnableMotive   = 0    q_meas is the delayed command, not measurement

With EnableMotive = 1 the StateSource switch takes the base pose from
Motive and the arm joints from the ARM, telemetry line instead.


THE HARDWARE BLOCK
------------------
ROMECommand opens a tcpclient to the ESP32 at 192.168.4.1:3333 and sends

    ROME,<4 wheel rpm>,<6 joint deg>\n

reading GV, and ARM, telemetry back. It takes jointRad in RADIANS and
converts with rad2deg immediately before formatting the packet. Radians and
SI hold everywhere upstream; degrees exist only on the wire.

Two things it does not do, worth knowing before energising:
  - wheel speeds are not clipped. The Mega's map() is integer and unclamped,
    so a command past 120 rpm leaves the useful range. Scenario 1 peaks at
    92.8, so there is margin, but another case study or scale may not.
  - joint angles are not clamped, and the limit check inside
    AR3Serial.updateStates is commented out.


FILES
-----
  ROME_9DOF.slx           the model
  define_constants.m      PostLoadFcn: constants, selector, seeds
  scenario_defaults.m     manuscript parameters and tabletop scaling per case
  scenario_seeds.m        solves q0 and qd0 for the selected case
  ik9_warm_start.m        damped least squares IK used by the seeds
  ik9_jac.m, ik9_fk.m     kinematics for the seeds and the checks
  EndEffectorTrajectory.m source of the reference block
  rome_uk_block.m         source of the UKDynamics block
  ROMECommand.m           the hardware sink
  Ground Vehicle/         GroundVehicleCommand
  Optitrack/              MotiveData, natnet
  resources/, .prj        project metadata

  check_9dof.m            acceptance gate; errors on failure
  verify_scenarios.m      references against the manuscript and CW
  verify_wheelmap.m       both paths address the same motors
  OrbitTrajectory_ref.m   the 3-DOF block, for the comparison in the above

The model carries its own copy of the two MATLAB Function bodies, so it runs
without EndEffectorTrajectory.m and rome_uk_block.m. They are kept because
they are where those blocks are edited.


DO NOT RUN THE CHECKS WITH THE MODEL OPEN
-----------------------------------------
check_9dof closes and reloads the model. Doing that under a live Simulink
editor crashed MATLAB R2026a repeatedly. Close Simulink first:

    bdclose('all')

If the editor starts crashing on open, the saved window layout has gone
stale; reset it with reset_model_windows.py from the Stage 1 folder.


REBUILDING
----------
build_9dof.m lives in ROME_9DOF_Integration/, not here, together with
ROME_Main.slx and the debug scripts. It regenerates ROME_9DOF.slx from the
3-DOF model. If this copy is edited in the Simulink editor, do not rebuild:
the rebuild starts from ROME_Main.slx and would discard those edits.
