function motive_cal_check(EnableHardware, EnableMotive, motive_source, motive_cal)
%MOTIVE_CAL_CHECK  Refuse a REAL-camera run on the robot while the camera frame calibration motive_cal is not measured (F5b, narrowed by F7).
%   motive_cal_check(EnableHardware, EnableMotive, motive_source, motive_cal)
%       checks the values GIVEN. define_constants.m calls it this way with its own local values (every run of the
%       file), so it works in any workspace: several tools (calibrateROMEArm, returnHomeROMEArm, jog_joint,
%       check_guard, derivation_checks, ...) run define_constants inside their own function workspace, where the
%       base workspace may be empty or stale (F5b review I1, 19 Nov 2026). motive_cal may be [] (undefined).
%   motive_cal_check()
%       reads the four from the BASE workspace (read NOW). apply_pacing.m calls it this way for a hardware run
%       (EnableHardware ~= 0); the model calls apply_pacing from its InitFcn AND StartFcn, so the check runs at every
%       run start, Fast Restart included, and also when a student sets the flags in the workspace without re-running
%       define_constants. Base workspace is right there: it is what the model compiles against.
%   Rule: EnableHardware ~= 0 AND EnableMotive ~= 0 AND motive_source = 'optitrack' requires motive_cal.measured = true.
%   Why: with the real cameras the controller's base pose is motive_to_floor(pose7, motive_cal) (Enabled Subsystem
%   block MotiveFloor), an ABSOLUTE floor pose. Through the placeholder transform of define_constants (R identity,
%   t zero, scale 1, UNMEASURED until Plan 3 Part C) the controller would believe the robot is wherever Motive's
%   origin and axes put it, and command the base toward the reference from there.
%   motive_source = 'emulator' (focus plan F7, mode 2) is NOT refused: the pose comes from the model itself and goes
%   out and back through the same motive_cal (floor_to_motive then motive_to_floor), which is exact for any proper
%   transform, so no lab frame is needed (focus plan: "The emulator needs no lab frame").
%   Refusals (error, run refused):
%     motive_cal_check:noflag      (base form) EnableHardware or EnableMotive undefined
%     motive_cal_check:badflag     either is not a finite numeric scalar
%     motive_cal_check:nosource    camera hardware run (both flags on) and motive_source undefined (base form)
%     motive_source_check:invalid  camera hardware run and motive_source neither 'emulator' nor 'optitrack'
%     motive_cal_check:nocal       'optitrack' camera hardware run and motive_cal undefined or missing a field
%                                  (R t scale measured)
%     motive_cal_check:unmeasured  'optitrack' camera hardware run and motive_cal.measured is not true
%     motive_cal_check:badcal      measured, but R is not a 3x3 rotation (|R'R - I| > 1e-6 or det <= 0), t not a
%                                  finite 3x1, or scale not finite and > 0
%   Tests: tests/test_motive_source.m (iv), (I1); tests/test_motive_model.m check (v).
if nargin == 0
    flags = {'EnableHardware', 'EnableMotive'};  % base-workspace flags read (cell of char)
    val = [NaN NaN];                             % their values (flag)
    for k = 1:2
        try
            v = evalin('base', flags{k});        % flag value (flag)
        catch
            error('motive_cal_check:noflag', '%s is not defined in the base workspace; run define_constants first. Run refused.', flags{k});
        end
        if ~(isnumeric(v) && isscalar(v) && isfinite(v))
            error('motive_cal_check:badflag', '%s must be a finite numeric scalar; run refused.', flags{k});
        end
        val(k) = v;
    end
    EnableHardware = val(1);  EnableMotive = val(2);
    if ~(EnableHardware ~= 0 && EnableMotive ~= 0)
        return                                   % not a camera run on the robot: nothing to check
    end
    try
        motive_source = evalin('base', 'motive_source');   % camera source (char)
    catch
        error('motive_cal_check:nosource', ['EnableHardware = 1 and EnableMotive = 1 but motive_source is not defined in ' ...
              'the base workspace; run define_constants. Run refused.']);
    end
    try
        motive_cal = evalin('base', 'motive_cal');         % camera frame calibration struct (see define_constants)
    catch
        motive_cal = [];                         % undefined: refused below if the source is 'optitrack'
    end
elseif nargin == 4
    fl = {EnableHardware, EnableMotive};         % the two flags given (cell)
    nm = {'EnableHardware', 'EnableMotive'};     % their names (cell of char)
    for k = 1:2
        v = fl{k};
        if ~(isnumeric(v) && isscalar(v) && isfinite(v))
            error('motive_cal_check:badflag', '%s must be a finite numeric scalar; run refused.', nm{k});
        end
    end
    if ~(EnableHardware ~= 0 && EnableMotive ~= 0)
        return                                   % not a camera run on the robot: nothing to check
    end
else
    error('motive_cal_check:nargs', 'call motive_cal_check() (base workspace) or motive_cal_check(EnableHardware, EnableMotive, motive_source, motive_cal)');
end
if motive_source_check(motive_source) == 1
    return                                       % 'emulator' (mode 2): no lab frame needed (F7)
end
cal = motive_cal;                                % camera frame calibration struct (see define_constants)
if ~(isstruct(cal) && isscalar(cal) && all(isfield(cal, {'R', 't', 'scale', 'measured'})))
    error('motive_cal_check:nocal', ['EnableHardware = 1, EnableMotive = 1 and motive_source = ''optitrack'' but motive_cal ' ...
          '(fields R, t, scale, measured) is not defined; run define_constants. Run refused.']);
end
measured = isscalar(cal.measured) && (islogical(cal.measured) || isnumeric(cal.measured)) && isequal(cal.measured, true);   % calibration entered (logical)
if ~measured
    error('motive_cal_check:unmeasured', ['EnableHardware = 1, EnableMotive = 1 and motive_source = ''optitrack'' but ' ...
          'motive_cal.measured is false: the camera-to-floor transform (motive_cal.R, .t, .scale) has not been measured ' ...
          '(Part C lab visit, docs/lab/PART_C_LAB_VISIT.md). The controller would take the robot to be wherever the Motive origin and axes put it. Enter ' ...
          'the measured calibration in lab_settings.m (the TA file) and set motive_cal.measured = true, or run with ' ...
          'run_mode 3 (fake camera). Run refused.']);
end
R = cal.R;  t = cal.t;  s = cal.scale;           % rotation (-), translation (m), scale (m per Motive unit)
okR = isnumeric(R) && isequal(size(R), [3 3]) && all(isfinite(R(:))) && norm(R.' * R - eye(3)) <= 1e-6 && det(R) > 0;
okT = isnumeric(t) && isequal(size(t), [3 1]) && all(isfinite(t));
okS = isnumeric(s) && isscalar(s) && isfinite(s) && s > 0;
if ~(okR && okT && okS)
    error('motive_cal_check:badcal', ['motive_cal.measured is true but the calibration is malformed (R 3x3 rotation %d, ' ...
          't finite 3x1 %d, scale finite > 0 %d). Run refused.'], okR, okT, okS);
end
end
