function use_emulator = motive_source_check(src)
%MOTIVE_SOURCE_CHECK  Validate the camera source motive_source and say whether it is the emulator (focus plan F7).
%   use_emulator = MOTIVE_SOURCE_CHECK(src)
%     src           motive_source of define_constants.m: 'emulator' or 'optitrack' (char row or string scalar)
%     use_emulator  1 for 'emulator', 0 for 'optitrack' (double, so a Constant block can use it directly)
%   Anything else is refused: error motive_source_check:invalid, run refused. Exact match, case included.
%   Meaning (focus plan docs/superpowers/plans/2026-11-19-case1-three-modes.md, mode 2 definition, 19 Nov 2026):
%     'emulator'   the base pose the controller reads with EnableMotive = 1 is the model's own predicted base pose
%                  (StateDelay), passed through the same camera pipeline as the real cameras (floor_to_motive, then
%                  MotiveFloor = motive_to_floor + yaw_unwrap). No OptiTrack, no socket, no lab frame needed. Mode 2.
%     'optitrack'  the real MotiveData reader (on the robot only; refused there until motive_cal is measured). Mode 3.
%   Called by define_constants.m (every run of the file, so an invalid value stops it before arm_freeze_user is
%   set), by motive_cal_check.m, and by two Constant blocks of ROME_9DOF.slx, evaluated at every compile
%   (tools/wire_motive_emulator.m): Enabled Subsystem/PoseSelect and the reader gate MotiveReaderEnable.
%   Tests: tests/test_motive_source.m check (v).
ok = (ischar(src) && (isrow(src) || isempty(src))) || (isstring(src) && isscalar(src));   % text of one value (logical)
if ok
    s = char(src);                            % the value as char (char)
    ok = any(strcmp(s, {'emulator', 'optitrack'}));
end
if ~ok
    if ischar(src) || isstring(src), shown = ['''' char(strjoin(string(src), ' ')) '''']; else, shown = sprintf('a %s', class(src)); end   % value as printed (char)
    error('motive_source_check:invalid', ['motive_source is %s; allowed: ''emulator'' (the model''s predicted base pose ' ...
          'through the camera pipeline, modes 1-3) or ''optitrack'' (the real cameras, mode 4). It follows run_mode: ' ...
          'set the mode with rome_mode(n). Run refused.'], shown);
end
use_emulator = double(strcmp(s, 'emulator'));
end
