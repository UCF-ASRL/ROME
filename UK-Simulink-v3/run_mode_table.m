function M = run_mode_table(n)
%RUN_MODE_TABLE  The author's four run modes (focus plan F12, 19 Nov 2026): run mode -> run flags.
%   M = run_mode_table(n) returns the flags for run mode n (1, 2, 3 or 4) as a struct:
%     M.n               the run mode (double, 1..4)
%     M.EnableHardware  0 simulate only | 1 command the real robot (flag)
%     M.EnableMotive    1 in every mode: the base pose reaches the controller through the camera pipeline (flag)
%     M.motive_source   'emulator' (the "fake camera": the model's own base pose) | 'optitrack' (the real cameras) (char)
%     M.arm_freeze      1 joint command held at the parked start posture | 0 the controller's joint command (flag)
%     M.banner          one line naming the mode in the author's words (char)
%   Any n that is not one of the numbers 1, 2, 3, 4 (non-numeric, non-scalar, complex, NaN, Inf, 2.5, '2', 0, 5,
%   logical true, ...) errors with id run_mode_table:bad, and the message lists the four modes.
%
%   THIS IS THE ONLY PLACE THE MAPPING LIVES. define_constants.m derives its four flags from it, set_run_mode.m
%   validates through it, and the tests read it. Do not copy these numbers anywhere else.
%
%   The modes, the author's words of 19 Nov 2026 (verbatim, typo kept):
%     "1. Simulation only (end-effector dynamics sim, pose data from sim)
%      2. ground vehicle moving, frozen arm, camera off (pose data from sim)
%      3. ground vehicle moving, moving arm, camera off (pose data from sim)
%      4. ground vehicle movinb, moving arm, camera on"
%   and "every mode I mentioned the camera off, it should wun the "fake camera" emulator"; mode 1 "Through fake
%   camera". Flags: docs/superpowers/plans/2026-11-19-case1-three-modes.md, THE FOUR MODES table.
%   The banners are those words with "moving" spelled correctly; "(pose data from sim)" of modes 2 and 3 is
%   printed as "(fake camera)" (the controller's example banner) and mode 4 adds "(OptiTrack)". Those two
%   parentheticals are Claude's wording, for the author to confirm. Mode 1's arm_freeze = 1 is the file default;
%   it has no effect on the robot because nothing is sent to it (EnableHardware = 0).

% One row per mode: n | EnableHardware | EnableMotive | motive_source | arm_freeze | banner text after "MODE n: "
T = {
    1, 0, 1, 'emulator',  1, 'Simulation only (end-effector dynamics sim, pose data from sim)';
    2, 1, 1, 'emulator',  1, 'ground vehicle moving, frozen arm, camera off (fake camera)';
    3, 1, 1, 'emulator',  0, 'ground vehicle moving, moving arm, camera off (fake camera)';
    4, 1, 1, 'optitrack', 0, 'ground vehicle moving, moving arm, camera on (OptiTrack)';
    };

ok = isnumeric(n) && isscalar(n) && isreal(n) && isfinite(n) && any(double(n) == [T{:,1}]);   % valid run mode (logical)
if ~ok
    lines = cellfun(@(k, w) sprintf('  MODE %d: %s', k, w), T(:,1), T(:,6), 'UniformOutput', false);   % 4x1 banner lines (cell of char)
    error('run_mode_table:bad', '%s', sprintf(['run_mode must be one of the numbers 1, 2, 3, 4 (got %s). ' ...
        'The four modes:\n%s'], describe(n), strjoin(lines.', newline)));
end
r = find([T{:,1}] == double(n));   % table row (index)
M = struct('n', T{r,1}, 'EnableHardware', T{r,2}, 'EnableMotive', T{r,3}, 'motive_source', T{r,4}, ...
           'arm_freeze', T{r,5}, 'banner', sprintf('MODE %d: %s', T{r,1}, T{r,6}));
end

function s = describe(v)
%DESCRIBE  Short text naming the refused value, for the error message (char).
if ischar(v) && size(v, 1) <= 1
    s = sprintf('''%s'' (char)', v);
elseif isstring(v) && isscalar(v)
    s = sprintf('"%s" (string)', v);
elseif (isnumeric(v) || islogical(v)) && numel(v) <= 8 && ismatrix(v)
    s = sprintf('%s (%s)', mat2str(v), class(v));
else
    s = sprintf('a %s of size %s', class(v), mat2str(size(v)));
end
end
