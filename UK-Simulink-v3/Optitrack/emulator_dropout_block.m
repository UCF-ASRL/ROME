function [pose7_out, valid] = emulator_dropout_block(pose7, t, win)
%EMULATOR_DROPOUT_BLOCK  Script of the MATLAB Function block EmuDropout in ROME_9DOF.slx (focus plan F6, test injection).
%   tools/wire_camera_guard.m copies THIS FILE, verbatim, into the chart (CLAUDE.md section 27).
%   Between MotiveEmulator and the Enabled Subsystem. Outside every window it passes the emulated pose unchanged with
%   valid = 1 (no arithmetic: modes 1-3 unchanged). Inside a window it mimics MotiveData on a lost frame: the last pose
%   it passed (held), or NaN(7,1) when none was passed yet, with valid = 0. So the camera guard can be tested on the
%   emulator path (mode 3, fake robot) exactly as the real reader would present a dropout.
%     pose7   7x1 MotiveEmulator output (Motive units, -)
%     t       simulation time (s), Clock9
%     win     k x 2 windows [t_start t_end) (s), rome_limit('emulate_dropout'): the base variable motive_emulate_dropout
%             when a test sets it, else [-1 -1] (no window). Not a student setting.
%     valid   1 outside the windows, 0 inside (flag)
persistent last
if isempty(last)
    last = NaN(7, 1);                       % no pose passed yet: MotiveData outputs NaN before its first frame
end
inwin = false;
for k = 1:size(win, 1)
    if t >= win(k, 1) && t < win(k, 2)
        inwin = true;
    end
end
if inwin
    pose7_out = last;
    valid = 0;
else
    pose7_out = pose7;
    valid = 1;
    last = pose7;
end
end
