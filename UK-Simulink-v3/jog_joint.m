function jog_joint(j, delta_deg)
%JOG_JOINT  Move ONE arm joint by delta_deg (firmware degrees) and report.
%
%   jog_joint(2, 10)   moves J2 by +10 deg from where the firmware says it is,
%   with zero wheel speed, and prints the reading before and after, so you
%   can see (a) which way the arm physically went and (b) whether the
%   reported angle followed the command. Use it after calibrateROMEArm, one
%   joint at a time, to measure arm_sign / arm_offset_deg (GETTING_STARTED.md
%   step 3) and to catch a joint whose encoder counts against its steps.
%
%   The target is capped to the firmware range with the margin from
%   define_constants, so the jog cannot ask for a limit.
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, 'Optitrack'));
evalc('define_constants');
assert(isscalar(j) && j >= 1 && j <= 6, 'j must be 1..6');

c = tcpclient("192.168.4.1", 3333, "ConnectTimeout", 10);
cleanupObj = onCleanup(@() delete_socket(c));
pause(0.3);  flush(c);

before = read_arm(c, 2.0);
if any(isnan(before)), error('no ARM telemetry; is the arm calibrated and the ESP32 connected?'); end
target = before;  target(j) = target(j) + delta_deg;
[target, hit] = arm_clamp(target, arm_fw_switch, arm_fw_other, arm_fw_margin_deg);
if hit(j), fprintf('  (target capped to the firmware range: %.1f)\n', target(j)); end
cmd = sprintf('ROME,0,0,0,0,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', target);
fprintf('J%d: reading %.1f -> commanding %.1f   [TX] %s', j, before(j), target(j), cmd);

% send, then keep the watchdog fed while the joint moves and settles
t0 = tic;  lastSend = tic;  lastRead = before;  lastChange = tic;
while toc(t0) < 15
    if toc(lastSend) > 0.2, write(c, uint8(char(cmd)));  lastSend = tic; end
    v = read_arm(c, 0.1);
    if ~any(isnan(v))
        if any(abs(v - lastRead) > 0.3), lastChange = tic; end
        lastRead = v;
    end
    if toc(t0) > 1.0 && toc(lastChange) > 1.5, break; end
end
after = lastRead;
fprintf('J%d: reading after %.1f  (moved %+.1f, asked %+.1f)\n', j, after(j), after(j) - before(j), target(j) - before(j));
others = setdiff(1:6, j);
if any(abs(after(others) - before(others)) > 1.0)
    fprintf('  other joints moved too: %s\n', mat2str(round(after - before, 1)));
end
if j == 4
    fprintf('  J4 has no encoder: the reading is the command. Judge by eye only.\n');
elseif abs((after(j) - before(j)) - (target(j) - before(j))) < 2.0
    fprintf('  reading followed the command. Now: did the ARM physically move the way\n');
    fprintf('  the model predicts for +%.0f deg on model joint %d with arm_sign(%d) = %+d?\n', ...
            delta_deg, j, j, arm_sign(j));
    fprintf('  (ik9_fk at arm_home with and without the jog; GETTING_STARTED.md step 3)\n');
else
    fprintf('  reading did NOT follow the command: encoder loop problem on J%d (Teensy side).\n', j);
end
end

function v = read_arm(c, timeout_s)
v = NaN(1,6);  t = tic;
while toc(t) < timeout_s
    if c.NumBytesAvailable > 0
        msg = strtrim(readline(c));
        if startsWith(msg, "ARM,")
            parts = split(msg, ",");
            if numel(parts) >= 8, v = str2double(parts(2:7)).'; end
        end
    else
        pause(0.01);
    end
end
end

function delete_socket(c)
if isvalid(c), flush(c); clear c; end
end
