function calibrateROMEArm()

% =========================================================================
% ROME Arm Calibration Utility (Production Version)
% =========================================================================

%% Configuration
ESP32_IP = "192.168.4.1";
PORT     = 3333;

TIMEOUT_SECONDS = 120;

% The arm is parked where the model starts, in FIRMWARE degrees, from the
% same define_constants the model runs on: q0_9dof(4:9) is the start
% posture the card prints, arm_sign / arm_offset_deg the model-to-firmware
% map, arm_fw_* the ranges the command is capped to. Nothing is typed here.
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, 'Optitrack'));
evalc('define_constants');
homePosDeg = arm_sign(:) .* rad2deg(q0_9dof(4:9)) + arm_offset_deg(:);
[homePosDeg, hit] = arm_clamp(homePosDeg, arm_fw_switch, arm_fw_other, arm_fw_margin_deg);
homePosDeg = homePosDeg(:);
if any(hit)
    error(['the start posture maps outside the firmware range on joint(s) %s: ' ...
           'fw %s. Fix arm_home / arm_sign / arm_offset_deg in define_constants ' ...
           'and re-run gate_all before calibrating.'], mat2str(find(hit)), ...
           mat2str(round(homePosDeg.', 1)));
end
fprintf('\nScenario %d start posture, firmware degrees: %s\n', scenario, ...
        mat2str(round(homePosDeg.', 1)));
fprintf('Model degrees: %s\n', mat2str(round(rad2deg(q0_9dof(4:9)).', 1)));

fprintf('\n===========================================\n');
fprintf('ROME ARM CALIBRATION\n');
fprintf('===========================================\n');

%% Connect
fprintf('Connecting to %s:%d ...\n', ESP32_IP, PORT);

try
    c = tcpclient(ESP32_IP, PORT, "ConnectTimeout", 10);
catch ME
    error('Failed to establish TCP connection: %s', ME.message);
end

% Guarantee socket cleanup if script fails or is interrupted
cleanupObj = onCleanup(@() cleanupSocket(c));

pause(0.5);
flush(c); % Clear stale startup frames
fprintf('Connected.\n');

%% 1. HOME first, so the calibration itself ends at the start posture
% Keanu's procedure (22 Sep 2026): the Teensy's fullCalibrate drives every
% joint to its limit switch and then to homePosDeg, so sending HOME before
% CAL_ARM makes calibration finish AT the scenario's start posture, one
% motion. The values are already capped to the firmware ranges above; the
% calibration path itself does not validate them.
homeCmd = sprintf('HOME,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', homePosDeg);
parkCmd = sprintf('ROME,0,0,0,0,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', homePosDeg);
fprintf('Sending HOME target configuration...\n');
fprintf('[TX] %s', homeCmd);
flush(c);
write(c, homeCmd, "char");
pause(0.2);

%% 2. Request Calibration
fprintf('Sending CAL_ARM...\n');
write(c, "CAL_ARM" + newline, "char");

tic;
armReady = false;

while toc < TIMEOUT_SECONDS
    if c.NumBytesAvailable > 0
        % Read available stream data
        rawMsg = strtrim(readline(c));
        % Check for hardware handshake complete string
        if contains(rawMsg, "ARM_READY")
            armReady = true;
            fprintf('[RX] %s\n', rawMsg);
            break;
        end
    end
    pause(0.02);
end

if ~armReady
    error('Calibration timeout (%d s). ARM_READY acknowledgment not received.', TIMEOUT_SECONDS);
end

fprintf('Calibration routine finished on hardware.\n');

%% 3. Park command: the same target, the way the model sends it
% Keeps the 500 ms firmware watchdog from holding the arm short of the
% posture while we watch, and confirms the ESP32 split (GV / ARM lines).
fprintf('Parking the arm at the start posture...\n');
fprintf('[TX] %s', parkCmd);
write(c, parkCmd, "char");

%% 4. Watch the telemetry settle, then judge each joint
% The ARM, line comes at 20 Hz. Read it until the angles stop changing (or
% 25 s). CAVEAT: at the end of fullCalibrate the firmware FORCES the
% reported angles to homePosDeg and only refreshes them from the encoders
% once a joint is given a target it is not already at. So a reading equal
% to the target proves nothing by itself; a reading that differs does.
% Confirm the posture by eye, and use jog_joint(j, delta) for the
% direction check on any joint in doubt.
fprintf('Watching the arm settle (up to 25 s)...\n');
lastRead = NaN(1,6);  lastChange = tic;  settleTimer = tic;  statusReceived = false;
lastSend = tic;
while toc(settleTimer) < 25
    if toc(lastSend) > 0.2
        write(c, parkCmd, "char");  lastSend = tic;
    end
    if c.NumBytesAvailable > 0
        msg = strtrim(readline(c));
        if startsWith(msg, "ARM,")
            parts = split(msg, ",");
            if numel(parts) >= 8
                vals = str2double(parts(2:7)).';
                statusReceived = statusReceived || contains(parts(8), "READY");
                if any(abs(vals - lastRead) > 0.3) || any(isnan(lastRead))
                    lastChange = tic;
                end
                lastRead = vals;
            end
        end
    end
    if ~any(isnan(lastRead)) && toc(lastChange) > 1.5, break; end
    pause(0.02);
end

if any(isnan(lastRead))
    warning('no ARM telemetry received while parking; cannot judge the posture');
else
    fprintf('\n  joint   target   reading   verdict\n');
    bad = false(1,6);
    for i = 1:6
        d1 = abs(lastRead(i) - homePosDeg(i));
        if i == 4
            v = 'open loop, reading is the command (no encoder): check by eye';
        elseif d1 < 2.0
            v = 'at target (or the forced home value: check by eye)';
        else
            v = 'NOT at target  <-- this joint did not get there'; bad(i) = true;
        end
        fprintf('  J%d    %7.1f  %8.1f   %s\n', i, homePosDeg(i), lastRead(i), v);
    end
    fprintf(['\n  Expected posture: elbow up and forward, forearm pointing down, tool at\n' ...
             '  the floor about 0.34 m in front of the base axis.\n']);
    if any(bad)
        fprintf(['  DO NOT set EnableHardware = 1. Joint(s) %s did not reach the target.\n' ...
                 '  Run jog_joint(j, 10) on each of them and watch which way the arm goes.\n'], ...
                mat2str(find(bad)));
    else
        fprintf('  If the arm does not look like that, run jog_joint(j, 10) joint by joint.\n');
    end
end
if statusReceived
    fprintf('Arm status: READY.\n');
else
    warning('READY status not confirmed by Teensy.');
end

fprintf('\nROME arm calibration successfully completed.\n\n');

end

% Helper for safe TCP socket termination
function cleanupSocket(c)
    if isvalid(c)
        flush(c);
        clear c;
        fprintf('TCP Socket disconnected.\n');
    end
end
