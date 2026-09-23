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
[homePosDeg, hit] = arm_clamp(homePosDeg, arm_fw_lo, arm_fw_hi, arm_fw_margin_deg);
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
 
%% Request Calibration
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
 
%% Send HOME Position, then park the arm there
% CAL_ARM drove the joints to the firmware's own home. HOME records the
% start posture for the next calibration and, since Keanu's 22 Sep 2026
% firmware, also sets the target angles, so the arm moves there through
% ValidateTraj. The park command that follows is the same target sent the
% way the model sends it (ROME with zero wheel speed and the six angles,
% split by the ESP32 into the GV and ARM lines) and is what keeps the
% Teensy's 500 ms watchdog from holding the arm short of the posture.
homeCmd = sprintf('HOME,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', homePosDeg);
parkCmd = sprintf('ROME,0,0,0,0,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', homePosDeg);

fprintf('Sending HOME target configuration...\n');
fprintf('[TX] %s', homeCmd);
flush(c);
write(c, homeCmd, "char");
pause(0.5);

fprintf('Parking the arm at the start posture...\n');
fprintf('[TX] %s', parkCmd);
write(c, parkCmd, "char");
pause(0.5);
fprintf(['Check the arm: elbow up and forward, forearm pointing down, tool\n' ...
         'at the floor in front of the base. If it did not move, the firmware\n' ...
         'rejected an angle: see GETTING_STARTED.md step 3.\n']);
 
%% Query Status
fprintf('Verifying arm system state...\n');
 
% Flush incoming telemetry queue so STATUS read gets the fresh reply
flush(c);
write(c, "STATUS" + newline, "char");
 
statusReceived = false;
statusTimer = tic;
 
while toc(statusTimer) < 3.0
    if c.NumBytesAvailable > 0
        msg = strtrim(readline(c));
        % Telemetry packets end with state string ("ARM,...,READY") or standalone "READY"
        if contains(msg, "READY") && ~contains(msg, "CALIBRATING") && ~contains(msg, "UNCALIBRATED")
            statusReceived = true;
            fprintf('[RX Status] %s\n', msg);
            break;
        end
    end
    pause(0.02);
end
 
if statusReceived
    fprintf('Arm status verified: READY.\n');
else
    warning('READY status not explicitly confirmed by Teensy.');
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