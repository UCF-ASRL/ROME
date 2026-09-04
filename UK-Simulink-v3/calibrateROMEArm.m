function calibrateROMEArm()
 
% =========================================================================
% ROME Arm Calibration Utility (Production Version)
% =========================================================================
 
%% Configuration
ESP32_IP = "192.168.4.1";
PORT     = 3333;
 
TIMEOUT_SECONDS = 120;
 
homePosDeg = [
     0.0
    90.0 
    90.0
     1.0
     0.0
     0.0
];
 
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
 
%% Send HOME Position
homeCmd = sprintf('HOME,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', homePosDeg);
 
fprintf('Sending HOME target configuration...\n');
fprintf('[TX] %s', homeCmd);
 
% Flush buffer before sending HOME command to ensure crisp tracking
flush(c);
write(c, homeCmd, "char");
pause(0.5);
 
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