function returnHomeROMEArm()
    % =========================================================================
    % ROME Arm: Safe Return to Home Position
    % =========================================================================
    ESP32_IP = "192.168.4.1";
    PORT = 3333;
    TIMEOUT_SECONDS = 30;

    %% 1. Load Configuration
    fprintf('Loading home position from define_constants...\n');
    try
        here = fileparts(mfilename('fullpath'));
        addpath(here, fullfile(here, 'Optitrack'));
        evalc('define_constants');
        
        homePosDeg = arm_sign(:) .* rad2deg(q0_9dof(4:9)) + arm_offset_deg(:);
        
        if exist('arm_fw_switch', 'var')
            [homePosDeg, ~] = arm_clamp(homePosDeg, arm_fw_switch, arm_fw_other, arm_fw_margin_deg);
        end
        homePosDeg = homePosDeg(:).'; % Ensure row vector
        
    catch ME
        error('Failed to load define_constants. Make sure you are in the correct directory.\nError: %s', ME.message);
    end
    
    fprintf('Target Home (Firmware Degrees): %s\n', mat2str(round(homePosDeg, 1)));

    %% 2. Connect
    fprintf('Connecting to %s:%d...\n', ESP32_IP, PORT);
    try
        c = tcpclient(ESP32_IP, PORT, "ConnectTimeout", 5);
    catch ME
        error('Failed to establish TCP connection: %s', ME.message);
    end
    
    cleanupObj = onCleanup(@() cleanExit(c));
    pause(0.5); flush(c);

    %% 3. Verify Calibration State
    fprintf('Checking arm calibration state...\n');
    isReady = false;
    armStateStr = 'UNKNOWN';
    t0 = tic;

    while toc(t0) < 3 && ~isReady
        if c.NumBytesAvailable > 0
            rawMsg = strtrim(readline(c));
            if startsWith(rawMsg, "ARM,")
                parts = split(rawMsg, ",");
                if numel(parts) >= 8
                    armStateStr = strtrim(parts{8});
                    if strcmpi(armStateStr, 'READY')
                        isReady = true;
                    end
                end
            end
        end
        pause(0.02);
    end

    if ~isReady
        error('Arm rejected move: Current state is %s. You must run calibrateROMEArm first.', armStateStr);
    end

    %% 4. Execute Move
    fprintf('Arm is calibrated. Moving to home...\n');
    parkCmd = sprintf('ROME,0,0,0,0,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', homePosDeg);
    
    tMove = tic;
    lastSend = tic;
    reached = false;

    while toc(tMove) < TIMEOUT_SECONDS
        
        % Feed the Watchdog at 10 Hz
        if toc(lastSend) > 0.1
            write(c, parkCmd, "char");
            lastSend = tic;
        end

        % Drain buffer and check distance to target
        while c.NumBytesAvailable > 0
            rawMsg = strtrim(readline(c));
            if startsWith(rawMsg, "ARM,")
                parts = split(rawMsg, ",");
                if numel(parts) >= 8
                    actualAngles = str2double(parts(2:7)).';
                    
                    % Joint 4 is open loop, skip it in the error threshold check
                    errorCheck = abs(actualAngles - homePosDeg);
                    errorCheck(4) = 0; 
                    
                    if max(errorCheck) < 1.0 % All tracked joints within 1 degree
                        reached = true;
                    end
                end
            end
        end
        
        if reached
            break;
        end
        
        pause(0.02);
    end

    if reached
        fprintf('Arm successfully reached the home position.\n');
    else
        warning('Movement timed out after %d seconds before reaching home.', TIMEOUT_SECONDS);
    end
end

function cleanExit(c)
    if isvalid(c)
        % Ensure the arm stops exactly where it is when the script closes
        write(c, "STOP_ALL" + newline, "char");
        pause(0.1);
        flush(c);
        clear c;
        fprintf('Socket disconnected. Arm locked.\n');
    end
end