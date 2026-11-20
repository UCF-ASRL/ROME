function jogROMEArm()
    % =====================================================================
    % ROME 6-DOF Arm Manual Jogger
    % =====================================================================
    ESP32_IP = "192.168.4.1";
    PORT = 3333;

    % Internal State
    % Every arm command goes through jog_rome_send (hw audit row 24, fixes5): capped to the firmware ranges of
    % define_constants (arm_clamp, as ROMECommand / jog_joint / calibrateROMEArm), and REFUSED until the first ARM
    % telemetry line has been read (synced), so the zeros below are never sent (J2 = 0 is a firmware range end).
    lim = jog_rome_send('limits');   % firmware ranges (struct fw_switch, fw_other, margin; deg)
    synced = false;                  % true once an ARM telemetry line with 6 finite angles has been read
    targetAngles = zeros(1, 6);
    actualAngles = zeros(1, 6);
    stepSize = 5.0; % Default jog step in degrees
    lastSendTime = tic;
    armStateStr = 'UNKNOWN';

    %% 1. Build the UI
    fig = uifigure('Name', 'ROME Manual Jog Control', ...
                   'Position', [200, 200, 500, 450], ...
                   'Color', [0.9 0.9 0.9]);

    % Status Header
    lblStatus = uilabel(fig, 'Position', [20, 410, 460, 22], ...
        'Text', 'Connecting to arm...', ...
        'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'FontSize', 14);

    % Step Size Selector
    uilabel(fig, 'Position', [140, 370, 100, 22], 'Text', 'Jog Step (deg):', 'FontWeight', 'bold');
    uidropdown(fig, 'Position', [240, 370, 80, 22], ...
        'Items', {'1.0', '5.0', '10.0', '25.0'}, 'Value', '5.0', ...
        'ValueChangedFcn', @(src, ~) updateStepSize(src));

    % Joint UI Array
    btnMinus = gobjects(1,6);
    btnPlus  = gobjects(1,6);
    lblAct   = gobjects(1,6);
    lblTgt   = gobjects(1,6);

    for i = 1:6
        y = 320 - (i-1)*50;
        
        % Joint Label
        uilabel(fig, 'Position', [30, y, 60, 30], ...
            'Text', sprintf('Joint %d', i), 'FontWeight', 'bold', 'FontSize', 14);
        
        % Minus Button
        btnMinus(i) = uibutton(fig, 'Position', [100, y, 40, 30], ...
            'Text', '-', 'FontSize', 16, 'FontWeight', 'bold', ...
            'ButtonPushedFcn', @(~, ~) jogJoint(i, -1));
        
        % Target & Actual Labels
        lblTgt(i) = uilabel(fig, 'Position', [155, y, 110, 30], ...
            'Text', 'Target: 0.0°', 'FontSize', 12);
        lblAct(i) = uilabel(fig, 'Position', [275, y, 110, 30], ...
            'Text', 'Actual: 0.0°', 'FontSize', 12, 'FontColor', [0 0.4 0]);
            
        % Plus Button
        btnPlus(i) = uibutton(fig, 'Position', [390, y, 40, 30], ...
            'Text', '+', 'FontSize', 16, 'FontWeight', 'bold', ...
            'ButtonPushedFcn', @(~, ~) jogJoint(i, 1));
    end

    %% 2. Connect and Sync
    try
        c = tcpclient(ESP32_IP, PORT, "ConnectTimeout", 5);
    catch ME
        lblStatus.Text = 'Connection Failed!';
        lblStatus.FontColor = 'red';
        uialert(fig, sprintf('Failed to connect to %s:%d', ESP32_IP, PORT), 'Error');
        return;
    end
    cleanupObj = onCleanup(@() cleanExit(c));
    pause(0.5); flush(c);

    % Sync with hardware to prevent snapping to zero
    lblStatus.Text = 'Syncing starting positions...';
    drawnow;
    t0 = tic;
    
    while toc(t0) < 3 && ~synced
        if c.NumBytesAvailable > 0
            rawMsg = strtrim(readline(c));
            if startsWith(rawMsg, "ARM,")
                parts = split(rawMsg, ",");
                if numel(parts) >= 8 && all(isfinite(str2double(parts(2:7))))
                    vals = str2double(parts(2:7));
                    targetAngles = vals(:)';
                    actualAngles = vals(:)';
                    armStateStr = strtrim(parts{8});
                    synced = true;
                end
            end
        end
    end

    if ~synced
        lblStatus.Text = 'No ARM telemetry: nothing is sent until the arm reports its angles.';
        lblStatus.FontColor = [0.8 0.4 0]; % Orange
    else
        lblStatus.Text = sprintf('Connected | Arm State: %s', armStateStr);
        lblStatus.FontColor = [0 0.5 0]; % Green
    end
    updateLabels();

    %% 3. UI Callbacks
    function updateStepSize(src)
        stepSize = str2double(src.Value);
    end

    function jogJoint(idx, dir)
        if ~synced, return; end      % no target before the arm's own reading
        targetAngles(idx) = targetAngles(idx) + (dir * stepSize);
        targetAngles = arm_clamp(targetAngles, lim.fw_switch, lim.fw_other, lim.margin);   % show the capped target
        updateLabels();
    end

    function updateLabels()
        for j = 1:6
            lblTgt(j).Text = sprintf('Target: %.1f°', targetAngles(j));
            lblAct(j).Text = sprintf('Actual: %.1f°', actualAngles(j));
        end
    end

    %% 4. Main Execution Loop
    while isvalid(fig)
        
        % 10 Hz Command Transmission to satisfy Watchdog
        if toc(lastSendTime) > 0.1
            try
                targetAngles = jog_rome_send(c, targetAngles, synced, lim);   % capped; refused before telemetry
            catch ME
                if ~strcmp(ME.identifier, 'jog_rome_send:notelemetry'), rethrow(ME); end
                lblStatus.Text = 'No ARM telemetry: nothing sent. Calibrate the arm or call the TA.';
                lblStatus.FontColor = 'red';
            end
            lastSendTime = tic;
        end

        % Process all incoming telemetry buffers
        updated = false;
        while c.NumBytesAvailable > 0
            rawMsg = strtrim(readline(c));
            if startsWith(rawMsg, "ARM,")
                parts = split(rawMsg, ",");
                if numel(parts) >= 8
                    actualAngles = str2double(parts(2:7))';
                    armStateStr = strtrim(parts{8});
                    updated = true;
                    if ~synced && all(isfinite(actualAngles))   % first telemetry: start the target at the reading
                        targetAngles = actualAngles;  synced = true;
                    end
                end
            end
        end
        
        if updated
            updateLabels();
            lblStatus.Text = sprintf('Connected | Arm State: %s', armStateStr);
        end

        pause(0.02);
    end

end % End of main function

function cleanExit(c)
    if isvalid(c)
        % Lock the arm in place before closing the socket
        write(c, "STOP_ALL" + newline, "char");
        pause(0.1);
        flush(c);
        clear c;
        fprintf('Jogger closed. Arm held at current position.\n');
    end
end