function testLimitSwitches()
    % =====================================================================
    % ROME Limit Switch Hardware Tester
    % =====================================================================
    ESP32_IP = "192.168.4.1";
    PORT = 3333;

    %% 1. Build the UI
    fig = uifigure('Name', 'ROME Limit Switch Diagnostics', ...
                   'Position', [200, 200, 600, 200], ...
                   'Color', [0.15 0.15 0.15]);
               
    uilabel(fig, 'Position', [20, 150, 560, 30], ...
            'Text', 'Press switches on the physical hardware to test connections.', ...
            'FontColor', 'white', 'FontSize', 14, 'HorizontalAlignment', 'center');

    % Create 6 LEDs and labels (1 to 6 for the user, mapping to 0 to 5 on Teensy)
    leds = gobjects(1, 6);
    for i = 1:6
        x_pos = 50 + (i-1)*90;
        
        leds(i) = uilamp(fig, 'Position', [x_pos+10, 80, 40, 40], ...
                         'Color', [0.3 0 0]); % Dark red (unpressed)
                    
        uilabel(fig, 'Position', [x_pos, 50, 60, 22], ...
                'Text', sprintf('Joint %d', i), ...
                'FontColor', 'white', 'HorizontalAlignment', 'center', ...
                'FontWeight', 'bold');
    end

    %% 2. Connect to Hardware
    try
        c = tcpclient(ESP32_IP, PORT, "ConnectTimeout", 5);
    catch ME
        uialert(fig, sprintf('Failed to connect to %s:%d\n%s', ESP32_IP, PORT, ME.message), 'Connection Error');
        return;
    end
    
    % Ensure clean disconnect when window is closed
    cleanupObj = onCleanup(@() cleanExit(c));
    
    pause(0.5);
    flush(c); 
    
    % Command the Teensy into the new LIMIT_TEST state
    write(c, "TEST_LIMITS" + newline, "char");
%% 3. Live Monitoring Loop
    prevState = zeros(1, 6); % Track state so we only print on changes

    while isvalid(fig)
        % --- DRAIN THE BUFFER ---
        % Process ALL queued messages instantly before pausing
        while c.NumBytesAvailable > 0
            rawMsg = strtrim(readline(c));
            
            % Look for our new LIMITS packet (Format: LIMITS,s0,s1,s2,s3,s4,s5)
            if startsWith(rawMsg, "LIMITS,")
                parts = split(rawMsg, ",");
                if numel(parts) == 7
                    for i = 1:6
                        isPressed = str2double(parts(i+1));
                        
                        % --- Console Print Logic ---
                        if isPressed == 1 && prevState(i) == 0
                            fprintf('Joint %d limit switch PRESSED\n', i);
                        elseif isPressed == 0 && prevState(i) == 1
                            fprintf('Joint %d limit switch RELEASED\n', i);
                        end
                        prevState(i) = isPressed; % Update history
                        
                        % --- UI Update Logic ---
                        if isPressed == 1
                            leds(i).Color = [0 1 0]; % Bright Green (Pressed)
                        else
                            leds(i).Color = [0.3 0 0]; % Dark Red (Unpressed)
                        end
                    end
                end
            end
        end
        
        % Small pause to let the UI redraw and prevent MATLAB from locking up
        pause(0.02); 
    end
end % <--- This was the missing 'end' for the main function!

function cleanExit(c)
    % Runs automatically when the figure is closed
    if isvalid(c)
        % Send STOP_ALL to safely exit the TEST state on the Teensy
        write(c, "STOP_ALL" + newline, "char");
        pause(0.1);
        flush(c);
        clear c;
        fprintf('Diagnostics complete. Socket closed.\n');
    end
end