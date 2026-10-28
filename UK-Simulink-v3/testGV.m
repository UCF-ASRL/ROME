function testGV()
    ESP32_IP = "192.168.4.1";
    PORT = 3333;

    fprintf('Connecting to ESP32...\n');
    c = tcpclient(ESP32_IP, PORT, "ConnectTimeout", 5);
    pause(0.5); flush(c);
    
    % 1. Send Handshake
    fprintf('Sending START_GV...\n');
    write(c, "START_GV" + newline, "char");
    pause(0.5);
    
    fprintf('Commanding 15.0 RPM to all wheels for 5 seconds...\n');
    t0 = tic;
    lastSend = tic;
    megaAlive = false;
    
    % 2. Main Loop: Send continuous commands and read telemetry
    while toc(t0) < 5
        
        % Feed the Mega's Watchdog every 100ms (10 Hz)
        if toc(lastSend) > 0.1
            write(c, "15.0,15.0,15.0,15.0" + newline, "char");
            lastSend = tic;
        end
        
        % Read Telemetry
        while c.NumBytesAvailable > 0
            rawMsg = strtrim(readline(c));
            if startsWith(rawMsg, "GV,")
                fprintf('[MEGA RX] %s\n', rawMsg);
                megaAlive = true;
            end
        end
        pause(0.02);
    end
    
    % 3. Diagnostics and Shutdown
    if ~megaAlive
        fprintf('\n!!! HARDWARE WARNING: No telemetry received from Mega !!!\n');
        fprintf('Check that ESP32 TX/RX is wired to Mega RX1(19) and TX1(18), and grounds are shared.\n\n');
    end
    
    % Safely stop the motors before closing
    write(c, "0.0,0.0,0.0,0.0" + newline, "char");
    clear c;
    fprintf('Test Complete.\n');
end