% Specify the COM port used by the HC-05. Replace 'COMX' with the actual port.
comPort = 'COM9'; % e.g., 'COM3'
baudRate = 9600;  % Ensure the baud rate matches the HC-05's configuration

% Create a serial port object
device = serialport(comPort, baudRate);
configureTerminator(device, "CR/LF");

% Check if the port is connected
if isvalid(device)
    disp("Connected to the HC-05 module.");
else
    error("Failed to connect to HC-05.");
end

% Define the blink sequence (1 for ON, 0 for OFF)
blinkSequence = '1 0 1 0 1 0 1 0 1 0 1 0 1 0 1';  % Example: blink ON, OFF, ON, OFF, ON

% Send the blink sequence as a string to Arduino
write(device, blinkSequence, "char");

% Optionally, read a response from Arduino (if applicable)
pause(14);  % Let Arduino finish sending

% Check if any bytes are available
if device.NumBytesAvailable > 0
    % Read and display all lines in the buffer
    while device.NumBytesAvailable > 0
        line = readline(device);
        disp("Response from Arduino: " + line);
    end
else
    disp("No response received.");
end

% Close the serial connection when done
clear device;
