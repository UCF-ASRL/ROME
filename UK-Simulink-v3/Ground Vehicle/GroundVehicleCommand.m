classdef GroundVehicleCommand < matlab.System

    properties (Nontunable)
        COMPort = 'COM4';
        BaudRate = 115200;
    end

    properties (Access = private)
        % Internal handle for the serial object
        DeviceHandle
        initial_run = true;
    end

    methods (Access = protected)
        % function setupImpl(obj)
        % 
        % end

        function stepImpl(obj, v, Enable)
            if Enable && obj.initial_run
                % Declare extrinsic functions for simulation-only execution
                coder.extrinsic('serialport', 'configureTerminator', 'flush', 'writeline', 'pause', 'error', 'fprintf');
                
                fprintf('Attempting to connect to Ground Vehicle on %s...\n', obj.COMPort);
                
                try
                    % Attempt to create the serial object
                    obj.DeviceHandle = serialport(obj.COMPort, obj.BaudRate);
                    
                    % Configure communication
                    configureTerminator(obj.DeviceHandle, "LF");
                    flush(obj.DeviceHandle);
                    pause(0.5); % Brief pause to let hardware stabilize
                    
                    % Send start trigger
                    writeline(obj.DeviceHandle, 'Y'); 
                    fprintf('Connection Successful!\n');
                    
                catch ME
                    % If connection fails, provide a helpful error message
                    error('Bluetooth Connection Failed on %s. Ensure the device is paired and the port is not in use. \nDetails: %s', ...
                        obj.COMPort, ME.message);
                end
                coder.extrinsic('sprintf', 'writeline');
                obj.initial_run = false;
            end

            if Enable
                % Check if handle exists before writing (safety check)
                if ~isempty(obj.DeviceHandle)
                    msg = sprintf('%.2f,%.2f,%.2f,%.2f', v(1), v(2), v(3), v(4));
                    writeline(obj.DeviceHandle, msg);
                    disp(msg)
                end
            end
        end

        function releaseImpl(obj)
            % Clean up the port when the simulation stops
            if ~isempty(obj.DeviceHandle)
                msg = sprintf('%.2f,%.2f,%.2f,%.2f', 0, 0, 0, 0); % Stop motors
                writeline(obj.DeviceHandle, msg);
                clear obj.DeviceHandle;
            end
            coder.extrinsic('clear');
        end

        % --- Essential for Sinks (Blocks with no outputs) ---
        function num = getNumOutputsImpl(~)
            num = 0;
        end
    end
end