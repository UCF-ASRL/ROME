classdef ROMECommand < matlab.System
    properties (Nontunable)
        ESP32_IP   = "192.168.4.1";
        ESP32_Port = 3333;
    end
    
    properties (Access = private)
        DeviceHandle
        initial_run = true
        clamp_warned = false
        actualWheelRPM_
        actualJointDeg_
        armReady_
        lastTelemetryTime_
        residualBuffer_
        
        % Pre-allocated Workspace Variables
        has_arm_map_ = false
        arm_sign_ = ones(1,6)
        arm_offset_deg_ = zeros(1,6)
        
        arm_freeze_ = false
        arm_freeze_deg_ = zeros(1,6)
        
        has_arm_clamp_ = false
        arm_fw_switch_ = zeros(1,6)
        arm_fw_other_ = zeros(1,6)
        arm_fw_margin_deg_ = zeros(1,6)
    end
    
    methods (Access = protected)
        %==============================================================
        % SETUP (Run once at model initialization)
        %==============================================================
        function setupImpl(obj)
            obj.actualWheelRPM_ = zeros(4,1);
            obj.residualBuffer_ = "";
            obj.armReady_ = false;
            obj.lastTelemetryTime_ = tic;
            
            % Load all base workspace variables ONCE to prevent real-time lag
            if evalin('base', 'exist(''arm_home'',''var'')')
                obj.actualJointDeg_ = rad2deg(evalin('base','arm_home(:)'));
            else
                obj.actualJointDeg_ = zeros(6,1);
            end
            
            if evalin('base', 'exist(''arm_offset_deg'',''var'')')
                obj.has_arm_map_ = true;
                obj.arm_sign_ = evalin('base', 'arm_sign(:).''');
                obj.arm_offset_deg_ = evalin('base', 'arm_offset_deg(:).''');
            end
            
            % --- FORCE ARM FREEZE OFF ---
            % Automatically disable the parking brake in the workspace on startup
            assignin('base', 'arm_freeze', 0);
            obj.arm_freeze_ = false; 
            
            % We still load the parked posture just in case other scripts need it
            if evalin('base', 'exist(''arm_freeze_deg'',''var'')')
                obj.arm_freeze_deg_ = evalin('base', 'arm_freeze_deg(:).''');
            end
            
            if evalin('base', 'exist(''arm_fw_switch'',''var'')')
                obj.has_arm_clamp_ = true;
                obj.arm_fw_switch_ = evalin('base', 'arm_fw_switch');
                obj.arm_fw_other_  = evalin('base', 'arm_fw_other');
                obj.arm_fw_margin_deg_ = evalin('base', 'arm_fw_margin_deg');
            end
        end
        
        %==============================================================
        % MAIN STEP (Runs every simulation step)
        %==============================================================
        function [actualWheelRPM, actualJointDeg, armReady] = stepImpl(obj, wheelRPM, jointRad, Enable)
            % Default outputs
            actualWheelRPM = obj.actualWheelRPM_;
            actualJointDeg = obj.actualJointDeg_;
            armReady       = obj.armReady_;
            
            if coder.target('MATLAB')
                %------------------------------------------------------
                % CONNECT ONCE
                %------------------------------------------------------
                if Enable && obj.initial_run
                    disp('Connecting to ROME ESP32...');
                    obj.DeviceHandle = tcpclient(obj.ESP32_IP, obj.ESP32_Port, "ConnectTimeout", 5);
                    pause(0.5);
                    write(obj.DeviceHandle, uint8(char("START_GV" + newline)));
                    disp('ROME Connected.');
                    obj.initial_run = false;
                end
                
                %------------------------------------------------------
                % SEND COMMANDS
                %------------------------------------------------------
                if Enable && ~isempty(obj.DeviceHandle)
                    jointDeg = rad2deg(jointRad(:).');
                    
                    % 1. Model-to-firmware joint map
                    if obj.has_arm_map_
                        jointDeg = obj.arm_sign_ .* jointDeg + obj.arm_offset_deg_;
                    end
                    
                    % 2. Arm freeze override
                    if obj.arm_freeze_
                        jointDeg = obj.arm_freeze_deg_;
                    end
                    
                    % 3. Firmware clamping
                    if obj.has_arm_clamp_
                        % Using extrinsic call to prevent code gen issues with external functions
                        [jointDeg, hit] = arm_clamp(jointDeg, obj.arm_fw_switch_, obj.arm_fw_other_, obj.arm_fw_margin_deg_);
                        if any(hit) && ~obj.clamp_warned
                            warning('ROMECommand:clamp', 'arm command capped to the firmware range on joint(s) %s', mat2str(find(hit)));
                            obj.clamp_warned = true;
                        end
                    end
                    
                    msg = sprintf('ROME,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', ...
                        wheelRPM(1), wheelRPM(2), wheelRPM(3), wheelRPM(4), ...
                        jointDeg(1), jointDeg(2), jointDeg(3), jointDeg(4), jointDeg(5), jointDeg(6));
                    
                    write(obj.DeviceHandle, uint8(msg));
                end
                
                %------------------------------------------------------
                % RECEIVE TELEMETRY (Fragmentation Safe)
                %------------------------------------------------------
                if ~isempty(obj.DeviceHandle)
                    available = obj.DeviceHandle.NumBytesAvailable;
                    if available > 0
                        rawChars = char(read(obj.DeviceHandle, available, "char"));
                        obj.residualBuffer_ = obj.residualBuffer_ + string(rawChars);
                    end
                    
                    % Only process if we have a complete line (contains newline)
                    if strlength(obj.residualBuffer_) > 0 && contains(obj.residualBuffer_, newline)
                        lines = splitlines(obj.residualBuffer_);
                        
                        % The last element is either empty (ended perfectly in newline) 
                        % or an incomplete string waiting for the next packet.
                        obj.residualBuffer_ = lines(end);
                        
                        % Process all fully complete lines
                        for k = 1:(length(lines)-1)
                            line = strtrim(lines(k));
                            
                            if startsWith(line,"GV,")
                                parts = split(line,",");
                                if numel(parts) >= 9
                                    obj.actualWheelRPM_ = [str2double(parts(2)); str2double(parts(3)); str2double(parts(4)); str2double(parts(5))];
                                    obj.lastTelemetryTime_ = tic;
                                end
                                
                            elseif startsWith(line,"ARM,")
                                parts = split(line,",");
                                if numel(parts) >= 8
                                    fwDeg = [str2double(parts(2)); str2double(parts(3)); str2double(parts(4)); str2double(parts(5)); str2double(parts(6)); str2double(parts(7))];
                                    
                                    if obj.has_arm_map_
                                        fwDeg = (fwDeg - obj.arm_offset_deg_(:)) ./ obj.arm_sign_(:);
                                    end
                                    
                                    obj.actualJointDeg_ = fwDeg;
                                    obj.armReady_ = strcmpi(strtrim(parts(8)), "READY");
                                    obj.lastTelemetryTime_ = tic;
                                end
                                
                            elseif strcmp(line,"ARM_READY")
                                obj.armReady_ = true;
                                obj.lastTelemetryTime_ = tic;
                            end
                        end
                    end
                end
            end
            
            %==========================================================
            % TELEMETRY TIMEOUT
            %==========================================================
            if toc(obj.lastTelemetryTime_) > 0.5
                obj.armReady_ = false;
            end
            
            actualWheelRPM = obj.actualWheelRPM_;
            actualJointDeg = obj.actualJointDeg_;
            armReady       = obj.armReady_;
        end
        
        %==============================================================
        % SHUTDOWN
        %==============================================================
        function releaseImpl(obj)
            if coder.target('MATLAB') && ~isempty(obj.DeviceHandle)
                write(obj.DeviceHandle, uint8(char("STOP_ALL" + newline)));
            end
            obj.DeviceHandle = [];
        end
        
        %==============================================================
        % OUTPUT DEFINITIONS
        %==============================================================
        function num = getNumOutputsImpl(~)
            num = 3;
        end
        function [o1,o2,o3] = getOutputSizeImpl(~)
            o1 = [4 1]; o2 = [6 1]; o3 = [1 1];
        end
        function [o1,o2,o3] = getOutputDataTypeImpl(~)
            o1 = "double"; o2 = "double"; o3 = "logical";
        end
        function [o1,o2,o3] = isOutputComplexImpl(~)
            o1 = false; o2 = false; o3 = false;
        end
        function [o1,o2,o3] = isOutputFixedSizeImpl(~)
            o1 = true; o2 = true; o3 = true;
        end
    end
end