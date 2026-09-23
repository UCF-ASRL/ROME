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
    end
    methods (Access = protected)
        %==============================================================
        % SETUP
        %==============================================================
        function setupImpl(obj)
            obj.actualWheelRPM_ = zeros(4,1);
            if evalin('base', 'exist(''arm_home'',''var'')')
                obj.actualJointDeg_ = rad2deg(evalin('base','arm_home(:)'));
            else
                obj.actualJointDeg_ = zeros(6,1);
            end
            obj.armReady_ = false;
            obj.lastTelemetryTime_ = tic;
        end
        %==============================================================
        % MAIN STEP
        %==============================================================
        function [actualWheelRPM,...
          actualJointDeg,...
          armReady] = ...
          stepImpl(obj,...
                   wheelRPM,...
                   jointRad,...
                   Enable)
    % Default outputs
    actualWheelRPM = obj.actualWheelRPM_;
    actualJointDeg = obj.actualJointDeg_;
    armReady       = obj.armReady_;
    %==========================================================
    % MATLAB-ONLY NETWORKING
    %==========================================================
    if coder.target('MATLAB')
        %------------------------------------------------------
        % CONNECT ONCE
        %------------------------------------------------------
        if Enable && obj.initial_run
            disp('Connecting to ROME ESP32...');
            obj.DeviceHandle = ...
                tcpclient( ...
                obj.ESP32_IP,...
                obj.ESP32_Port);
            pause(0.5);
            write( ...
                obj.DeviceHandle,...
                uint8(char("START_GV" + newline)));
            disp('ROME Connected.');
            obj.initial_run = false;
        end
        %------------------------------------------------------
        % SEND COMMANDS
        %------------------------------------------------------
        if Enable && ~isempty(obj.DeviceHandle)
            jointDeg = rad2deg(jointRad);
            % Model-to-firmware joint map, set in define_constants and
            % measured on the arm (GETTING_STARTED.md step 3). Identity
            % until then.
            if evalin('base', 'exist(''arm_offset_deg'',''var'')')
                jointDeg = evalin('base', 'arm_sign(:).''') .* jointDeg(:).' ...
                         + evalin('base', 'arm_offset_deg(:).''');
            end
            % Arm freeze: keep the parked posture on the wire (define_constants,
            % ARM FREEZE) while the Teensy cannot be flashed. Base unaffected.
            if evalin('base', 'exist(''arm_freeze'',''var'') && arm_freeze ~= 0')
                jointDeg = evalin('base', 'arm_freeze_deg(:).''');
            end
            % Cap to what the firmware accepts, in firmware degrees, so a
            % posture outside the ranges is pulled to the end instead of
            % being rejected and silently not executed.
            if evalin('base', 'exist(''arm_fw_switch'',''var'')')
                [jointDeg, hit] = arm_clamp(jointDeg, evalin('base','arm_fw_switch'), ...
                                                      evalin('base','arm_fw_other'), ...
                                                      evalin('base','arm_fw_margin_deg'));
                if any(hit) && ~obj.clamp_warned
                    warning('ROMECommand:clamp', ...
                        'arm command capped to the firmware range on joint(s) %s', ...
                        mat2str(find(hit)));
                    obj.clamp_warned = true;
                end
            end
            msg = sprintf(...
               ['ROME,' ...
                '%.3f,%.3f,%.3f,%.3f,' ...
                '%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n'],...
                wheelRPM(1),...
                wheelRPM(2),...
                wheelRPM(3),...
                wheelRPM(4),...
                jointDeg(1),...
                jointDeg(2),...
                jointDeg(3),...
                jointDeg(4),...
                jointDeg(5),...
                jointDeg(6));
            write( ...
                obj.DeviceHandle,...
                uint8(msg));
        end
        %------------------------------------------------------
        % RECEIVE TELEMETRY
        %------------------------------------------------------
        if ~isempty(obj.DeviceHandle)
            available = obj.DeviceHandle.NumBytesAvailable;
            if available > 0
                raw = char( ...
                    read( ...
                    obj.DeviceHandle,...
                    available,...
                    "char"));
                lines = splitlines(string(raw));
                maxLines = min(length(lines),10);
                for k = 1:maxLines
                    line = strtrim(lines(k));
                    if startsWith(line,"GV,")
                        parts = split(line,",");
                        if numel(parts) >= 9
                            obj.actualWheelRPM_ = ...
                                [ ...
                                str2double(parts(2))
                                str2double(parts(3))
                                str2double(parts(4))
                                str2double(parts(5)) ];
                            obj.lastTelemetryTime_ = tic;
                        end
                    elseif startsWith(line,"ARM,")
                        parts = split(line,",");
                        if numel(parts) >= 8
                            fwDeg = ...
                                [ ...
                                str2double(parts(2))
                                str2double(parts(3))
                                str2double(parts(4))
                                str2double(parts(5))
                                str2double(parts(6))
                                str2double(parts(7)) ];
                            % Undo the model-to-firmware map, so what goes
                            % back into the solve (ArmDeg2Rad -> StateSource
                            % when UseMotive = 1) is in MODEL degrees, the
                            % convention it commands in.
                            if evalin('base', 'exist(''arm_offset_deg'',''var'')')
                                fwDeg = (fwDeg(:) - evalin('base','arm_offset_deg(:)')) ...
                                        ./ evalin('base','arm_sign(:)');
                            end
                            obj.actualJointDeg_ = fwDeg(:);
                            stateStr = ...
                                strtrim(parts(8));
                            obj.armReady_ = ...
                                strcmpi(stateStr,"READY");
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

            if coder.target('MATLAB')

                if ~isempty(obj.DeviceHandle)

                    write( ...
                        obj.DeviceHandle,...
                        uint8(char("STOP_ALL" + newline)));

                end

            end

            obj.DeviceHandle = [];

        end
        %==============================================================
        % OUTPUT DEFINITIONS
        %==============================================================
        function num = getNumOutputsImpl(~)
            num = 3;
        end
        function [o1,o2,o3] = ...
            getOutputSizeImpl(~)
            o1 = [4 1];
            o2 = [6 1];
            o3 = [1 1];
        end
        function [o1,o2,o3] = ...
            getOutputDataTypeImpl(~)
            o1 = "double";
            o2 = "double";
            o3 = "logical";
        end
        function [o1,o2,o3] = ...
            isOutputComplexImpl(~)
            o1 = false;
            o2 = false;
            o3 = false;
        end
        function [o1,o2,o3] = ...
            isOutputFixedSizeImpl(~)
            o1 = true;
            o2 = true;
            o3 = true;
        end
    end
end
