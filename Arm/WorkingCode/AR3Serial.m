classdef AR3Serial
    %AR3Serial Summary of this class goes here
    %   Detailed explanation goes here df
    
    properties(Access = private, Constant = true)
        STEPPER_CREATE = uint8(1);
        SET_RADSEC = uint8(2);
        UPDATE_STEPPERS = uint8(3);

        SET_STATES = uint8(4);
        UPDATE_STATES = uint8(5);

        CALIBRATE = uint8(6);
        READ = uint8(7);
        SET = uint8(8);
        READ_ALL = uint8(9);
    end
    
    properties
        serial
    end
    
    properties(Access = private, Constant = true)
        %A collection of default positions
        % REST = [0,-110*pi/180,141*pi/180,0,0,0,...
        %         .25,.25,.25,.25,.25,.25];
        % CWLimit = [-170*(pi/180),-132*(pi/180),-0*(pi/180),...
        %            -155*(pi/180),-105*(pi/180),-155*(pi/180)];
        % CCWLimit = [170*(pi/180),0*(pi/180),141*(pi/180),...
        %             155*(pi/180),105*(pi/180),155*(pi/180)];
        %CWLimit = [-180*(pi/180), 42.5*(pi/180),-145*(pi/180),...
        %            -155*(pi/180), -86.5*(pi/180), -155*(pi/180)]
        %CCWLimit = [160*(pi/180), -42.5*(pi/180), 145*(pi/180)...
        %            155*(pi/180), 86.5*(pi/180), 155*(pi/180)]
        % REST = [0,120,140,0,0,0,...
        %         .25,.25,.25,.25,.25,.25];
        %  CWLimit = [-180,132,1,...
        %             -165,90,-170];
        %  CCWLimit = [160,0,1,...
        %              165,-90,180]
        CWLimit = [160,0,141,...
                   165,-90,180];
        CCWLimit = [-180,132,1,...
                    -165,90,-170]
        % CWLimit = [-170,-129,1,...
        %            -164,-104,-148];
        % CCWLimit = [170,0,143,...
        %             164,104,148]
    end

    properties(Access = private, Constant = true)
        % Steps/deg for each stepper (currently unsure about motor 4,5,6)
        STEPPER_CONSTANTS_DEG = [44.44444444, ...
                             55.55555556, ...
                             55.55555556, ...
                             46.66666667, ...
                             10, ...
                             20];
        %Steps/radian for each stepper(unsure if right)
        STEPPER_CONSTANTS_RAD = [1/((.022368421*2)), ...
                             1/(.018082192), ...
                             1/(.017834395), ...
                             1/(.021710526), ...
                             1/(.045901639), ...
                             1/(.046792453)];
    end
    
    methods
        function obj = AR3Serial(serial)
            %GVSERIAL Construct an instance of this class
            %   Detailed explanation goes here
            obj.serial = serial;
            fopen(obj.serial);
           
        end
        
        function [] = calibrate(obj)
            CMD = obj.CALIBRATE;
            writePacket(obj.serial,CMD);
        end
        
        function [steps] = read(obj,ID)
            CMD = obj.READ;
            data = uint8([CMD,ID]);
            
            writePacket(obj.serial,data);
            
            binarySteps = readPacket(obj.serial);
            steps = typecast(uint8(binarySteps(1:4)),'int32');
        end
         
        function [stepsArray] = read_all(obj)
            CMD = obj.READ_ALL;
            data = uint8(CMD);
            
            writePacket(obj.serial,data);
            
            binarySteps = readPacket(obj.serial);
            
            for i = [1 5 9 13 17]
                stepsArray(i) = typecast(uint8(binarySteps(i,i+3)),'int32');
            end
        end
        
        % Not Implemented
        function [] = updateSteppers(obj,radSecArray)
            CMD = obj.UPDATE_STEPPERS;

            countsSecArray = radSecArray*obj.STEPPER_CONSTANTS;
            countsSecArrayBin = typecast(single(countsSecArray),'uint8');
            
            data = [CMD, countsSecArrayBin];
            
            writePacket(obj.serial,data);
        end
        
        function [] = setStates(obj,states,ID)
            CMD = obj.SET_STATES;
            ID = uint8(ID);
            
            % Check limits
            if (states(1) >= obj.CCWLimit(ID+1))
                    %states(1) = obj.CCWLimit(ID+1);
            elseif (states(1) <= obj.CWLimit(ID+1))
                    %states(1) = obj.CWLimit(ID+1);
            end
            
            states = states * obj.STEPPER_CONSTANTS(ID+1);
            
            disp(states)
            
            position = [typecast(int32(states(1)),'uint8')];
            velocity = [typecast(single(states(2)),'uint8')];

            
            data = [CMD,ID,position,velocity];
            disp(data)
            writePacket(obj.serial,data);
        end
         
        function [] = updateStates(obj,statesArray)
            CMD = obj.UPDATE_STATES;
            
            for i = 1:6
            %     if (statesArray(i) <= obj.CCWLimit(i))
            %         statesArray(i) = obj.CCWLimit(i);
            %         disp(statesArray(i))
            %     elseif (statesArray(i) >= obj.CWLimit(i))
            %         statesArray(i) = obj.CWLimit(i);
            %     end
                % Convert position to steps
                statesArray(i) = statesArray(i) * obj.STEPPER_CONSTANTS_DEG(i);
                % Convert velocity to steps
                statesArray(i+6) = statesArray(i+6) * obj.STEPPER_CONSTANTS_DEG(i);
            end

            
            posArray = int32(statesArray(1:6));
            velArray = single(statesArray(7:12));
            positions = typecast(posArray,'uint8');
            velocities = typecast(velArray,'uint8');
            data = [CMD,positions,velocities];
            
            disp("States ARray");
            disp(statesArray)
            disp("Positions"); 
            disp(positions);
            disp("Velocities");
            disp(velocities); 
            disp("")
            writePacket(obj.serial,data);
            
        end
         
        function [] = default(obj,string)
           if string == "REST"
             obj.updateStates(obj.REST)
           end
        end
        
        % Not Implemented
        function [] = set(obj,statesArray)
            CMD = obj.SET;

            input = typecast(int32(statesArray*obj.STEPPER_CONSTANT1),'uint8');
            data = [CMD,input];
            
            writePacket(data);
        end
        
    end
end
