classdef MotiveData < matlab.System
    % MotiveData: Streams OptiTrack motion capture data into Simulink
        
    properties (Nontunable)
        RigidBodyID = 1;
        HostIP = '127.0.0.1';
    end

    properties (Access = private)
        natnetclient
        origin = [0; 0; 0];
        initial_run = true;
    end

    methods (Access = protected)
        % function setupImpl(obj)
        % 
        % end

        function [x_pos, y_pos, z_pos, x_rot, y_rot, z_rot] = stepImpl(obj, Enable)
            [x_pos, y_pos, z_pos, x_rot, y_rot, z_rot] = deal(0);
            
            if Enable && obj.initial_run
                % Runs once at simulation start
                obj.natnetclient = natnet;
                obj.natnetclient.HostIP = obj.HostIP;
                obj.natnetclient.ClientIP = obj.HostIP;
                obj.natnetclient.ConnectionType = 'Unicast';
                obj.natnetclient.connect;
                
                if obj.natnetclient.IsConnected == 0
                    error('Motive: Connection Failed. Check NatNet settings.');
                end
                
                % Initialize Origin
                data = obj.natnetclient.getFrame;
                rb = data.RigidBodies(obj.RigidBodyID);
                obj.origin = [rb.x; rb.y; rb.z];

                obj.initial_run = false;
            end

            if Enable
                % Main loop - Fetch data
                data = obj.natnetclient.getFrame;
                rb = data.RigidBodies(obj.RigidBodyID);
                
                % Positions - Cast to double to match getOutputDataTypeImpl
                x_pos = double(rb.x - obj.origin(1));
                y_pos = double(rb.y - obj.origin(2));
                z_pos = double(rb.z);
                
                % Rotations
                q = [rb.qw, rb.qx, rb.qy, rb.qz];
                eul = quat2eul(q, 'XYZ');
                
                % Cast Euler angles to double
                x_rot = double(eul(1)); 
                y_rot = double(eul(2)); 
                z_rot = double(eul(3));
            end
        end

        function releaseImpl(obj)
            % Clean up connection when simulation stops
            if ~isempty(obj.natnetclient)
                obj.natnetclient.disconnect;
            end
        end

        % --- REQUIRED FOR MULTI-OUTPUT BLOCKS ---
        
        function [sz1,sz2,sz3,sz4,sz5,sz6] = getOutputSizeImpl(obj)
            sz1=[1 1]; sz2=[1 1]; sz3=[1 1]; sz4=[1 1]; sz5=[1 1]; sz6=[1 1];
        end

        function [dt1,dt2,dt3,dt4,dt5,dt6] = getOutputDataTypeImpl(obj)
            dt1='double'; dt2='double'; dt3='double'; dt4='double'; dt5='double'; dt6='double';
        end

        function [cp1,cp2,cp3,cp4,cp5,cp6] = isOutputComplexImpl(obj)
            cp1=false; cp2=false; cp3=false; cp4=false; cp5=false; cp6=false;
        end

        function [fx1,fx2,fx3,fx4,fx5,fx6] = isOutputFixedSizeImpl(obj)
            fx1=true; fx2=true; fx3=true; fx4=true; fx5=true; fx6=true;
        end
    end
end