classdef MotiveData < matlab.System
    % MotiveData: Reads Motive Multicast UDP stream directly via base Java
    %
    % Supplied by Keanu Brayman, 22 September 2026, replacing the NatNet SDK
    % client. Three notes for whoever reads this next.
    %
    %   quat2eul is NOT a base MATLAB function. It ships with Robotics System
    %   Toolbox and Navigation Toolbox, and neither is installed on every
    %   machine this model runs on. A self-contained implementation sits beside
    %   this file in Optitrack/. Do NOT replace it with ConvertQuattoEulerXYZ
    %   from the dyn-sim utilities: despite its name that routine computes the
    %   negated 321 (ZYX) sequence, and differs from XYZ by up to 3.07 rad.
    %
    %   RigidBodyID is declared but never read. readRigidBodyFrame parses a
    %   fixed byte offset corresponding to rigid body 1, so setting this to
    %   anything else has no effect. Fine while the table carries one rigid
    %   body; a trap if a second is ever added.
    %
    %   The block parameters in ROME_9DOF.slx were HostIP for the NatNet
    %   client. They are MulticastIP and DataPort now.

    properties (Nontunable)
        RigidBodyID = 1;
        MulticastIP = '239.255.42.99';
        DataPort = 1511;
    end

    properties (Access = private)
        socket
        group
        buffer
        packet
        origin = [0; 0; 0];
    end

    methods (Access = protected)
        function setupImpl(obj)
            import java.net.*;
            import java.io.*;

            % Open native Java Multicast Socket
            obj.socket = MulticastSocket(obj.DataPort);
            obj.group = InetAddress.getByName(obj.MulticastIP);

            % Join Motive's Multicast Group on port 1511
            obj.socket.joinGroup(obj.group);
            obj.socket.setSoTimeout(500); % 500 ms timeout

            % Create 2048-byte buffer for incoming binary datagrams
            obj.buffer = zeros(1, 2048, 'int8');
            obj.packet = DatagramPacket(obj.buffer, length(obj.buffer));

            % Initialize origin on first frame
            [x0, y0, z0] = obj.readRigidBodyFrame();
            obj.origin = [x0; y0; z0];
        end
        function [x_pos, y_pos, z_pos, x_rot, y_rot, z_rot] = stepImpl(obj, Enable)
            [x_pos, y_pos, z_pos, x_rot, y_rot, z_rot] = deal(0);

            if Enable
                [x, y, z, qx, qy, qz, qw] = obj.readRigidBodyFrame();

                % Position relative to origin
                x_pos = double(x - obj.origin(1));
                y_pos = double(y - obj.origin(2));
                z_pos = double(z);

                % Convert Quaternion to Euler XYZ (radians)
                q = [qw, qx, qy, qz];
                eul = quat2eul(q, 'XYZ');

                x_rot = double(eul(1));
                y_rot = double(eul(2));
                z_rot = double(eul(3));
            end
        end
        function releaseImpl(obj)
            % Clean shutdown on simulation stop
            if ~isempty(obj.socket)
                try
                    obj.socket.leaveGroup(obj.group);
                    obj.socket.close();
                catch
                    % Socket already closed
                end
            end
        end
        function [x, y, z, qx, qy, qz, qw] = readRigidBodyFrame(obj)
            % Read raw UDP datagram directly from socket
            try
                obj.socket.receive(obj.packet);
                rawBytes = obj.packet.getData();

                % Parse binary payload (NatNet Packet Structure)
                % Rigid body 1 position starts at byte offset 41 for standard NatNet streams
                b = typecast(int8(rawBytes(41:68)), 'single');

                x = b(1);
                y = b(2);
                z = b(3);
                qx = b(4);
                qy = b(5);
                qz = b(6);
                qw = b(7);
            catch
                % If frame read times out, retain zeros
                x = 0; y = 0; z = 0;
                qx = 0; qy = 0; qz = 0; qw = 1;
            end
        end
        % --- REQUIRED FOR MULTI-OUTPUT BLOCKS ---
        function [sz1,sz2,sz3,sz4,sz5,sz6] = getOutputSizeImpl(~)
            sz1=[1 1]; sz2=[1 1]; sz3=[1 1]; sz4=[1 1]; sz5=[1 1]; sz6=[1 1];
        end
        function [dt1,dt2,dt3,dt4,dt5,dt6] = getOutputDataTypeImpl(~)
            dt1='double'; dt2='double'; dt3='double'; dt4='double'; dt5='double'; dt6='double';
        end
        function [cp1,cp2,cp3,cp4,cp5,cp6] = isOutputComplexImpl(~)
            cp1=false; cp2=false; cp3=false; cp4=false; cp5=false; cp6=false;
        end
        function [fx1,fx2,fx3,fx4,fx5,fx6] = isOutputFixedSizeImpl(~)
            fx1=true; fx2=true; fx3=true; fx4=true; fx5=true; fx6=true;
        end
    end
end
