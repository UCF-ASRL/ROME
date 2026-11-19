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
    %   RigidBodyID is read since Plan 2 Task 1 (19 Nov 2026): every frame is
    %   parsed by natnet_parse_rigidbody, which rejects a frame whose message
    %   ID is not 7, whose rigid-body ID (the 4 bytes before the pose) is not
    %   RigidBodyID, whose pose has a non-finite value, or whose quaternion
    %   norm is outside 1 +/- 1e-3. The byte offsets are the ones this file
    %   always used (pose at byte 41); when they hold is in
    %   docs/verification/NATNET_PACKET_LAYOUT.md (TRACKER Q-P2a).
    %
    %   The block parameters in ROME_9DOF.slx were HostIP for the NatNet
    %   client. They are MulticastIP and DataPort now.
    %
    % Fails loudly (Plan 2 Task 1). A rejected frame or a receive timeout
    % never produces zeros: the outputs hold the last valid pose, output 8
    % (valid) is 0, output 9 counts the consecutive rejected frames, and the
    % warning MotiveData:dropout is raised on the first rejected frame of a
    % run of them. Before the first valid frame every pose output is NaN.
    %
    % Outputs. 1-6 are the outputs the model is wired to today (Enabled
    % Subsystem SID 100: Mux of 1, 2 and -6), kept unchanged in meaning
    % until Plan 2 Task 2 rewires the model:
    %   1 x_pos, 2 y_pos  position minus the first valid frame's (Motive units)
    %   3 z_pos           raw position z (Motive units)
    %   4-6 x/y/z_rot     Euler XYZ of the quaternion (rad), quat2eul
    % 7-9 are the Plan 2 interface [pose7, valid, nDropped]:
    %   7 pose7    7x1 [x y z qx qy qz qw] RAW (no origin, no Euler)
    %   8 valid    1 if this step's frame was accepted, else 0 (double)
    %   9 nDropped consecutive rejected frames up to this step (double)
    %
    % No network until the block first executes with Enable = 1: the socket
    % is opened in the first enabled step, not in setupImpl, so a run with
    % EnableMotive = 0 never opens a socket. Test seam: openSocket and
    % receiveBytes are protected; tests/MotiveDataFake.m overrides them.

    properties (Nontunable)
        RigidBodyID = 1;                % Motive Streaming ID of the rigid body to read (integer, -)
        MulticastIP = '239.255.42.99';
        DataPort = 1511;
    end

    properties (Nontunable, Access = protected)
        % 1-based byte indices in the datagram (header included) of the uint16
        % message ID, the int32 rigid-body ID and the 7 float32 pose values.
        % Defaults = the offsets this file always used (pose at 41).
        Layout = struct('msgIdOffset', 1, 'rbIdOffset', 37, 'poseOffset', 41);
    end

    properties (SetAccess = private, GetAccess = protected)
        socket                          % java.net.MulticastSocket; empty until the first enabled step
    end

    properties (Access = private)
        group                           % java.net.InetAddress of the multicast group
        buffer                          % 1x2048 int8 receive buffer (bytes)
        packet                          % java.net.DatagramPacket over buffer
        origin = NaN(3, 1);             % x y z of the first valid frame (Motive units); NaN until then
        lastPose = NaN(7, 1);           % last accepted [x y z qx qy qz qw] (Motive units / -); NaN until the first
        nDrop = 0;                      % consecutive rejected frames (count)
        opened = false;                 % true once openSocket has been called this run (logical)
    end

    methods
        function obj = MotiveData(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(~)
            % Nothing here on purpose: no socket until the first enabled step (see header).
        end
        function resetImpl(obj)
            obj.origin   = NaN(3, 1);
            obj.lastPose = NaN(7, 1);
            obj.nDrop    = 0;
        end
        function [x_pos, y_pos, z_pos, x_rot, y_rot, z_rot, pose7, valid, nDropped] = stepImpl(obj, Enable)
            valid = 0;                          % 1 if this step's frame was accepted (double)
            if Enable
                if ~obj.opened
                    obj.openSocket();
                    obj.opened = true;
                end
                [bytes, why] = obj.receiveBytes();   % uint8 column, empty on timeout; why = receive error text
                if ~isempty(bytes)
                    [p, ok, why] = natnet_parse_rigidbody(bytes, obj.RigidBodyID, obj.Layout);   % p: 7x1 pose or NaN; ok: frame accepted (logical)
                else
                    ok = false;                 % timeout or receive error: frame not accepted (logical)
                end
                if ok
                    obj.lastPose = p;
                    obj.nDrop = 0;
                    valid = 1;
                    if any(isnan(obj.origin))
                        obj.origin = p(1:3);
                    end
                else
                    obj.nDrop = obj.nDrop + 1;
                    if obj.nDrop == 1
                        warning('MotiveData:dropout', ...
                            'MotiveData: frame rejected (%s); holding the last valid pose (rigid body %d)', ...
                            why, obj.RigidBodyID);
                    end
                end
            end
            pose7 = obj.lastPose;               % output 7: last accepted [x y z qx qy qz qw] (Motive units / -)
            nDropped = obj.nDrop;               % output 9: consecutive rejected frames (count)
            if any(isnan(pose7))
                % No valid frame yet: NaN, never zeros. quat2eul is not called, because on
                % NaN input its gimbal-lock branch returns 0 for the third angle.
                [x_pos, y_pos, z_pos, x_rot, y_rot, z_rot] = deal(NaN);
                return
            end
            x_pos = pose7(1) - obj.origin(1);   % output 1: x minus the first valid frame's x (Motive units)
            y_pos = pose7(2) - obj.origin(2);   % output 2: y minus the first valid frame's y (Motive units)
            z_pos = pose7(3);                   % output 3: raw z (Motive units)

            % Convert Quaternion to Euler XYZ (radians)
            eul = quat2eul([pose7(7), pose7(4), pose7(5), pose7(6)], 'XYZ');   % 1x3 Euler XYZ (rad)
            x_rot = eul(1);                     % output 4: Euler XYZ angle 1 (rad)
            y_rot = eul(2);                     % output 5: Euler XYZ angle 2 (rad)
            z_rot = eul(3);                     % output 6: Euler XYZ angle 3 (rad)
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
            obj.opened = false;
        end
        function openSocket(obj)
            % Test seam: overridden by tests/MotiveDataFake.m. Opens the native Java multicast socket.
            import java.net.*;
            obj.socket = MulticastSocket(obj.DataPort);
            obj.group = InetAddress.getByName(obj.MulticastIP);

            % Join Motive's Multicast Group on port 1511
            obj.socket.joinGroup(obj.group);
            obj.socket.setSoTimeout(500); % 500 ms timeout

            % Create 2048-byte buffer for incoming binary datagrams
            obj.buffer = zeros(1, 2048, 'int8');
            obj.packet = DatagramPacket(obj.buffer, length(obj.buffer));
        end
        function [bytes, why] = receiveBytes(obj)
            % Test seam: overridden by tests/MotiveDataFake.m. One datagram as a uint8 column,
            % truncated to the bytes received (getLength); empty on timeout or any receive error.
            why = '';                           % receive error text (char), '' on success
            try
                obj.packet.setLength(numel(obj.buffer));     % full buffer for every receive (bytes)
                obj.socket.receive(obj.packet);
                raw = obj.packet.getData();                  % int8 column, the whole 2048-byte buffer
                n = double(obj.packet.getLength());          % bytes received in this datagram
                bytes = typecast(int8(raw(1:n)), 'uint8');   % the received datagram (uint8, n bytes)
                bytes = bytes(:);
            catch e
                bytes = zeros(0, 1, 'uint8');   % empty: no datagram (timeout or error)
                why = sprintf('receive failed: %s', e.message);
            end
        end
        % --- REQUIRED FOR MULTI-OUTPUT BLOCKS ---
        function [sz1,sz2,sz3,sz4,sz5,sz6,sz7,sz8,sz9] = getOutputSizeImpl(~)
            sz1=[1 1]; sz2=[1 1]; sz3=[1 1]; sz4=[1 1]; sz5=[1 1]; sz6=[1 1];
            sz7=[7 1]; sz8=[1 1]; sz9=[1 1];
        end
        function [dt1,dt2,dt3,dt4,dt5,dt6,dt7,dt8,dt9] = getOutputDataTypeImpl(~)
            dt1='double'; dt2='double'; dt3='double'; dt4='double'; dt5='double'; dt6='double';
            dt7='double'; dt8='double'; dt9='double';
        end
        function [cp1,cp2,cp3,cp4,cp5,cp6,cp7,cp8,cp9] = isOutputComplexImpl(~)
            cp1=false; cp2=false; cp3=false; cp4=false; cp5=false; cp6=false;
            cp7=false; cp8=false; cp9=false;
        end
        function [fx1,fx2,fx3,fx4,fx5,fx6,fx7,fx8,fx9] = isOutputFixedSizeImpl(~)
            fx1=true; fx2=true; fx3=true; fx4=true; fx5=true; fx6=true;
            fx7=true; fx8=true; fx9=true;
        end
    end
end
