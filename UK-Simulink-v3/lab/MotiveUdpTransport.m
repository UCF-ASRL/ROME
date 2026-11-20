classdef MotiveUdpTransport < handle
    % MotiveUdpTransport: the real network transport for record_motive (Motive multicast UDP, listen only).
    % Same interface as tests/RecorderFakeTransport.m: open(), [bytes, why] = receive(), t = time(), close().
    % Same socket calls as ROME_merged/Optitrack/MotiveData.m openSocket/receiveBytes (join the group, 500 ms
    % receive timeout, 2048-byte buffer). It only RECEIVES; nothing is ever sent to Motive or to the robot.
    % NOT TESTED: no offline test can open a socket (Plan 3 Global Constraints). First real use is Part C, C9R.
    properties
        MulticastIP = '239.255.42.99';      % Motive multicast group (MotiveData.m default; the lab's setting is NOT VERIFIED, C9.1)
        DataPort = 1511;                    % UDP data port (MotiveData.m default; NOT VERIFIED for the lab)
    end
    properties (Access = private)
        socket                              % java.net.MulticastSocket
        group                               % java.net.InetAddress
        buffer                              % 1x2048 int8 receive buffer (bytes)
        packet                              % java.net.DatagramPacket over buffer
        t0                                  % tic id at open (clock origin)
    end
    methods
        function obj = MotiveUdpTransport(ip, port)
            if nargin >= 1 && ~isempty(ip), obj.MulticastIP = ip; end
            if nargin >= 2 && ~isempty(port), obj.DataPort = port; end
        end
        function open(obj)
            import java.net.*;
            obj.socket = MulticastSocket(obj.DataPort);
            obj.group = InetAddress.getByName(obj.MulticastIP);
            obj.socket.joinGroup(obj.group);
            obj.socket.setSoTimeout(500);   % receive timeout (ms)
            obj.buffer = zeros(1, 2048, 'int8');
            obj.packet = DatagramPacket(obj.buffer, length(obj.buffer));
            obj.t0 = tic;
        end
        function [bytes, why] = receive(obj)
            why = '';                       % receive error text
            try
                obj.packet.setLength(numel(obj.buffer));
                obj.socket.receive(obj.packet);
                raw = obj.packet.getData();
                n = double(obj.packet.getLength());      % bytes received
                bytes = typecast(int8(raw(1:n)), 'uint8');
                bytes = bytes(:);
            catch e
                bytes = zeros(0, 1, 'uint8');
                why = sprintf('receive failed: %s', e.message);
            end
        end
        function t = time(obj)
            t = toc(obj.t0);                % seconds since open
        end
        function close(obj)
            if ~isempty(obj.socket)
                try
                    obj.socket.leaveGroup(obj.group);
                    obj.socket.close();
                catch
                    % already closed
                end
            end
            obj.socket = [];
        end
    end
end
