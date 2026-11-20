classdef RecorderFakeTransport < handle
    % RecorderFakeTransport: fake transport for record_motive (tests/test_lab_frame.m). NEVER opens a socket.
    % Same interface as lab/MotiveUdpTransport. receive() pops the next datagram of Queue; [] or an
    % exhausted queue is a timeout. The clock is deterministic: time() = (receive calls so far) * dt.
    properties
        Queue = {};                         % datagrams in order (cell of uint8 columns; [] = timeout)
        dt = 0.0078125;                     % clock advance per receive call (s); 2^-7, exact in binary
        nOpen = 0;                          % open() calls (count)
        nClose = 0;                         % close() calls (count)
        nReceive = 0;                       % receive() calls (count)
    end
    methods
        function obj = RecorderFakeTransport(queue, dt)
            obj.Queue = queue;
            if nargin >= 2, obj.dt = dt; end
        end
        function open(obj)
            obj.nOpen = obj.nOpen + 1;
        end
        function close(obj)
            obj.nClose = obj.nClose + 1;
        end
        function [bytes, why] = receive(obj)
            obj.nReceive = obj.nReceive + 1;
            why = '';
            if obj.nReceive > numel(obj.Queue) || isempty(obj.Queue{obj.nReceive})
                bytes = zeros(0, 1, 'uint8');
                why = 'receive failed: fake timeout';
            else
                bytes = obj.Queue{obj.nReceive};
            end
        end
        function t = time(obj)
            t = obj.nReceive * obj.dt;
        end
    end
end
