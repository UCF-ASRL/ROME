classdef MotiveDataFake < MotiveData
    % MotiveDataFake: MotiveData with its network seam replaced, for tests/test_motive_reader.m.
    % NEVER opens a socket: openSocket does nothing, receiveBytes pops the next datagram from a queue
    % given to the constructor. A queue entry [] (or an exhausted queue) is a receive timeout.
    % Everything else (parsing, hold, valid, nDropped, warning, legacy outputs) is MotiveData's own code.
    % Lives in tests/, not tests/stubs/, because test_pacing copies tests/stubs/MotiveData.m into its
    % model copy. Usage: md = MotiveDataFake({pkt1, [], pkt2}, 'RigidBodyID', 3); [..] = md(1);
    properties (Nontunable)
        Queue = {};                         % datagrams to deliver, in order (cell of uint8 columns; [] = timeout)
    end
    properties (Access = private)
        next = 1;                           % index of the next Queue entry to deliver (-)
    end
    methods
        function obj = MotiveDataFake(queue, varargin)
            obj@MotiveData(varargin{:});
            obj.Queue = queue;
        end
        function tf = socketIsEmpty(obj)
            % true if MotiveData's socket property was never set (no socket was created)
            tf = isempty(obj.socket);
        end
    end
    methods (Access = protected)
        function openSocket(~)
            % no network
        end
        function [bytes, why] = receiveBytes(obj)
            why = '';                       % receive error text (char)
            if obj.next > numel(obj.Queue) || isempty(obj.Queue{obj.next})
                bytes = zeros(0, 1, 'uint8');   % empty: simulated timeout (uint8 column)
                why = 'receive failed: fake timeout';
            else
                bytes = obj.Queue{obj.next};    % queued datagram (uint8 column)
            end
            obj.next = obj.next + 1;
        end
    end
end
