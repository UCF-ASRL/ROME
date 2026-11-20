classdef MotiveDataInject < MotiveData
    % MotiveDataInject: the REAL MotiveData (Optitrack/MotiveData.m) with only its two network seams
    % overridden, for tests/test_motive_model.m (focus plan F5b). NEVER opens a socket.
    %   openSocket    appends "openSocket" to motive_inject_count.txt in this file's folder (the test's
    %                 temporary copy of ROME_merged; the test counts the calls) and reads the base-workspace
    %                 struct motive_inject, set by the test before each run:
    %                   .nTimeout  receive calls answered with a timeout before the first datagram (count)
    %                   .packet    NatNet frame-of-data datagram returned on every later call (uint8 column)
    %   receiveBytes  a timeout (no datagram) for the first nTimeout calls of the run, then motive_inject.packet.
    % Everything else (Enable handling, natnet_parse_rigidbody, hold, valid, nDropped, outputs 1-9) is
    % MotiveData's own code. The test points the copy's MotiveData block (Enabled Subsystem/MATLAB System,
    % SID 105) at this class, in memory only, in interpreted execution.
    properties (Access = private)
        injCalls = 0;                       % receiveBytes calls so far this run (count)
        injTimeouts = 0;                    % calls answered with a timeout before the first datagram (count)
        injPacket = zeros(0, 1, 'uint8');   % datagram delivered after the timeouts (uint8 column, bytes)
    end
    methods
        function obj = MotiveDataInject(varargin)
            obj@MotiveData(varargin{:});
        end
    end
    methods (Access = protected)
        function openSocket(obj)
            % no network: count the call, read the injection set by the test
            cntf = fullfile(fileparts(which('MotiveDataInject')), 'motive_inject_count.txt');   % counter file (char)
            fid = fopen(cntf, 'a');
            assert(fid > 0, 'MotiveDataInject:counter', 'cannot open the counter file %s', cntf);
            fprintf(fid, 'openSocket\n');
            fclose(fid);
            cfg = evalin('base', 'motive_inject');   % struct with nTimeout (count) and packet (uint8 column)
            obj.injTimeouts = cfg.nTimeout;
            obj.injPacket = cfg.packet;
            obj.injCalls = 0;
        end
        function [bytes, why] = receiveBytes(obj)
            obj.injCalls = obj.injCalls + 1;
            why = '';                           % receive error text (char), '' on success
            if obj.injCalls <= obj.injTimeouts
                bytes = zeros(0, 1, 'uint8');   % no datagram: an injected timeout (uint8 column)
                why = 'receive failed: injected timeout';
            else
                bytes = obj.injPacket;          % the injected datagram (uint8 column)
            end
        end
    end
end
