classdef RobotLinkRecorder < handle
    % RobotLinkRecorder: the fake robot link of tests/test_hw_nocam.m (Plan 2 Task 1c). NEVER opens a socket.
    % tests/stubs_hw/ROMECommand.m constructs it where Keanu's ROMECommand.m constructs tcpclient, with the
    % same arguments, so ROMECommand's own write(...) calls, NumBytesAvailable test and releaseImpl run
    % unchanged against it. Every call is appended to robot_link_record.txt in this file's folder (the
    % test's temporary copy of ROME_merged), one record per line, tab separated:
    %   kind   C = constructed (connect), W = write
    %   tsim   simulation time of ROME_9DOF at the call (s); NaN if the model is not loaded
    %   twall  wall-clock stamp, java.lang.System.nanoTime (s); only differences are meaningful
    %   hex    the bytes written, hex (for C: "<address>:<port>")
    % A file, not a variable, so the record survives an error stop and clear all / clear classes.
    % The fake robot sends no telemetry: NumBytesAvailable is 0, and read() errors if ever called.
    properties
        NumBytesAvailable = 0           % telemetry bytes waiting (count); the fake robot never answers
    end
    properties (SetAccess = private)
        Address                         % address ROMECommand asked for (char)
        Port                            % port ROMECommand asked for (-)
        RecordFile                      % full path of the record file (char)
    end
    methods
        function obj = RobotLinkRecorder(address, port, varargin)
            obj.Address = char(address);
            obj.Port = double(port);
            obj.RecordFile = fullfile(fileparts(mfilename('fullpath')), 'robot_link_record.txt');
            tsim = NaN;                 % simulation time of the call (s)
            try, tsim = get_param('ROME_9DOF', 'SimulationTime'); catch, end
            twall = double(java.lang.System.nanoTime()) * 1e-9;   % wall stamp (s)
            fid = fopen(obj.RecordFile, 'a');
            fprintf(fid, 'C\t%.17g\t%.9f\t%s\n', tsim, twall, sprintf('%02x', uint8(sprintf('%s:%d', obj.Address, obj.Port))));
            fclose(fid);
        end
        function write(obj, data, varargin)
            % One write of ROMECommand: recorded, never transmitted.
            tsim = NaN;                 % simulation time of the call (s)
            try, tsim = get_param('ROME_9DOF', 'SimulationTime'); catch, end
            twall = double(java.lang.System.nanoTime()) * 1e-9;   % wall stamp (s)
            fid = fopen(obj.RecordFile, 'a');
            fprintf(fid, 'W\t%.17g\t%.9f\t%s\n', tsim, twall, sprintf('%02x', uint8(data)));
            fclose(fid);
        end
        function data = read(~, varargin) %#ok<STOUT>
            error('RobotLinkRecorder:read', 'the fake robot sends no telemetry; read must not be called (NumBytesAvailable is 0)');
        end
    end
end
