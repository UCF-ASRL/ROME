classdef RobotLinkTool < RobotLinkRecorder
    % RobotLinkTool: the recording fake robot link (RobotLinkRecorder) with the two calls the TA tools make that
    % ROMECommand does not: flush(c) and readline(c). NEVER opens a socket (fixes5, 20 Nov 2026).
    % Used by tests/test_prerun_wiring.m (h): the copy's testLimitSwitches.m is generated with ONLY tcpclient(...)
    % replaced by RobotLinkTool(...) (one match asserted), so the tool's own write(...) calls are recorded in
    % robot_link_record.txt next to this file (C/W records of RobotLinkRecorder). The fake robot sends nothing:
    % NumBytesAvailable stays 0, flush does nothing, readline errors if ever called.
    methods
        function write(obj, data, varargin)
            % The tools write strings ("TEST_LIMITS" + newline), which tcpclient accepts; RobotLinkRecorder records
            % uint8(data), which errors on a string, so the text is passed on as char (bytes unchanged).
            write@RobotLinkRecorder(obj, char(data), varargin{:});
        end
        function flush(~, varargin)
            % Nothing is ever queued, so there is nothing to discard.
        end
        function line = readline(~) %#ok<STOUT>
            error('RobotLinkTool:readline', 'the fake robot sends no telemetry; readline must not be called (NumBytesAvailable is 0)');
        end
    end
end
