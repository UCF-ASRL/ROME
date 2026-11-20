classdef MotiveDataCounting < MotiveData
    % MotiveDataCounting: the REAL MotiveData (Optitrack/MotiveData.m) with only its two network seams
    % overridden, for tests/test_hw_nocam.m (Plan 2 Task 1c). NEVER opens a socket.
    %   openSocket    appends "openSocket" to motive_open_count.txt in this file's folder (the test's
    %                 temporary copy of ROME_merged) and opens nothing. The test asserts 0 calls.
    %   receiveBytes  returns no datagram (a receive timeout to MotiveData's own code).
    % Everything else (Enable handling, parsing, hold, outputs) is MotiveData's own code. The test points
    % the copy's Enabled Subsystem MotiveData block (SID 105) at this class, in memory only.
    methods
        function obj = MotiveDataCounting(varargin)
            obj@MotiveData(varargin{:});
        end
    end
    methods (Access = protected)
        function openSocket(~)
            % Interpreted execution only (as the real block): the first make-it-fail run had the block in
            % code generation, where this fopen failed (docs/verification/2026-11-19_p2t1c_makefail.log).
            % A failed fopen errors here, loudly.
            cntf = fullfile(fileparts(which('MotiveDataCounting')), 'motive_open_count.txt');   % counter file (char)
            fid = fopen(cntf, 'a');
            assert(fid > 0, 'MotiveDataCounting:counter', 'cannot open the counter file %s', cntf);
            fprintf(fid, 'openSocket\n');
            fclose(fid);
        end
        function [bytes, why] = receiveBytes(~)
            bytes = zeros(0, 1, 'uint8');   % no datagram (bytes)
            why = 'no network (test_hw_nocam)';
        end
    end
end
