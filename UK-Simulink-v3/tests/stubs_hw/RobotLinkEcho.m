classdef RobotLinkEcho < RobotLinkRecorder
    % RobotLinkEcho: the recording fake robot (RobotLinkRecorder) PLUS a fake arm-telemetry source, for
    % tests/test_motive_emulator.m (focus plan F7, mode 2). NEVER opens a socket.
    % The test builds, in its temporary copy, ROMECommand.m = Keanu's ROMECommand.m with ONLY tcpclient replaced by
    % RobotLinkEcho (asserted there), so ROMECommand's own telemetry parser (the "ARM," branch: firmware degrees ->
    % (fw - arm_offset_deg) ./ arm_sign -> output 2, model degrees) reads what this class answers.
    % Every write is recorded by RobotLinkRecorder (C/W records, robot_link_record.txt). After each "ROME," line it
    % queues ONE telemetry line, which ROMECommand reads in the same step (its write comes before its read):
    %   replay  if robot_link_replay.txt exists in this file's folder: its n-th line answers the n-th ROME line
    %           (the test writes there the joint angles a mode-1 run commanded, firmware degrees at %.17g, so the
    %           fake arm reports exactly the commanded joints); nothing is answered past its last line
    %   echo    otherwise: "ARM,<the six joint fields of the ROME line, as sent>,READY" (the arm is at its command,
    %           to the %.3f print of the line)
    % Each answer is also recorded, as an R record (tab separated, like RobotLinkRecorder): R tsim twall hex.
    properties (Access = private)
        Pending = ''                    % telemetry text not yet read by ROMECommand (char)
        Replay = {}                     % replay lines, LF-terminated (cell of char); empty: echo mode
        NRome = 0                       % ROME lines received so far (count)
    end
    methods
        function obj = RobotLinkEcho(varargin)
            obj@RobotLinkRecorder(varargin{:});
            rf = fullfile(fileparts(mfilename('fullpath')), 'robot_link_replay.txt');   % replay file (char)
            if isfile(rf)
                L = splitlines(string(fileread(rf)));
                L = L(strlength(L) > 0);
                obj.Replay = cellfun(@(s) [s char(10)], cellstr(L), 'UniformOutput', false);
            end
        end
        function write(obj, data, varargin)
            write@RobotLinkRecorder(obj, data, varargin{:});
            s = char(data(:).');                % the line written (char)
            if ~startsWith(s, 'ROME,'), return; end
            obj.NRome = obj.NRome + 1;
            if ~isempty(obj.Replay)
                if obj.NRome > numel(obj.Replay), return; end
                ans_ = obj.Replay{obj.NRome};   % telemetry line (char)
            else
                f = strsplit(strtrim(s), ',');  % 'ROME', 4 wheels (rpm), 6 joints (firmware deg), as text
                assert(numel(f) == 11, 'RobotLinkEcho:format', 'ROME line with %d fields (expected 11): %s', numel(f), strtrim(s));
                ans_ = sprintf('ARM,%s,READY\n', strjoin(f(6:11), ','));
            end
            obj.Pending = [obj.Pending ans_];
            obj.NumBytesAvailable = numel(obj.Pending);
            tsim = NaN;                         % simulation time of the call (s)
            try, tsim = get_param('ROME_9DOF', 'SimulationTime'); catch, end
            twall = double(java.lang.System.nanoTime()) * 1e-9;   % wall stamp (s)
            fid = fopen(obj.RecordFile, 'a');
            fprintf(fid, 'R\t%.17g\t%.9f\t%s\n', tsim, twall, sprintf('%02x', uint8(ans_)));
            fclose(fid);
        end
        function data = read(obj, n, varargin)
            % ROMECommand reads NumBytesAvailable characters: read(DeviceHandle, available, "char").
            n = min(n, numel(obj.Pending));
            data = obj.Pending(1:n);
            obj.Pending(1:n) = [];
            obj.NumBytesAvailable = numel(obj.Pending);
        end
    end
end
