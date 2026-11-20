function S = read_link_record(recf)
%READ_LINK_RECORD  The send log of the fake robot link (tests/stubs_hw/RobotLinkRecorder.m) as the run verdict reads it.
%   S = read_link_record(recf)   recf = robot_link_record.txt of a test's temporary copy (char).
%   S.tw    wall stamps of the writes (s, column; java nanoTime, only differences are meaningful)
%   S.ts    simulation time of each write (s, column)
%   S.line  the lines written (cellstr, LF kept)
%   Only W (write) records; C (connect) and R (telemetry answer) records are skipped. A missing file gives empty
%   fields. Used by tests/test_run_verdict.m as the base variable rome_send_recorder = @() read_link_record(recf)
%   (focus plan F14a): the recorder the run verdict asks for STOP_ALL and the watchdog gap.
S = struct('tw', zeros(0, 1), 'ts', zeros(0, 1), 'line', {cell(0, 1)});
if ~isfile(recf), return; end
C = textscan(fileread(recf), '%s %f %f %s', 'Delimiter', '\t');
w = strcmp(C{1}, 'W');                         % write records (logical)
S.ts = C{2}(w);  S.tw = C{3}(w);
S.line = cellfun(@(h) char(sscanf(h, '%2x').'), C{4}(w), 'UniformOutput', false);
end
