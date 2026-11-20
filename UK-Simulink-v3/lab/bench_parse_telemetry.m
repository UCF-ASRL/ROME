function [vals, aux] = bench_parse_telemetry(lines, kind)
%BENCH_PARSE_TELEMETRY  Parse raw ESP32 telemetry lines exactly as ROMECommand.m does.
%   [vals, aux] = bench_parse_telemetry(lines, 'ARM')
%       vals  Nx6 firmware joint angles (deg), fields 2..7 of "ARM,j1..j6,READY"
%       aux   Nx1 logical, field 8 equals "READY" (case-insensitive)
%   [vals, aux] = bench_parse_telemetry(lines, 'GV')
%       vals  Nx4 actual wheel rpm, fields 2..5 of "GV,act1..act4,cmd1..cmd4"
%       aux   Nx4 echoed command rpm, fields 6..9
%   Acceptance rules copied from ROMECommand.m (ROME_merged): a line is trimmed (L147), "GV," lines need
%   numel(split(line,",")) >= 9 (L149-152), "ARM," lines need >= 8 fields (L156-159, READY test L165). Anything
%   else, including "ARM_READY" (L168), is dropped. jog_joint.m read_arm (L59-72) uses the same ARM rule.
%   The wheel numbers are the Mega's GV format "GV,act1,act2,act3,act4,cmd1,cmd2,cmd3,cmd4"
%   (ROME_Mega_Code.ino L197). Pure function: no I/O, no network.
lines = string(lines(:));
switch upper(kind)
    case 'ARM', pre = "ARM,";  need = 8;  nv = 6;
    case 'GV',  pre = "GV,";   need = 9;  nv = 4;
    otherwise, error('bench_parse_telemetry:kind', 'kind must be ''ARM'' or ''GV''');
end
vals = zeros(0, nv);
if strcmpi(kind, 'ARM'), aux = false(0, 1); else, aux = zeros(0, 4); end
for i = 1:numel(lines)
    ln = strtrim(lines(i));
    if ~startsWith(ln, pre), continue; end
    parts = split(ln, ",");
    if numel(parts) < need, continue; end
    if numel(parts) < 9, parts(end+1:9) = ""; end    % pad so a (mis)relaxed field-count rule cannot index past the end
    v = str2double(parts(2:1+nv)).';                  % 1xnv numbers (NaN if a field is not numeric)
    if strcmpi(kind, 'ARM')
        vals(end+1, :) = v;                           %#ok<AGROW>
        aux(end+1, 1) = strcmpi(strtrim(parts(8)), "READY"); %#ok<AGROW>
    else
        vals(end+1, :) = v;                           %#ok<AGROW>
        aux(end+1, :) = str2double(parts(6:9)).';     %#ok<AGROW>
    end
end
end
