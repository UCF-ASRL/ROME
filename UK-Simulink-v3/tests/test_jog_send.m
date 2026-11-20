function varargout = test_jog_send(brk)
%TEST_JOG_SEND  jogROMEArm's arm commands are capped and never sent before telemetry (hw audit row 24 / mismatch 2,
%   fixes5, 20 Nov 2026). Before the fix jogROMEArm.m sent its targets unclamped (no arm_clamp; the firmware has no J4
%   range check while tracking) and sent zeros before any telemetry (J2 = 0 is a firmware range end).
%   NO NETWORK. The send core jog_rome_send.m is copied to a temporary folder (the break, if any, is applied there) and
%   driven with the recording fake link tests/stubs_hw/RobotLinkRecorder.m, copied to the same folder, which records
%   every write in robot_link_record.txt next to itself (read back with tests/read_link_record.m).
%   Checks (all evaluated, then one error test_jog_send:fail listing the failed ones):
%     (a) limits  jog_rome_send('limits') (the real file) returns arm_fw_switch, arm_fw_other, arm_fw_margin_deg exactly
%                 as define_constants gives them in this test's own workspace (the source the other senders use)
%     (b) clamp   synced, request [1000 -1000 50 1000 0 -1000] fw deg: exactly one line is written, and it equals
%                 'ROME,0,0,0,0,<six %.3f>' with the EXPECTED values computed here from the raw ranges (lo = min(switch,
%                 other) + margin, hi = max - margin; not by calling arm_clamp): J1 158, J2 2, J3 50 (in range,
%                 unchanged), J4 163, J5 0, J6 -168 with the lab_settings values of 20 Nov 2026; the returned value
%                 equals the sent one
%     (c) gate    synced false: error jog_rome_send:notelemetry and NOTHING written (no W record at all)
%     (d) finite  synced, a NaN in the request: error jog_rome_send:target and nothing written
%     (e) wiring  jogROMEArm.m (real file) holds no 'ROME,' text and exactly one write( (the STOP_ALL in cleanExit), calls
%                 jog_rome_send(c, targetAngles, synced, lim) and jog_rome_send('limits'): every arm command it sends
%                 goes through the core
%   NOT VERIFIED: the jogger's window and a real socket (the UI loop is not run; (e) is a text check of its wiring).
%   MAKE-IT-FAIL: test_jog_send('<id>') applies one break to the temporary copy of jog_rome_send.m; the named check
%   must FAIL. test_jog_send('list') returns the table.
%     mf_noclamp  the arm_clamp line replaced by an unclamped copy of the request   -> (b)
%     mf_nogate   the synced test removed (if false)                               -> (c)
%   Flat function without local functions (CLAUDE.md section 5, crash 2).
B = {'mf_noclamp', 'b', 'out = arm_clamp(target, lim.fw_switch, lim.fw_other, lim.margin);', 'out = target;', 'no clamp';
     'mf_nogate',  'c', '|| ~synced', '|| false', 'no telemetry gate'};
if nargin >= 1 && strcmp(brk, 'list'), varargout{1} = B; return; end
ib = [];
if nargin >= 1 && ~isempty(brk)
    ib = find(strcmp(B(:,1), brk));
    assert(numel(ib) == 1, 'test_jog_send:brk', 'unknown break %s', brk);
    fprintf('MAKE-IT-FAIL break %s (check (%s) must FAIL): %s\n', brk, B{ib,2}, B{ib,5});
end
here = fileparts(mfilename('fullpath'));          % tests/ (char)
root = fileparts(here);                           % ROME_merged (char)
addpath(root, here);
fails = {};
% ---- (a) limits from define_constants ------------------------------------------------------------------------------
lim = jog_rome_send('limits');                    % the real core (struct)
evalc('define_constants');                        % this test's own workspace: the values every sender reads
okA = isequal(lim.fw_switch, arm_fw_switch) && isequal(lim.fw_other, arm_fw_other) && isequal(lim.margin, arm_fw_margin_deg);
fprintf('(a) limits: switch %s, other %s, margin %g (define_constants: %s, %s, %g) -> %s\n', mat2str(lim.fw_switch), ...
        mat2str(lim.fw_other), lim.margin, mat2str(arm_fw_switch), mat2str(arm_fw_other), arm_fw_margin_deg, mat2str(okA));
if ~okA, fails{end+1} = '(a) limits'; end
% ---- temporary copy of the core and the fake link -------------------------------------------------------------------
tmp = tempname;  mkdir(tmp);  path0 = path;  startdir = pwd;
cleanup = onCleanup(@() cellfun(@(c) c(), {@() cd(startdir), @() path(path0), @() rmdir(tmp, 's')})); %#ok<NASGU>
s = fileread(fullfile(root, 'jog_rome_send.m'));
if ~isempty(ib)
    n = numel(strfind(s, B{ib,3}));
    assert(n == 1, 'test_jog_send:brk', 'break edit matches %d times (expected 1)', n);
    s = strrep(s, B{ib,3}, B{ib,4});
end
fid = fopen(fullfile(tmp, 'jog_rome_send.m'), 'w');  fwrite(fid, uint8(s));  fclose(fid);
copyfile(fullfile(here, 'stubs_hw', 'RobotLinkRecorder.m'), fullfile(tmp, 'RobotLinkRecorder.m'), 'f');
clear jog_rome_send RobotLinkRecorder;
addpath(tmp);  cd(tmp);  rehash;                 % the current folder outranks the path: ROME_merged must not shadow the copy
wh = cellfun(@which, {'jog_rome_send', 'RobotLinkRecorder'}, 'UniformOutput', false);
assert(all(startsWith(wh, tmp)), 'test_jog_send:path', 'not resolved in the temporary folder: %s', strjoin(wh, ' | '));
recf = fullfile(tmp, 'robot_link_record.txt');
% ---- (b) clamp -------------------------------------------------------------------------------------------------------
req = [1000 -1000 50 1000 0 -1000];               % request (fw deg): J1, J2, J4, J6 beyond their ranges, J3, J5 inside
lo = min(arm_fw_switch, arm_fw_other) + arm_fw_margin_deg;   % 1x6 lower cap (deg)
hi = max(arm_fw_switch, arm_fw_other) - arm_fw_margin_deg;   % 1x6 upper cap (deg)
want = req;  want(req < lo) = lo(req < lo);  want(req > hi) = hi(req > hi);   % expected sent joints (deg)
wantLine = sprintf('ROME,0,0,0,0,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f', want);
if isfile(recf), delete(recf); end
c = RobotLinkRecorder('192.0.2.1', 0);            % fake link (TEST-NET address, never contacted)
errB = '';  sentB = [];
try, sentB = jog_rome_send(c, req, true, lim); catch ex, errB = ex.message; end
S = read_link_record(recf);
lnB = cellfun(@strtrim, S.line(:).', 'UniformOutput', false);
okB = isempty(errB) && numel(lnB) == 1 && strcmp(lnB{1}, wantLine) && isequal(sentB, want);
fprintf('(b) request %s -> expected %s; written {%s}; returned %s; error ''%s'' -> %s\n', mat2str(req), wantLine, ...
        strjoin(lnB, ' | '), mat2str(sentB), errB, mat2str(okB));
if ~okB, fails{end+1} = '(b) out-of-range request clamped'; end
% ---- (c) no telemetry ------------------------------------------------------------------------------------------------
if isfile(recf), delete(recf); end
c = RobotLinkRecorder('192.0.2.1', 0);
idC = '';  msgC = '';
try, jog_rome_send(c, [0 0 0 0 0 0], false, lim); catch ex, idC = ex.identifier;  msgC = ex.message; end
S = read_link_record(recf);
okC = strcmp(idC, 'jog_rome_send:notelemetry') && isempty(S.line);
fprintf('(c) not synced: error ''%s'' (%s); lines written %d -> %s\n', idC, msgC, numel(S.line), mat2str(okC));
if ~okC, fails{end+1} = '(c) nothing sent before telemetry'; end
% ---- (d) non-finite request ------------------------------------------------------------------------------------------
if isfile(recf), delete(recf); end
c = RobotLinkRecorder('192.0.2.1', 0);
idD = '';
try, jog_rome_send(c, [10 NaN 10 0 0 0], true, lim); catch ex, idD = ex.identifier; end
S = read_link_record(recf);
okD = strcmp(idD, 'jog_rome_send:target') && isempty(S.line);
fprintf('(d) NaN in the request: error ''%s''; lines written %d -> %s\n', idD, numel(S.line), mat2str(okD));
if ~okD, fails{end+1} = '(d) non-finite request refused'; end
% ---- (e) jogROMEArm wiring -------------------------------------------------------------------------------------------
jt = fileread(fullfile(root, 'jogROMEArm.m'));
nRome = numel(strfind(jt, 'ROME,'));  nW = numel(regexp(jt, '(?<![\w.])write\(', 'start'));
okE = nRome == 0 && nW == 1 && contains(jt, 'write(c, "STOP_ALL" + newline, "char");') && ...
      contains(jt, 'jog_rome_send(c, targetAngles, synced, lim)') && contains(jt, 'lim = jog_rome_send(''limits'');');
fprintf('(e) jogROMEArm.m: ''ROME,'' x%d, write( x%d, core called %d, limits read %d -> %s\n', nRome, nW, ...
        contains(jt, 'jog_rome_send(c, targetAngles, synced, lim)'), contains(jt, 'lim = jog_rome_send(''limits'');'), mat2str(okE));
if ~okE, fails{end+1} = '(e) jogROMEArm sends only through jog_rome_send'; end
if ~isempty(fails)
    tag = '';  if ~isempty(ib), tag = sprintf(' [break %s]', B{ib,1}); end
    error('test_jog_send:fail', 'test_jog_send FAILED: %s%s', strjoin(fails, '; '), tag);
end
fprintf('test_jog_send PASS: (a) (b) (c) (d) (e)\n');
end
