function varargout = test_run_verdict_core(brk)
%TEST_RUN_VERDICT_CORE  Offline test of run_verdict_core.m, the PASS/FAIL rule of a robot run (focus plan F14a).
%   Pure: synthetic facts only, no model, no workspace, no file. The rule is a controller ruling (author may change);
%   this test pins it as written in run_verdict_core.m's header.
%   Checks (all evaluated, then one error test_run_verdict_core:fail listing the failed ones):
%     (i)    mode 2, every fact good: verdict PASS, the 6 items tstop refusal watchdog stopall saved wheels all PASS, the
%            line starts 'RUN VERDICT mode 2: PASS'; boundaries: gap = fw_watchdog_s exactly and peak wheel = max_rpm
%            exactly still PASS; t_end one step short of T_stop (31.00 for 31.0193, dt 0.05) PASS; a 2 s gap between
%            START_GV and the first ROME line is not a speed-packet gap (PASS).
%     (ii)   one fault per item, each making THAT item FAIL and the verdict FAIL, every other item PASS:
%            tstop  t_end = 2 s;  tstop-err  an error message (not a refusal);  refusal  'Run refused' in the error (tstop
%            fails with it: a refused run has an error);  watchdog  one gap of 0.3125 s;  stopall  no STOP_ALL / STOP_ALL
%            twice;  saved  save error;  wheels  peak 80.001 rpm for max_rpm 80.
%     (iii)  NOT MEASURED: no send log -> watchdog and stopall NOT MEASURED, verdict INCOMPLETE (never PASS); no
%            wheel_sent -> wheels NOT MEASURED, INCOMPLETE; a FAIL elsewhere still gives FAIL.
%     (iv)   mode 3: arm item UNSOURCED (arm_track_tol_deg NaN) with the measured maximum over the arm_ready samples
%            only (0.5000 deg; a 50 deg offset on a not-ready sample is ignored); verdict PASS with "1 item(s) not
%            judged"; with a tolerance of 0.1 deg the same data FAIL (the judged path); mode 4: dropout UNSOURCED.
%     (v)    the refusal facts run_refused.m builds: verdict FAIL, refusal FAIL, saved FAIL, watchdog/stopall/wheels NOT
%            MEASURED.
%   MAKE-IT-FAIL: test_run_verdict_core('<id>') copies run_verdict_core.m to a temp folder, applies one break (exactly
%   one match asserted), puts the copy first on the path (which() asserted) and must then FAIL at the named check.
%   test_run_verdict_core('list') returns the table. tests/manual_makefail_f14a.m runs them all.
if nargin < 1 || isempty(brk), brk = ''; end
here = fileparts(mfilename('fullpath'));  root = fileparts(here);
B = {
  'mf_tstop',    'ii',  'reached = isfinite(t_end) && isfinite(T_stop) && isfinite(dt) && t_end > T_stop - dt;', 'reached = true;', 'tstop never fails';
  'mf_refusal',  'ii',  'refused = logical(g(F, ''refused'', false)) || contains(err, ''Run refused'');', 'refused = false;', 'refusals not seen';
  'mf_watchdog', 'ii',  'pf(gap <= wd)', 'pf(true)', 'watchdog gap not judged';
  'mf_stopall',  'ii',  'okStop = nStop == 1 && strcmp(ln{end}, ''STOP_ALL'');', 'okStop = true;', 'STOP_ALL not judged';
  'mf_saved',    'ii',  'if isempty(se) && ~isempty(sf)', 'if true', 'save errors ignored';
  'mf_wheels',   'ii',  'pf(isfinite(mr) && pk <= mr)', 'pf(true)', 'wheel ceiling not judged';
  'mf_armready', 'iv',  'if numel(AR) == size(JS, 1), use = logical(AR(:)); end', '', 'arm maximum over all samples (not-ready ones included)';
  'mf_incomplete','iii', 'elseif any(strcmp(st, ''NOT MEASURED''))', 'elseif false', 'a run with unmeasured items reads PASS';
};
if strcmp(brk, 'list'), varargout{1} = B; return; end
path0 = path;  dir0 = pwd;  cleanup = onCleanup(@() cellfun(@(c) c(), {@() path(path0), @() cd(dir0)})); %#ok<NASGU>
addpath(root);
if ~isempty(brk)
    ib = find(strcmp(B(:,1), brk));
    assert(numel(ib) == 1, 'test_run_verdict_core:brk', 'unknown break %s', brk);
    tmp = tempname;  mkdir(tmp);
    cl2 = onCleanup(@() cellfun(@(c) c(), {@() cd(dir0), @() rmdir(tmp, 's')})); %#ok<NASGU>
    s = fileread(fullfile(root, 'run_verdict_core.m'));
    n = numel(strfind(s, B{ib,3}));
    assert(n == 1, 'test_run_verdict_core:brk', 'break %s matches %d times (expected 1)', brk, n);
    fid = fopen(fullfile(tmp, 'run_verdict_core.m'), 'w');  fwrite(fid, uint8(strrep(s, B{ib,3}, B{ib,4})));  fclose(fid);
    addpath(tmp, '-begin');  cd(tmp);  rehash;   % cd: the current folder outranks the path (it may be ROME_merged)
    assert(strcmp(fileparts(which('run_verdict_core')), tmp), 'test_run_verdict_core:brk', 'run_verdict_core resolves to %s', which('run_verdict_core'));
    fprintf('MAKE-IT-FAIL break %s (check (%s) must FAIL): %s\n', brk, B{ib,2}, B{ib,5});
end
fprintf('test_run_verdict_core: %s\n', which('run_verdict_core'));
fails = {};
lf = char(10);
N = 80;                                        % samples (count)
sendOk = struct('tw', (0:N + 1).' / 16, 'line', {[{['START_GV' lf]}; repmat({['ROME,1,1,1,1,0,0,0,0,0,0' lf]}, N, 1); {['STOP_ALL' lf]}]});
F0 = struct('mode', 2, 't_end', 31.0, 'T_stop', 31.0193, 'dt', 0.05, 'err_msg', '', 'refused', false, ...
            'saved_file', 'C:\x\2026-11-19_120000_s1_mode2.mat', 'save_err', '', 'wheel_sent', 40 * sin((1:N).' / 9) * [1 -1 1 -1], ...
            'max_rpm', 80, 'send_log', sendOk, 'fw_watchdog_s', 0.25);
ids = {'tstop', 'refusal', 'watchdog', 'stopall', 'saved', 'wheels'};
stOf = @(V, id) V.items(strcmp({V.items.id}, id)).status;
allStat = @(V) cellfun(@(i) stOf(V, i), ids, 'UniformOutput', false);
% ---- (i) ----
V = run_verdict_core(F0);
okI = strcmp(V.verdict, 'PASS') && all(strcmp(allStat(V), 'PASS')) && startsWith(V.line, 'RUN VERDICT mode 2: PASS') && numel(V.items) == 6;
Fb = F0;  Fb.send_log.tw(40:end) = Fb.send_log.tw(40:end) + (0.25 - 1/16);   % one gap of exactly 0.25 s (binary-exact steps of 1/16 s)
Fb.wheel_sent(5, 2) = 80;                      % peak exactly max_rpm
Vb = run_verdict_core(Fb);
okI = okI && strcmp(Vb.verdict, 'PASS');
Fg = F0;  Fg.send_log.tw(2:end) = Fg.send_log.tw(2:end) + 2;   % 2 s from START_GV to the first ROME line: not a speed-packet gap
Vg = run_verdict_core(Fg);
okI = okI && strcmp(Vg.verdict, 'PASS');
fprintf('(i)    2 s between START_GV and the first ROME line (not counted): %s\n', Vg.verdict);
fprintf('(i)    all good: %s (%s); boundaries gap 0.25 s and peak 80 rpm: %s\n       line: %s\n', V.verdict, strjoin(allStat(V), ' '), Vb.verdict, V.line);
if ~okI, fails{end+1} = '(i) all-good facts do not give PASS'; end
% ---- (ii) one fault per item ----
T = {'tstop',     @(F) setfield(F, 't_end', 2), {'tstop'};
     'tstop-err', @(F) setfield(F, 'err_msg', 'Assertion detected at time 2'), {'tstop'};
     'refusal',   @(F) setfield(F, 'err_msg', 'arm_freeze_banner:stale ... Run refused.'), {'tstop', 'refusal'};
     'watchdog',  @(F) setfield(F, 'send_log', setfield(F.send_log, 'tw', F.send_log.tw + [zeros(40, 1); 0.25 * ones(N - 38, 1)])), {'watchdog'};
     'stopall',   @(F) setfield(F, 'send_log', setfield(F.send_log, 'line', F.send_log.line(1:end-1))), {'stopall'};
     'stopall2',  @(F) setfield(F, 'send_log', struct('tw', [F.send_log.tw; F.send_log.tw(end) + 0.01], 'line', {[F.send_log.line; {['STOP_ALL' lf]}]})), {'stopall'};
     'saved',     @(F) setfield(setfield(F, 'saved_file', ''), 'save_err', 'MATLAB:MKDIR:OSError: cannot create runs_lab'), {'saved'};
     'wheels',    @(F) setfield(F, 'wheel_sent', [F.wheel_sent; 80.001 0 0 0]), {'wheels'}};
okII = true;
for k = 1:size(T, 1)
    Vk = run_verdict_core(T{k,2}(F0));
    st = allStat(Vk);
    want = repmat({'PASS'}, 1, numel(ids));  want(ismember(ids, T{k,3})) = {'FAIL'};
    ok = strcmp(Vk.verdict, 'FAIL') && isequal(st, want);
    fprintf('(ii)   fault %-9s: verdict %s, items %s (want %s) -> %s\n', T{k,1}, Vk.verdict, strjoin(st, ' '), strjoin(want, ' '), mat2str(ok));
    if strcmp(T{k,1}, 'watchdog'), fprintf('       %s\n', Vk.items(3).detail); end
    okII = okII && ok;
end
if ~okII, fails{end+1} = '(ii) a single fault does not fail exactly its item'; end
% ---- (iii) NOT MEASURED ----
V1 = run_verdict_core(rmfield(F0, 'send_log'));
V2 = run_verdict_core(setfield(F0, 'wheel_sent', [])); %#ok<SFLD>
V3 = run_verdict_core(setfield(rmfield(F0, 'send_log'), 't_end', 1)); %#ok<SFLD>
okIII = strcmp(V1.verdict, 'INCOMPLETE') && isequal(allStat(V1), {'PASS', 'PASS', 'NOT MEASURED', 'NOT MEASURED', 'PASS', 'PASS'}) && ...
        strcmp(V2.verdict, 'INCOMPLETE') && strcmp(stOf(V2, 'wheels'), 'NOT MEASURED') && strcmp(V3.verdict, 'FAIL');
fprintf('(iii)  no send log: %s (%s); no wheel_sent: %s; no send log + short run: %s -> %s\n', V1.verdict, strjoin(allStat(V1), ' '), V2.verdict, V3.verdict, mat2str(okIII));
if ~okIII, fails{end+1} = '(iii) NOT MEASURED items do not give INCOMPLETE'; end
% ---- (iv) mode 3 arm, mode 4 dropout ----
JS = deg2rad(repmat([10 20 30 40 50 60], N, 1));   % commanded (model rad)
JM = rad2deg(JS) + 0.5;  JM(1, :) = JM(1, :) + 50;   % telemetry 0.5 deg off; the first sample 50 deg off but not READY
AR = true(N, 1);  AR(1) = false;
F3 = setfield(setfield(setfield(setfield(setfield(F0, 'mode', 3), 'joint_sent_rad', JS), 'joint_meas_deg', JM), 'arm_ready', AR), 'arm_track_tol_deg', NaN); %#ok<SFLD>
V4 = run_verdict_core(F3);
V5 = run_verdict_core(setfield(F3, 'arm_track_tol_deg', 0.1)); %#ok<SFLD>
V6 = run_verdict_core(setfield(setfield(F3, 'mode', 4), 'cam_dropout_max_s', NaN)); %#ok<SFLD>
okIV = strcmp(V4.verdict, 'PASS') && strcmp(stOf(V4, 'arm'), 'UNSOURCED') && contains(V4.items(strcmp({V4.items.id}, 'arm')).detail, 'max |telemetry - command| 0.5000 deg over 79 of 80 samples') && ...
       contains(V4.line, '1 item(s) not judged') && strcmp(V5.verdict, 'FAIL') && strcmp(stOf(V5, 'arm'), 'FAIL') && ...
       strcmp(stOf(V6, 'dropout'), 'UNSOURCED') && strcmp(V6.verdict, 'PASS');
fprintf('(iv)   mode 3 arm: %s, %s; verdict %s; with tol 0.1 deg: %s; mode 4 dropout: %s -> %s\n', stOf(V4, 'arm'), ...
        V4.items(strcmp({V4.items.id}, 'arm')).detail, V4.verdict, V5.verdict, stOf(V6, 'dropout'), mat2str(okIV));
if ~okIV, fails{end+1} = '(iv) arm / dropout items'; end
% ---- (v) refusal facts (run_refused.m) ----
FR = struct('mode', 2, 'refused', true, 'refusal_id', 'arm_freeze_banner:stale', 'err_msg', '... Run refused.', 'saved_file', '', ...
            'save_err', 'refused before the start: nothing ran, nothing to save');
V7 = run_verdict_core(FR);
okV = strcmp(V7.verdict, 'FAIL') && isequal(allStat(V7), {'FAIL', 'FAIL', 'NOT MEASURED', 'NOT MEASURED', 'FAIL', 'NOT MEASURED'}) && ...
      contains(V7.text, 'arm_freeze_banner:stale');
fprintf('(v)    refusal: %s (%s) -> %s\n', V7.verdict, strjoin(allStat(V7), ' '), mat2str(okV));
if ~okV, fails{end+1} = '(v) refusal facts'; end
fprintf('\n%s\n', V4.text);
if ~isempty(fails)
    error('test_run_verdict_core:fail', 'test_run_verdict_core FAILED: %s', strjoin(fails, '; '));
end
fprintf('\ntest_run_verdict_core PASS: (i)-(v)\n');
end
