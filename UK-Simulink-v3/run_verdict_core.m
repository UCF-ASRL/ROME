function V = run_verdict_core(F)
%RUN_VERDICT_CORE  PASS / FAIL of one robot run, from facts gathered at its end (focus plan F14a, 19 Nov 2026).
%   V = run_verdict_core(F)       pure: reads no file, no workspace, no model; prints nothing.
%
%   THE RULE IS A CONTROLLER RULING, AUTHOR MAY CHANGE (focus plan F14a, verbatim):
%     "mode 2 PASS = run reached T_stop, no refusal, no watchdog stop, STOP_ALL sent, recorder file saved, every
%      wheel command <= max_rpm; mode 3 = mode 2 + arm telemetry within the bench tolerance of the commanded joints
%      over the run; mode 4 = mode 3 + no camera dropout beyond the F6 limit."
%   Items (id: what is judged, from which fact; every threshold from F, which the caller takes from define_constants /
%   lab_settings (max_rpm, T_stop, dt_9dof) and allowed_ranges.m C (fw_watchdog_s, arm_track_tol_deg,
%   cam_dropout_max_s)):
%     tstop     the run reached T_stop without an error: t_end > T_stop - dt (the fixed-step solver's last step is the
%               last multiple of dt not beyond T_stop, so it can end up to one step short of T_stop; derived, not a
%               tolerance) and err_msg empty
%     refusal   no refusal: refused false and err_msg does not contain "Run refused" (every refusal text of
%               apply_pacing, arm_freeze_banner and motive_cal_check ends with it)
%     watchdog  no watchdog stop: the longest wall-clock gap between consecutive ROME lines (speed packets) sent before
%               STOP_ALL is <= fw_watchdog_s (a speed packet sets the Mega's lastCommandTime and the Mega zeroes the
%               wheels after a longer silence, ROME_Mega_Code.ino:144-155, :180-190 as read in
%               docs/verification/2026-11-19_f4_pacing_rate_analysis.md). The gap from START_GV to the first ROME line is
%               not counted: whether START_GV sets lastCommandTime is NOT VERIFIED. NOT MEASURED without a send log
%     stopall   STOP_ALL sent: the send log's last line is STOP_ALL and it appears exactly once. NOT MEASURED without a
%               send log
%     saved     the run file was saved: saved_file non-empty and save_err empty
%     wheels    every wheel command <= max_rpm: max |wheel_sent| <= max_rpm (wheel_sent = CommandGuard output, so only a
%               guard fault can make this fail). NOT MEASURED when wheel_sent is empty
%     arm       (mode 3, 4) arm telemetry within arm_track_tol_deg of the commanded joints: max over the samples with
%               arm_ready of |joint_meas_deg - rad2deg(joint_sent_rad)| (model degrees, same step). arm_track_tol_deg
%               NaN = UNSOURCED: the measured number is printed and nothing is judged
%     dropout   (mode 4) no camera-guard stop and no dropout beyond cam_dropout_max_s (F.cam_guard, F6); NaN = UNSOURCED
%   Status of an item: 'PASS' | 'FAIL' | 'NOT MEASURED' (its data are absent) | 'UNSOURCED' (no threshold: measured
%   number only). Overall V.verdict: 'FAIL' if any item FAILs; else 'INCOMPLETE' if any item is NOT MEASURED (a run
%   cannot PASS on an item nobody measured); else 'PASS'. UNSOURCED items do not change the overall verdict; the
%   line lists them as "not judged".
%
%   F fields (absent fields count as absent data):
%     mode            run mode 2, 3 or 4 (double)                  T_stop, dt  run length and solver step (s)
%     t_end           simulation time at the stop (s)              err_msg     the run's error message ('' = none)
%     refused         true when a refusal stopped the run (logical); refusal_id (char) names it
%     saved_file      file the run was saved to ('' = none)        save_err    save error ('' = none)
%     wheel_sent      N x 4 wheel commands (rpm)                   max_rpm     wheel ceiling (rpm)
%     send_log        [] or struct with tw (wall s, column) and line (cellstr, LF kept or not) of every line sent
%     fw_watchdog_s   (s)
%     joint_sent_rad  N x 6 joint command, model rad               joint_meas_deg  N x 6 arm telemetry, model deg
%     arm_ready       N x 1 telemetry READY flag                    arm_track_tol_deg (deg), NaN = UNSOURCED
%     cam_dropout_max_s (s), NaN = UNSOURCED
%   V fields: verdict (char), mode, items (struct array: id, name, status, detail), line (one line), text (the line
%   plus one indented line per item, LF-separated).
if ~isstruct(F), F = struct(); end
mode = g(F, 'mode', NaN);                          % run mode (double)
items = struct('id', {}, 'name', {}, 'status', {}, 'detail', {});

% ---- tstop -------------------------------------------------------------------------------------------------------
t_end = g(F, 't_end', NaN);  T_stop = g(F, 'T_stop', NaN);  dt = g(F, 'dt', NaN);   % (s)
err = char(g(F, 'err_msg', ''));                   % error message of the run (char)
reached = isfinite(t_end) && isfinite(T_stop) && isfinite(dt) && t_end > T_stop - dt;
errTxt = '';  if ~isempty(err), errTxt = sprintf('; stopped by an error: %s', oneline(err, 160)); end
items(end+1) = it('tstop', 'reached T_stop without an error', pf(reached && isempty(err)), ...
    sprintf('t_end %.3f s, T_stop %.3f s (one step %.3f s)%s', t_end, T_stop, dt, errTxt));

% ---- refusal -----------------------------------------------------------------------------------------------------
refused = logical(g(F, 'refused', false)) || contains(err, 'Run refused');
rid = char(g(F, 'refusal_id', ''));
if refused
    d = 'the run was refused';  if ~isempty(rid), d = [d ' (' rid ')']; end
else
    d = 'no refusal';
end
items(end+1) = it('refusal', 'no refusal', pf(~refused), d);

% ---- watchdog and STOP_ALL (send log) ------------------------------------------------------------------------------
SL = g(F, 'send_log', []);
wd = g(F, 'fw_watchdog_s', NaN);                   % (s)
haveLog = isstruct(SL) && isfield(SL, 'tw') && isfield(SL, 'line') && ~isempty(SL.line);
if haveLog
    ln = cellfun(@(s) strtrim(char(s)), SL.line(:), 'UniformOutput', false);   % lines without LF (cellstr)
    tw = double(SL.tw(:));                         % wall stamps (s)
    iStop  = find(strcmp(ln, 'STOP_ALL'), 1, 'first');
    if isempty(iStop), iStop = numel(ln) + 1; end
    iR = find(startsWith(ln, 'ROME,'));            % speed packets (index); each one resets the Mega's watchdog clock
    iR = iR(iR < iStop);                           % those sent before STOP_ALL
    seg = tw(iR);                                  % their wall stamps (s)
    gap = 0;  tgap = NaN;                          % longest gap (s) and the line index where it ends
    if numel(seg) > 1, [gap, k] = max(diff(seg)); tgap = iR(k + 1); end
    if ~isfinite(wd)
        items(end+1) = it('watchdog', 'no watchdog stop', 'UNSOURCED', sprintf('longest gap between commands %.3f s (no watchdog threshold given)', gap));
    else
        items(end+1) = it('watchdog', 'no watchdog stop', pf(gap <= wd), ...
            sprintf('longest gap between commands %.3f s (ending at line %d of %d), watchdog %.3f s', gap, tgap, numel(ln), wd));
    end
    nStop = sum(strcmp(ln, 'STOP_ALL'));
    okStop = nStop == 1 && strcmp(ln{end}, 'STOP_ALL');
    items(end+1) = it('stopall', 'STOP_ALL sent', pf(okStop), sprintf('STOP_ALL x%d, last line ''%s''', nStop, oneline(ln{end}, 40)));
else
    items(end+1) = it('watchdog', 'no watchdog stop', 'NOT MEASURED', 'no send log (no recorder on this link, Q-F3a)');
    items(end+1) = it('stopall', 'STOP_ALL sent', 'NOT MEASURED', 'no send log (no recorder on this link, Q-F3a)');
end

% ---- saved -------------------------------------------------------------------------------------------------------
sf = char(g(F, 'saved_file', ''));  se = char(g(F, 'save_err', ''));
if isempty(se) && ~isempty(sf)
    items(end+1) = it('saved', 'run file saved', 'PASS', sf);
else
    if isempty(se), se = 'no file'; end
    items(end+1) = it('saved', 'run file saved', 'FAIL', oneline(se, 160));
end

% ---- wheels ------------------------------------------------------------------------------------------------------
W = double(g(F, 'wheel_sent', []));                % N x 4 (rpm)
mr = g(F, 'max_rpm', NaN);                         % (rpm)
if isempty(W)
    items(end+1) = it('wheels', 'every wheel command <= max_rpm', 'NOT MEASURED', 'wheel_sent not logged');
else
    pk = max(abs(W(:)));
    items(end+1) = it('wheels', 'every wheel command <= max_rpm', pf(isfinite(mr) && pk <= mr), sprintf('peak |wheel| %.3f rpm, max_rpm %g', pk, mr));
end

% ---- arm (modes 3, 4) --------------------------------------------------------------------------------------------
if mode >= 3
    JS = double(g(F, 'joint_sent_rad', []));  JM = double(g(F, 'joint_meas_deg', []));  AR = g(F, 'arm_ready', []);
    tol = g(F, 'arm_track_tol_deg', NaN);          % (deg)
    if isempty(JS) || isempty(JM) || size(JS, 1) ~= size(JM, 1)
        items(end+1) = it('arm', 'arm telemetry within the bench tolerance', 'NOT MEASURED', 'joint_sent_rad / joint_meas_deg not logged or of different length');
    else
        use = true(size(JS, 1), 1);                % samples with telemetry (logical)
        if numel(AR) == size(JS, 1), use = logical(AR(:)); end
        if ~any(use)
            items(end+1) = it('arm', 'arm telemetry within the bench tolerance', 'NOT MEASURED', 'no sample with arm telemetry (arm_ready never true)');
        else
            dmax = max(abs(JM(use, :) - rad2deg(JS(use, :))), [], 'all');   % (deg)
            d = sprintf('max |telemetry - command| %.4f deg over %d of %d samples with telemetry', dmax, sum(use), numel(use));
            if isfinite(tol)
                items(end+1) = it('arm', 'arm telemetry within the bench tolerance', pf(dmax <= tol), sprintf('%s, tolerance %g deg', d, tol));
            else
                items(end+1) = it('arm', 'arm telemetry within the bench tolerance', 'UNSOURCED', [d '; no bench tolerance exists, not judged']);
            end
        end
    end
end
% ---- camera dropout (mode 4) -------------------------------------------------------------------------------------
if mode >= 4
    CG = double(g(F, 'cam_guard', []));  lim = g(F, 'cam_dropout_max_s', NaN);  nmD = 'no camera dropout beyond the F6 limit';   % N x 6 CameraGuard log [wait hold stop reason longest_s n]; limit (s)
    if ~isfinite(lim), items(end+1) = it('dropout', nmD, 'UNSOURCED', 'no camera dropout limit (allowed_ranges.m); not evaluated');
    elseif isempty(CG), items(end+1) = it('dropout', nmD, 'NOT MEASURED', 'no camera guard signal (cam_guard) in the run output');
    else, items(end+1) = it('dropout', nmD, pf(~any(CG(:, 3)) && max(CG(:, 5)) <= lim), sprintf('longest camera dropout %.2f s, limit %.2f s (UNSOURCED value), %d dropout(s), guard stop %d (reason %d)', max(CG(:, 5)), lim, max(CG(:, 6)), any(CG(:, 3)), max(CG(:, 4))));
    end
end

% ---- overall -----------------------------------------------------------------------------------------------------
st = {items.status};
if any(strcmp(st, 'FAIL'))
    verdict = 'FAIL';
elseif any(strcmp(st, 'NOT MEASURED'))
    verdict = 'INCOMPLETE';
else
    verdict = 'PASS';
end
short = strjoin(cellfun(@(i, s) sprintf('%s %s', i, s), {items.id}, st, 'UniformOutput', false), ' | ');
nj = sum(strcmp(st, 'UNSOURCED'));
tail = '';  if nj > 0, tail = sprintf(' (%d item(s) not judged: UNSOURCED)', nj); end
line = sprintf('RUN VERDICT mode %s: %s%s  [%s]', num2str(mode), verdict, tail, short);
body = cellfun(@(n, s, d) sprintf('    %-42s %-12s %s', n, s, d), {items.name}, st, {items.detail}, 'UniformOutput', false);
V = struct('verdict', verdict, 'mode', mode, 'items', items, 'line', line, ...
           'text', strjoin([{line}, body, {'    (rule: controller ruling, author may change; run_verdict_core.m)'}], newline));
end

function v = g(S, f, d)
% Field f of struct S, or default d when absent or empty-by-absence.
if isfield(S, f), v = S.(f); else, v = d; end
end

function s = pf(ok)
% 'PASS' for true, 'FAIL' for false.
if ok, s = 'PASS'; else, s = 'FAIL'; end
end

function x = it(id, name, status, detail)
% One verdict item.
x = struct('id', id, 'name', name, 'status', status, 'detail', detail);
end

function s = oneline(s, n)
% Text on one line, at most n characters.
s = regexprep(char(s), '\s+', ' ');
if numel(s) > n, s = [s(1:n - 3) '...']; end
end
