function V = rome_run_finish(a, b)
%ROME_RUN_FINISH  Save a finished robot run and print its PASS/FAIL line (focus plan F14a, 19 Nov 2026).
%   rome_run_finish(tmr, evt)   the timer callback started by rome_run_end.m (the model's StopFcn). Each tick looks
%                               for a NEW run output in the base workspace (variable out: a Simulink.SimulationOutput
%                               of the same model whose start stamp differs from the one there at the stop, and whose
%                               stop stamp lies within 15 s of the stop); when found it stops the timer and finishes
%                               the run as below. After the timer's last tick (10 s) or 15 s past the stop it ends
%                               silently (a run started by sim() from code returns its output to the caller).
%   V = rome_run_finish(out, R) finishes directly: out = the run output, R = the rome_last_run record of its stop.
%   Finishing: (1) save_run(out, R.snap, 'mode<n>', R.dirOut) (the snapshot of the stop, not of now); a save error is
%   kept as a fact, not raised; (2) the send log, when a recorder is installed: the base variable rome_send_recorder,
%   a function handle returning struct(tw, line) of every line sent on the robot link (the fake robot's recorder); else
%   ROMECommand.m's own send log, base rome_send_log (20 Nov 2026, Q-F3a; see below; when off: NOT MEASURED as before);
%   (3) V = run_verdict_core(facts) with thresholds from the run's own workspace (max_rpm, T_stop, dt_9dof as recorded
%   at the stop) and allowed_ranges.m (fw_watchdog_s, arm_track_tol_deg, cam_dropout_max_s); (4) prints V.text and
%   stores V in the base as rome_last_verdict and the saved file in rome_last_run.file.
%   Never errors in the timer form (a failure is printed as one line).
V = [];
if isa(a, 'timer')
    tmr = a;
    try
        R = tmr.UserData;
        o = [];                                    % candidate run output (Simulink.SimulationOutput or [])
        if evalin('base', 'exist(''out'', ''var'')') == 1, o = evalin('base', 'out'); end
        late = seconds(datetime('now') - datetime(R.stop_wall, 'InputFormat', 'yyyy-MM-dd HH:mm:ss')) > 15;   % (logical)
        if fresh_output(o, R)
            stop(tmr);
            V = rome_run_finish(o, R);
        elseif late || tmr.TasksExecuted >= tmr.TasksToExecute
            stop(tmr);                             % no output reached the base: a run started by code; stay silent
        end
    catch e
        try, stop(tmr); catch, end
        fprintf('rome_run_finish: %s (no verdict for this run)\n', e.message);
    end
    return
end
o = a;  R = b;
try, R.out_start = o.SimulationMetadata.TimingInfo.WallClockTimestampStart; catch, end
F = struct('mode', R.mode, 't_end', R.t_end, 'T_stop', R.T_stop, 'dt', R.dt, 'max_rpm', R.max_rpm);
F.err_msg = '';  try, F.err_msg = char(o.ErrorMessage); catch, end
F.refused = contains(F.err_msg, 'Run refused');
F.wheel_sent = logged(o, 'wheel_sent');            % N x 4 (rpm)
F.joint_sent_rad = logged(o, 'joint_sent_rad');    % N x 6 (model rad)
F.joint_meas_deg = logged(o, 'joint_meas_deg');    % N x 6 (model deg)
F.arm_ready = logged(o, 'arm_ready');              % N x 1 (flag)
F.cam_guard = logged(o, 'cam_guard');              % N x 6 CameraGuard [wait hold stop reason longest_dropout_s n_dropouts] (F6)
F.send_log = [];
try
    if evalin('base', 'exist(''rome_send_recorder'', ''var'')') == 1
        rec = evalin('base', 'rome_send_recorder');
        if isa(rec, 'function_handle'), F.send_log = rec(); end
    end
catch e
    fprintf('rome_run_finish: send recorder failed (%s); STOP_ALL and watchdog NOT MEASURED\n', e.message);
end
if isempty(F.send_log)
    % ROMECommand.m's send log (20 Nov 2026): every line written to the robot, dumped when the block is released.
    % Used only when written within 15 s of this run's stop, cleared once read, saved with the run as snap.send_log.
    % romecommand_sendlog = 0 (lab_settings.m) turns the log off: STOP_ALL and the watchdog are then NOT MEASURED.
    try
        if evalin('base', 'exist(''rome_send_log'', ''var'')') == 1
            SL = evalin('base', 'rome_send_log');
            evalin('base', 'clear rome_send_log');     % read once: it can never serve a later run
            age = abs(seconds(datetime(SL.dump_wall, 'InputFormat', 'yyyy-MM-dd HH:mm:ss') - ...
                              datetime(R.stop_wall, 'InputFormat', 'yyyy-MM-dd HH:mm:ss')));   % (s)
            if age <= 15
                F.send_log = SL;
            else
                fprintf('rome_run_finish: rome_send_log written %.0f s away from this run''s stop (another run''s), not used\n', age);
            end
        end
    catch e
        fprintf('rome_run_finish: send log unreadable (%s); STOP_ALL and watchdog NOT MEASURED\n', e.message);
    end
end
if ~isempty(F.send_log), R.snap.send_log = F.send_log; end   % saved with the run (save_run writes snap)
[~, C] = allowed_ranges();
F.fw_watchdog_s = C.fw_watchdog_s;  F.arm_track_tol_deg = C.arm_track_tol_deg;  F.cam_dropout_max_s = C.cam_dropout_max_s;
F.saved_file = '';  F.save_err = '';
tag = 'run';  if isnumeric(R.mode) && isscalar(R.mode) && any(R.mode == 1:4), tag = sprintf('mode%d', R.mode); end
try
    F.saved_file = save_run(o, R.snap, tag, R.dirOut);
catch e
    F.save_err = sprintf('%s: %s', e.identifier, e.message);
end
R.file = F.saved_file;
V = run_verdict_core(F);
fprintf('%s\n', V.text);
assignin('base', 'rome_last_verdict', V);
assignin('base', 'rome_last_run', R);
end

function ok = fresh_output(o, R)
% True when o is the output of the run that stopped at R.stop_wall.
ok = false;
if ~isa(o, 'Simulink.SimulationOutput'), return; end
try
    md = o.SimulationMetadata;
    if ~strcmp(md.ModelInfo.ModelName, R.mdl), return; end
    s0 = md.TimingInfo.WallClockTimestampStart;  s1 = md.TimingInfo.WallClockTimestampStop;   % (char)
    if strcmp(s0, R.prev_out_start), return; end
    dt = seconds(datetime(s1, 'InputFormat', 'yyyy-MM-dd HH:mm:ss') - datetime(R.stop_wall, 'InputFormat', 'yyyy-MM-dd HH:mm:ss'));   % (s)
    ok = dt >= -2 && dt <= 15;
catch
    ok = false;
end
end

function X = logged(o, name)
% Logged To Workspace signal name of the run output as rows N x k ([] when absent).
X = [];
try
    if ~any(strcmp(o.who, name)), return; end
    s = o.get(name);
    if isa(s, 'timeseries')
        d = s.Data;  N = numel(s.Time);
    else
        d = s.signals.values;  N = size(d, 1);
    end
    k = find(size(d) == N, 1, 'last');             % the time dimension (index)
    X = reshape(permute(d, [k, setdiff(1:ndims(d), k)]), N, []);
    X = double(X);
catch
    X = [];
end
end
