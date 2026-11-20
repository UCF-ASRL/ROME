function rome_run_end(mdl)
%ROME_RUN_END  The model's StopFcn (focus plan F14a, 19 Nov 2026; set by tools/wire_run_verdict.m). Never errors.
%   At the end of EVERY run (normal end, error stop, refusal in StartFcn; NOT after an InitFcn refusal, where Simulink
%   runs no StopFcn: those print their verdict line from run_refused.m) it records, in the base workspace,
%     rome_last_run   struct: mdl, t_end (simulation time at the stop, s), stop_wall ('yyyy-MM-dd HH:mm:ss'),
%                     robot (EnableHardware ~= 0), mode, T_stop, dt (dt_9dof), max_rpm, snap (collect_run_snapshot of
%                     the define_constants that the run used, taken NOW: a later rome_mode(n) does not change it),
%                     dirOut (<folder of save_run>/runs_lab), prev_out_start (start stamp of the run output already in
%                     the base, '' if none), out_start ('' until the run's output is seen), file ('' until saved)
%   save_run() (no arguments) uses it to save the last run with the snapshot of its end.
%   ROBOT RUNS (EnableHardware ~= 0) only: Simulink assigns the run output (base variable out, ReturnWorkspaceOutputs
%   on, measured 19 Nov: also after an error stop, with ErrorMessage) around this callback, before or after it (both
%   seen on the Run-button path, 19 Nov), never readable as logged data from inside it, so a timer
%   (rome_run_finish.m, every 0.2 s for up to 10 s) waits for that new output, then saves the run (save_run, into
%   runs_lab) and prints the PASS/FAIL line (run_verdict_core.m). A run started by sim() from code returns its output
%   to the caller instead: no timer is started for it (rome_last_run.by_code = true; detected from the call stack),
%   and a timer that still finds no new output ends silently after 10 s.
%   mdl  model name (char); default bdroot.
try
    if nargin < 1 || isempty(mdl), mdl = bdroot; end
    R = struct('mdl', mdl, 't_end', NaN, 'stop_wall', '', 'robot', false, 'mode', NaN, 'T_stop', NaN, 'dt', NaN, ...
               'max_rpm', NaN, 'snap', struct(), 'dirOut', '', 'prev_out_start', '', 'out_start', '', 'file', '');
    try, R.t_end = get_param(mdl, 'SimulationTime'); catch, end
    d = datetime('now');  d.Format = 'yyyy-MM-dd HH:mm:ss';  R.stop_wall = char(d);
    eh = bv('EnableHardware', NaN);                % 0 simulate only, 1 command the real robot (flag)
    R.robot = isnumeric(eh) && isscalar(eh) && eh ~= 0;
    R.mode = bv('run_mode', NaN);  R.T_stop = bv('T_stop', NaN);  R.dt = bv('dt_9dof', NaN);  R.max_rpm = bv('max_rpm', NaN);
    dcf = which('define_constants');               % the define_constants this run used (char)
    try
        R.snap = collect_run_snapshot(dcf);
    catch e
        R.snap = struct('snapshot_error', e.message);
    end
    R.dirOut = fullfile(fileparts(which('save_run')), 'runs_lab');
    o = bv('out', []);                             % run output in the base now
    if isa(o, 'Simulink.SimulationOutput')
        % On the Run-button (asynchronous) path Simulink may assign THIS run's output before the StopFcn (measured 19 Nov
        % 2026: its stop stamp equals the StopFcn's second). An output that stopped more than 2 s ago is an earlier
        % run's: its start stamp is recorded so the timer never takes it; one that stopped within 2 s is this run's.
        try
            ti = o.SimulationMetadata.TimingInfo;
            age = seconds(datetime(R.stop_wall, 'InputFormat', 'yyyy-MM-dd HH:mm:ss') - ...
                          datetime(ti.WallClockTimestampStop, 'InputFormat', 'yyyy-MM-dd HH:mm:ss'));   % (s)
            if age > 2, R.prev_out_start = ti.WallClockTimestampStart; end
        catch
        end
    end
    % A run started by sim() from code (tests, scripts) returns its output to its caller: no new base out will appear,
    % so no timer is started for it (it would outlive the caller, e.g. a test's deleted temporary copy). Detected
    % from the call stack: the frame that called into Simulink is a line of code containing "sim(". The Run button
    % (and set_param(mdl, 'SimulationCommand', 'start')) leaves no such frame.
    R.by_code = false;
    st = dbstack('-completenames');
    if numel(st) >= 2
        try
            Lc = splitlines(fileread(st(2).file));
            R.by_code = st(2).line <= numel(Lc) && contains(Lc{st(2).line}, 'sim(');
        catch
        end
    end
    assignin('base', 'rome_last_run', R);
    if R.robot && ~R.by_code
        t = timer('Name', 'rome_run_finish', 'ExecutionMode', 'fixedSpacing', 'Period', 0.2, 'TasksToExecute', 50, ...
                  'BusyMode', 'drop', 'UserData', R, 'TimerFcn', @rome_run_finish, 'StopFcn', @(tt, ~) delete(tt));
        start(t);
    end
catch e
    fprintf('rome_run_end: %s (no run record, no verdict for this run)\n', e.message);
end
end

function v = bv(n, d)
% Base-workspace variable n, or d when it does not exist.
try
    if evalin('base', sprintf('exist(''%s'', ''var'')', n)) == 1, v = evalin('base', n); else, v = d; end
catch
    v = d;
end
end
