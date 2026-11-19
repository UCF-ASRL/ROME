function test_run_length()
%TEST_RUN_LENGTH  Every simulation path runs to T_stop; pacing follows EnableHardware.
%   Plan 1 Task 4. Errors if any assertion fails; all assertions are printed first.
%     A  the literal T_ON in EndEffectorTrajectory.m (embedded in the model, CLAUDE.md
%        section 27, so it keeps its literal) equals T_ON in define_constants.m; check_9dof.m
%        and run_9dof_cases.m carry no T_ON literal of their own (ruling R4.1)
%     B  define_constants.m runs with no model loaded and does not load one (ruling R4.4)
%     C  scenarios 1-5, EnableHardware = 0, the model as saved:
%          StopTime is 'T_stop'; EnablePacing is 'off';
%          sim stops at T_stop (|t(end) - T_stop| < dt);
%          the run reaches the end of the path: |t(end) - (T_scaled + T_ON/2)| < dt, with T_ON the literal
%          of EndEffectorTrajectory.m (part A), so a T_stop that is not the path end fails even when the
%          run stops exactly at T_stop (Task 7c; the Task 7b break T_stop = T_scaled passed before);
%          EndEffectorTrajectory(t(end)) equals the reference at the end of the path, 1e-9 m.
%        The end-of-path reference is written out here from the scenario equations
%        (scenario_defaults.m, verify_scenarios.m) at the reference clock T_scaled, with the
%        onset over (clock = t - T_ON/2, which is T_scaled at t = T_stop; the fixed step may
%        stop up to dt short of T_stop, e.g. 31.00 of 31.02 s on scenario 1); scenario 1 uses
%        OrbitTrajectory_ref with mu*D^3, as verify_eetraj does.
%     D  EnablePacing follows EnableHardware when a model is loaded: 1 -> 'on', 0 -> 'off',
%        each started from the opposite state so the test can fail. define_constants.m stays at
%        EnableHardware = 0 throughout (final review I2, 19 Nov 2026): D runs define_constants, then
%        sets EnableHardware in the BASE workspace (assignin) and calls apply_pacing(model), the
%        function the model's InitFcn/StartFcn and define_constants call, and reads the pacing back.
%   Takes custody of define_constants.m under the PRISTINE lock (CLAUDE.md section 20.4) to
%   select the scenario, and restores it on exit. Every text it writes has EnableHardware = 0
%   (setsc forces it), so a crash can never leave the lab file in hardware mode. Flat, no local functions
%   (CLAUDE.md section 5, crash 2). Do not run it while gate_all or check_guard runs.
here = fileparts(mfilename('fullpath'));    % ROME_merged/tests
root = fileparts(here);                     % ROME_merged
addpath(here, root, fullfile(root,'Ground Vehicle'), fullfile(root,'Optitrack'));
cd(root);
mdl  = 'ROME_9DOF';
dcf  = fullfile(root, 'define_constants.m');
lock = fullfile(root, 'define_constants.PRISTINE');
fails = {};                                 % one message per failed assertion (cell of char)

%% A  T_ON literals
rd  = @(f) fileread(fullfile(root, f));     % text of a file in ROME_merged
lit = @(txt) regexp(txt, '(?m)^\s*T_ON\s*=\s*([0-9.]+)\s*;', 'tokens');   % "T_ON = <number>;" lines
tE = lit(rd('EndEffectorTrajectory.m'));  tD = lit(rd('define_constants.m'));
vE = NaN;                                   % T_ON literal of EndEffectorTrajectory.m (s), NaN until parsed; used again in C
if numel(tE) ~= 1 || numel(tD) ~= 1
    fails{end+1} = sprintf('A: expected one literal T_ON line in each of EndEffectorTrajectory.m and define_constants.m, found %d and %d', numel(tE), numel(tD));
else
    vE = str2double(tE{1}{1});  vD = str2double(tD{1}{1});   % T_ON in each file (s)
    fprintf('A  T_ON literal: EndEffectorTrajectory.m %g s, define_constants.m %g s\n', vE, vD);
    if vE ~= vD, fails{end+1} = sprintf('A: T_ON differs: EndEffectorTrajectory.m %g, define_constants.m %g', vE, vD); end
end
for f = {'check_9dof.m', 'run_9dof_cases.m'}
    n = numel(regexp(rd(f{1}), '(?m)^\s*T_ON\w*\s*=\s*[0-9]', 'match'));   % T_ON assigned a number
    fprintf('A  %s: %d T_ON literal assignments (must be 0)\n', f{1}, n);
    if n ~= 0, fails{end+1} = sprintf('A: %s carries its own T_ON literal (%d); it must read define_constants', f{1}, n); end %#ok<AGROW>
end

%% custody of define_constants.m
assert(~isfile(lock), 'test_run_length:lock', 'define_constants.PRISTINE exists: restore it first (CLAUDE.md 20.4)');
orig = fileread(dcf);                       % unmodified text
copyfile(dcf, lock);
restore = onCleanup(@() cellfun(@(c) c(), {@() copyfile(lock, dcf, 'f'), @() delete(lock)})); %#ok<NASGU>
setsc = @(sc) regexprep(regexprep(orig, 'scenario = \d+;', sprintf('scenario = %d;', sc), 'once'), ...
                        'EnableHardware = \d;', 'EnableHardware = 0;', 'once');   % define_constants text with the scenario set and EnableHardware = 0 ALWAYS (char)

%% B  no model loaded
if bdIsLoaded(mdl), close_system(mdl, 0); end
fid = fopen(dcf, 'w'); fwrite(fid, setsc(1)); fclose(fid);
evalin('base', 'clear all');
try
    evalin('base', sprintf('run(''%s'');', dcf));
    okB = ~bdIsLoaded(mdl) && evalin('base', 'exist(''T_stop'',''var'')') == 1;
    fprintf('B  define_constants ran with no model: model loaded afterwards = %d, T_stop defined = %d\n', ...
            bdIsLoaded(mdl), evalin('base', 'exist(''T_stop'',''var'')') == 1);
    if ~okB, fails{end+1} = 'B: define_constants did not define T_stop without a model, or loaded the model'; end
catch e
    fails{end+1} = ['B: define_constants failed without a model: ' e.message];
end

%% C  scenarios 1-5
for sc = 1:5
    fid = fopen(dcf, 'w'); fwrite(fid, setsc(sc)); fclose(fid);
    evalin('base', 'clear all');
    evalin('base', sprintf('run(''%s'');', dcf));
    hw = evalin('base', 'EnableHardware');
    if hw ~= 0, fails{end+1} = sprintf('C%d: EnableHardware = %d, expected 0', sc, hw); end %#ok<AGROW>
    load_system(fullfile(root, [mdl '.slx']));       % PostLoadFcn runs define_constants in base
    set_param(mdl, 'StopFcn', '');
    stp = get_param(mdl, 'StopTime');                % StopTime as saved (char)
    pac = get_param(mdl, 'EnablePacing');            % pacing as loaded
    fprintf('C%d StopTime = ''%s'', EnablePacing = ''%s''\n', sc, stp, pac);
    if ~strcmp(stp, 'T_stop'), fails{end+1} = sprintf('C%d: StopTime is ''%s'', expected ''T_stop''', sc, stp); end %#ok<AGROW>
    if ~strcmp(pac, 'off'),    fails{end+1} = sprintf('C%d: EnablePacing is ''%s'' with EnableHardware = 0, expected ''off''', sc, pac); end %#ok<AGROW>
    if strcmp(pac, 'on'), set_param(mdl, 'EnablePacing', 'off'); end    % only so a failing run does not take real time
    if evalin('base', 'exist(''T_stop'',''var'')') ~= 1
        fails{end+1} = sprintf('C%d: T_stop is not defined in the base workspace', sc); %#ok<AGROW>
        close_system(mdl, 0);  continue
    end
    T_stop = evalin('base', 'T_stop');  dt = evalin('base', 'dt_9dof');  T_scaled = evalin('base', 'T_scaled'); %#ok<NASGU>
    evalin('base', 'clear ukd_current');
    out = sim(mdl, 'SimulationMode', 'normal', 'ReturnWorkspaceOutputs', 'on');
    t = out.q_cmd.Time;
    close_system(mdl, 0);
    fprintf('C%d sim ended at %.4f s, T_stop = %.4f s (T_scaled %.4f s, dt %.3f s)\n', sc, t(end), T_stop, evalin('base','T_scaled'), dt);
    if ~(abs(t(end) - T_stop) < dt)
        fails{end+1} = sprintf('C%d: sim ended at %.4f s, T_stop = %.4f s (|diff| %.4g >= dt %.3g)', sc, t(end), T_stop, abs(t(end) - T_stop), dt); %#ok<AGROW>
    end
    % Run length against the end of the path, independent of the T_stop variable (Task 7c). The reference clock
    % is t - T_ON/2 after the onset (EndEffectorTrajectory.m), so it reaches the path end T_scaled at
    % t = T_scaled + T_ON/2, with T_ON the function's own literal (vE, part A), not define_constants' T_stop.
    t_path = evalin('base', 'T_scaled') + vE/2;      % time at which the reference reaches the end of the path (s)
    fprintf('C%d run ended at %.4f s, path end on the reference clock %.4f s (T_scaled + T_ON/2, T_ON = %g s from EndEffectorTrajectory.m)\n', sc, t(end), t_path, vE);
    if ~(abs(t(end) - t_path) < dt)
        fails{end+1} = sprintf('C%d: run ended at %.4f s, but the reference reaches the end of the path at %.4f s (T_scaled + T_ON/2; |diff| %.4g >= dt %.3g)', sc, t(end), t_path, abs(t(end) - t_path), dt); %#ok<AGROW>
    end
    % reference at the end of the path, from the scenario equations at clock T_scaled
    els = evalin('base', 'elements'); mu_ = evalin('base', 'mu');
    ts_ = evalin('base', 'time_scale'); ds_ = evalin('base', 'dist_scale');
    zw_ = evalin('base', 'z_work'); pr_ = evalin('base', 'par');
    Tc  = t(end) - evalin('base', 'T_ON')/2;          % reference clock at the last sample (s); within dt of T_scaled, since t_end is within dt of T_stop
    tsc = ts_ * Tc;                                   % scenario time at the last sample (s)
    switch sc
        case 1, [qd_, ~, ~] = OrbitTrajectory_ref(Tc, els, mu_*ds_^3, ts_, ds_);  pend = qd_(1:2);
        case 2, pend = ds_ * [0; pr_(1) - pr_(2)*tsc];
        case 3, pend = ds_ * [pr_(1) - pr_(2)*tsc; 0];
        case 4, pend = ds_ * [pr_(1)*cos(pr_(2)*tsc + pr_(3)); -2*pr_(1)*sin(pr_(2)*tsc + pr_(3))];
        case 5, pend = ds_ * [pr_(1)*cos(pr_(2)*tsc); pr_(1)*sin(pr_(2)*tsc)];
    end
    pend = [pend(:); zw_];                            % 3x1 reference at the path end (m)
    pe = EndEffectorTrajectory(t(end), sc, els, mu_, ts_, ds_, zw_, pr_, evalin('base', 'tool_tilt_deg'));
    d = norm(pe - pend);                              % m
    fprintf('C%d |EndEffectorTrajectory(t_end) - reference at path end| = %.3e m\n', sc, d);
    if ~(d < 1e-9), fails{end+1} = sprintf('C%d: end of run is %.3e m from the end of the path (limit 1e-9)', sc, d); end %#ok<AGROW>
end

%% D  pacing follows EnableHardware
% Final review I2 (19 Nov 2026): D used to write EnableHardware = 1 into the live define_constants.m, so a
% crash inside D left the lab file in hardware mode. Now the file is written with EnableHardware = 0 only
% (setsc). Order, so each case starts from the opposite state: define_constants runs (load, then run; it sets
% pacing 'off'), THEN the opposite pacing is preset, THEN EnableHardware is set in the base workspace, THEN
% apply_pacing(mdl) runs and the pacing is read back. With EnableHardware = 1, sensor_model is set to 0 too:
% apply_pacing refuses a hardware run with SensorModel noise on (apply_pacing:sensormodel, test_pacing d7).
for hw = [1 0]
    fid = fopen(dcf, 'w'); fwrite(fid, setsc(1)); fclose(fid);
    evalin('base', 'clear all');
    load_system(fullfile(root, [mdl '.slx']));
    evalin('base', sprintf('run(''%s'');', dcf));                        % EnableHardware = 0 from the file
    opp = {'on', 'off'};  set_param(mdl, 'EnablePacing', opp{1 + hw});   % the opposite of what is expected
    assignin('base', 'EnableHardware', hw);                              % workspace only, never the file (flag)
    if hw ~= 0, assignin('base', 'sensor_model', 0); end                 % 0 exact state to the controller (flag)
    try
        apply_pacing(mdl);
    catch e
        fails{end+1} = sprintf('D: EnableHardware = %d: apply_pacing errored %s: %s', hw, e.identifier, e.message); %#ok<AGROW>
    end
    pac = get_param(mdl, 'EnablePacing');  want = opp{2 - hw};
    fprintf('D  EnableHardware = %d -> EnablePacing = ''%s'' (expected ''%s'')\n', hw, pac, want);
    if ~strcmp(pac, want), fails{end+1} = sprintf('D: EnableHardware = %d gave EnablePacing ''%s'', expected ''%s''', hw, pac, want); end %#ok<AGROW>
    close_system(mdl, 0);
end
tokD = regexp(fileread(dcf), 'EnableHardware = (\d);', 'tokens', 'once');   % the file's flag after D (cell of char)
if isempty(tokD), ftxt = '(no flag line found)'; else, ftxt = tokD{1}; end   % printable flag (char)
fprintf('D  define_constants.m on disk after D: EnableHardware = %s\n', ftxt);
if ~strcmp(ftxt, '0'), fails{end+1} = 'D: define_constants.m on disk is not at EnableHardware = 0 after D'; end

if ~isempty(fails)
    error('test_run_length:fail', '%d assertion(s) failed:\n  %s', numel(fails), strjoin(fails, sprintf('\n  ')));
end
fprintf('\ntest_run_length PASS\n');
end
