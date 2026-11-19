function test_pacing()
%TEST_PACING  Real-time pacing follows EnableHardware at every run start (Task 4b).
%   NO NETWORK, EVER. The test runs a TEMPORARY COPY of ROME_merged (tempname) in which
%       ROMECommand.m           is replaced by tests/stubs/ROMECommand.m   (same ports, no tcpclient)
%       Optitrack/MotiveData.m  is replaced by tests/stubs/MotiveData.m    (same ports, no socket)
%   and asserts, before any run, that the copy's two files contain no tcpclient / MulticastSocket /
%   serial call and that `which` resolves both classes inside the copy. The real ROME_merged model
%   and classes are never loaded here. EnableHardware = 1 therefore only drives the stub.
%   Cases (each simulation is run in the copy, EnableHardware set in the base workspace WITHOUT
%   running define_constants, the way a student would). "Callbacks" are the model's InitFcn and
%   StartFcn, both apply_pacing(bdroot).
%     a  EnableHardware = 1: after the run EnablePacing is 'on' (PacingRate 1) and the run took
%        about real time (wall >= 0.9 x simulated time). A change made in InitFcn applies to the
%        same run: measured in docs/verification/2026-11-19_task4b_callback_order_experiment.log.
%     b  EnableHardware = 0 with pacing set 'on' in the loaded model first: after the run pacing
%        is 'off' and the run was fast (wall < 0.5 x simulated time; the wall time includes about
%        5 s of compilation, hence the long simulated time). Fails if the 'off' branch is broken.
%     c  make-it-fail of a: callbacks emptied in the copy, the check of (a) must FAIL.
%     c2 make-it-fail of b: callbacks emptied, pacing preset 'on', the check of (b) must FAIL.
%     d  apply_pacing refuses with error(): EnableHardware = 1 and pacing cannot be set (model not
%        loaded); EnableHardware undefined; not a numeric scalar; EnableHardware = 1 or undefined
%        with no model in context (empty model name). With EnableHardware = 0 and no model it
%        returns silently (define_constants standalone).
%     f  Fast Restart on, run 1 with EnableHardware = 0, then EnableHardware = 1 and run 2: run 2
%        must be refused (the measured bypass, docs/verification/2026-11-19_task4b_fix1_fastrestart_experiment.log:
%        InitFcn runs once, StartFcn on every run, run 2 was unpaced).
%     g  make-it-fail of f: callbacks emptied, run 2 is NOT refused, so the check of (f) must FAIL.
%     d7 (final review, 19 Nov 2026) EnableHardware = 1 set in the workspace with define_constants' sensor_model
%        = 1 still there (SensorModel noise on): sim refused and apply_pacing:sensormodel, also with sensor_model
%        undefined; with sensor_model = 0 it passes (positive control).
%     d8 (final review) define_constants evaluated IN MEMORY with EnableHardware = 1 (no file written): it
%        forces sensor_model = 0 ahead of its own apply_pacing call, prints the notice, pacing 'on'.
%   Cases a, c, f and g set sensor_model = 0 with EnableHardware = 1, because apply_pacing refuses a hardware
%   run with SensorModel noise on (d7); each case keeps testing only its own property.
%   Flat function without local functions (CLAUDE.md section 5, crash 2). Clears the base
%   workspace like the other harnesses.
here = fileparts(mfilename('fullpath'));    % ROME_merged/tests
root = fileparts(here);                     % ROME_merged
tmp  = tempname;                            % temporary copy of ROME_merged (char)
mdl  = 'ROME_9DOF';
startdir = pwd;                             % restored at exit
path0 = path;                               % MATLAB path at entry, restored at exit
if bdIsLoaded(mdl), close_system(mdl, 0); end
mkdir(tmp);
copyfile(root, tmp);
copyfile(fullfile(here, 'stubs', 'ROMECommand.m'), fullfile(tmp, 'ROMECommand.m'), 'f');
copyfile(fullfile(here, 'stubs', 'MotiveData.m'),  fullfile(tmp, 'Optitrack', 'MotiveData.m'), 'f');
mdlcell = {mdl};                            % the model name as a cell, for the conditional close below
cleanup = onCleanup(@() cellfun(@(c) c(), {@() cd(startdir), @() path(path0), ...
          @() cellfun(@(m) set_param(m, 'FastRestart', 'off'), mdlcell(bdIsLoaded(mdl))), ...
          @() cellfun(@(m) close_system(m, 0), mdlcell(bdIsLoaded(mdl))), @() rmdir(tmp, 's')})); %#ok<NASGU>
addpath(tmp, fullfile(tmp, 'Ground Vehicle'), fullfile(tmp, 'Optitrack'));
cd(tmp);
evalin('base', 'clear all; clear classes');
fails = {};                                 % one message per failed assertion (cell of char)

% no-network guard: the stubs are in place and nothing else can be resolved
for f = {fullfile(tmp, 'ROMECommand.m'), fullfile(tmp, 'Optitrack', 'MotiveData.m')}
    txt = fileread(f{1});                   % stub source (char)
    bad = regexp(txt, '(tcpclient|MulticastSocket|DatagramSocket|serialport|serial\(|tcpip|udpport|natnet)\s*\(', 'match');
    assert(isempty(bad), 'test_pacing:network', 'stub %s contains a network call: %s', f{1}, strjoin(bad, ', '));
end
assert(strcmp(fileparts(which('ROMECommand')), tmp), 'test_pacing:stub', 'ROMECommand resolves to %s, not the stub in the copy', which('ROMECommand'));
assert(strcmp(fileparts(which('MotiveData')), fullfile(tmp, 'Optitrack')), 'test_pacing:stub', 'MotiveData resolves to %s, not the stub in the copy', which('MotiveData'));
fprintf('guard: ROMECommand -> %s\n       MotiveData  -> %s\n', which('ROMECommand'), which('MotiveData'));

Tlong = 12;                                 % simulated time for the paced cases (s)
Tfast = 30;                                 % simulated time for the unpaced case (s)
Tb2   = 20;                                 % simulated time of the make-it-fail of b (paced, s)
% case, EnableHardware, callbacks kept?, simulated time (s), pacing preset in the loaded model, check expected to pass?
runs = {'a',  1, true,  Tlong, 'off', true;
        'c',  1, false, Tlong, 'off', false;
        'b',  0, true,  Tfast, 'on',  true;
        'c2', 0, false, Tb2,   'on',  false};
for r = 1:size(runs, 1)
    cs = runs{r,1};  hwv = runs{r,2};  keep = runs{r,3};  Tsim = runs{r,4};  preset = runs{r,5};  expect = runs{r,6};
    evalin('base', 'clear all');
    load_system(fullfile(tmp, [mdl '.slx']));   % PostLoadFcn = define_constants: EnableHardware 0, pacing off
    set_param(mdl, 'StopFcn', '');
    if keep
        assert(contains(get_param(mdl, 'InitFcn'), 'apply_pacing') && contains(get_param(mdl, 'StartFcn'), 'apply_pacing'), ...
               'test_pacing:callback', 'the model InitFcn/StartFcn are ''%s'' / ''%s'', expected apply_pacing', get_param(mdl, 'InitFcn'), get_param(mdl, 'StartFcn'));
    else
        set_param(mdl, 'InitFcn', '');  set_param(mdl, 'StartFcn', '');   % callbacks disabled in the copy
    end
    set_param(mdl, 'EnablePacing', preset);     % the state the model is in when the student presses Run
    assignin('base', 'EnableHardware', hwv);    % NOT through define_constants
    if hwv ~= 0, assignin('base', 'sensor_model', 0); end   % 0 exact state (flag): apply_pacing refuses a hardware run with SensorModel noise on (tested in d7)
    evalin('base', 'clear ukd_current');
    t0 = tic;
    sim(mdl, 'StopTime', num2str(Tsim), 'SimulationMode', 'normal', 'ReturnWorkspaceOutputs', 'on');
    wall = toc(t0);                             % wall time of the run incl. compilation (s)
    pac  = get_param(mdl, 'EnablePacing');      % pacing as the run left it (char)
    rate = get_param(mdl, 'PacingRate');        % x
    if hwv ~= 0
        ok = strcmp(pac, 'on') && rate == 1 && wall >= 0.9 * Tsim;
    else
        ok = strcmp(pac, 'off') && wall < 0.5 * Tsim;
    end
    fprintf('%-2s EnableHardware=%d callbacks kept=%d preset=%s: EnablePacing=%s rate=%g, simulated %g s, wall %.2f s -> check %s (expected %s)\n', ...
            cs, hwv, keep, preset, pac, rate, Tsim, wall, mat2str(ok), mat2str(expect));
    close_system(mdl, 0);
    if ok ~= expect
        fails{end+1} = sprintf('%s: check gave %d, expected %d (EnableHardware=%d, callbacks kept=%d, pacing %s, wall %.2f s for %g s)', cs, ok, expect, hwv, keep, pac, wall, Tsim); %#ok<AGROW>
    end
end

% f, g: Fast Restart. Run 1 with EnableHardware = 0, then EnableHardware = 1 and run 2.
frruns = {'f', true, true; 'g', false, false};   % case, callbacks kept?, refusal expected?
for r = 1:size(frruns, 1)
    cs = frruns{r,1};  keep = frruns{r,2};  expect = frruns{r,3};
    evalin('base', 'clear all');
    load_system(fullfile(tmp, [mdl '.slx']));
    set_param(mdl, 'StopFcn', '');
    if ~keep, set_param(mdl, 'InitFcn', ''); set_param(mdl, 'StartFcn', ''); end
    set_param(mdl, 'FastRestart', 'on');
    assignin('base', 'EnableHardware', 0);
    evalin('base', 'clear ukd_current');
    sim(mdl, 'StopTime', num2str(Tlong));       % run 1, simulation only
    assignin('base', 'EnableHardware', 1);      % NOT through define_constants
    assignin('base', 'sensor_model', 0);        % 0 exact state (flag), so only the Fast Restart refusal is under test here
    refused = false;  msg = '';                 % run 2 refused? and the error text (with causes)
    t0 = tic;
    try
        sim(mdl, 'StopTime', num2str(Tlong));
    catch e
        refused = true;  msg = e.message;
        for c = 1:numel(e.cause), msg = [msg ' | ' e.cause{c}.message]; end %#ok<AGROW>
    end
    wall = toc(t0);                             % wall time of run 2 (s)
    pac  = get_param(mdl, 'EnablePacing');
    fprintf('%s  Fast Restart, run 2 with EnableHardware=1, callbacks kept=%d: refused=%d, EnablePacing=%s, wall %.2f s for %g s simulated\n', ...
            cs, keep, refused, pac, wall, Tlong);
    if refused, fprintf('   message: %s\n', strtrim(msg(1:min(end, 300)))); end
    set_param(mdl, 'FastRestart', 'off');
    close_system(mdl, 0);
    ok = refused && contains(msg, 'Fast Restart');   % refused for the right reason
    if ok ~= expect
        fails{end+1} = sprintf('%s: Fast Restart run 2 refused=%d (reason ok=%d), expected %d', cs, refused, ok, expect); %#ok<AGROW>
    end
end

% d7: SensorModel noise on hardware (final review I5(4)(b), 19 Nov 2026). The copy's model is loaded, so
% define_constants ran with the file's EnableHardware = 0 and left sensor_model = 1; then EnableHardware = 1
% is set in the workspace only, the way a student who flips just that flag would. The run must be refused for
% that reason: sim() errors with 'sensor_model' in its message (or a cause), and apply_pacing on the loaded
% model raises apply_pacing:sensormodel, with sensor_model 1 and with it undefined. Positive control: with
% sensor_model = 0 apply_pacing returns and pacing reads 'on', so the check does not refuse every hardware run.
% MAKE-IT-FAIL (Phase 2, makefail_campaign row tp_sensormodel): apply_pacing.m "    if ~smok" -> "    if false";
% d7 must then FAIL (sim runs 1 s against the stub, no refusal, empty identifiers).
evalin('base', 'clear all');
load_system(fullfile(tmp, [mdl '.slx']));       % PostLoadFcn = define_constants: EnableHardware 0, sensor_model 1
set_param(mdl, 'StopFcn', '');
sm0 = evalin('base', 'sensor_model');           % sensor_model as define_constants left it (flag), expected 1
assignin('base', 'EnableHardware', 1);          % NOT through define_constants
evalin('base', 'clear ukd_current');
refused = false;  msg = '';                     % run refused? and the error text with two levels of causes (char)
try
    sim(mdl, 'StopTime', '1');
catch e
    refused = true;  msg = e.message;
    for c = 1:numel(e.cause)
        msg = [msg ' | ' e.cause{c}.message]; %#ok<AGROW>
        for c2 = 1:numel(e.cause{c}.cause), msg = [msg ' | ' e.cause{c}.cause{c2}.message]; end %#ok<AGROW>
    end
end
id7 = {'', '', ''};                             % error identifier of apply_pacing(mdl) with sensor_model 1, undefined, 0 (cell of char)
for k = 1:3
    if k == 2, evalin('base', 'clear sensor_model'); end
    if k == 3, assignin('base', 'sensor_model', 0);  set_param(mdl, 'EnablePacing', 'off'); end
    try
        apply_pacing(mdl);
    catch e
        id7{k} = e.identifier;
    end
end
pac7 = get_param(mdl, 'EnablePacing');          % pacing after the positive control (char), expected 'on'
close_system(mdl, 0);
fprintf('d7 sensor_model after define_constants = %g; sim with EnableHardware = 1: refused = %d; apply_pacing ids: sensor_model 1 ''%s'', undefined ''%s'', 0 ''%s'' (pacing %s)\n', ...
        sm0, refused, id7{1}, id7{2}, id7{3}, pac7);
if refused, fprintf('   message: %s\n', strtrim(msg(1:min(end, 400)))); end
ok7 = sm0 == 1 && refused && contains(msg, 'sensor_model') && strcmp(id7{1}, 'apply_pacing:sensormodel') ...
      && strcmp(id7{2}, 'apply_pacing:sensormodel') && isempty(id7{3}) && strcmp(pac7, 'on');   % logical
if ~ok7
    fails{end+1} = sprintf('d7: hardware run with SensorModel noise on not refused as required (sensor_model %g, sim refused %d, ids ''%s'' ''%s'' ''%s'', pacing %s)', ...
                           sm0, refused, id7{1}, id7{2}, id7{3}, pac7); %#ok<AGROW>
end

% d8: define_constants itself, with EnableHardware = 1, forces sensor_model = 0 BEFORE its own apply_pacing call
% (final review I5(4)(a)). NO FILE IS WRITTEN: the copy's define_constants.m text is changed in memory (exactly
% one 'EnableHardware = 0;' asserted) and evaluated in the base workspace with the copy's model loaded, so
% define_constants' apply_pacing call runs; it would refuse (apply_pacing:sensormodel) if the override were
% missing or below it. Asserts: no error, sensor_model = 0, the one-line notice printed, pacing 'on'.
% MAKE-IT-FAIL (Phase 2, makefail_campaign row tp_dc_force): define_constants.m override line
% "    sensor_model = 0;  % forced" -> "    sensor_model = 1;  % forced"; d8 must then FAIL.
dctxt = fileread(fullfile(tmp, 'define_constants.m'));   % the copy's define_constants text (char)
nEH   = numel(strfind(dctxt, 'EnableHardware = 0;'));     % matches of the user-block flag line (count), must be 1
evalin('base', 'clear all');
load_system(fullfile(tmp, [mdl '.slx']));                 % PostLoadFcn = define_constants from the file (EnableHardware 0)
set_param(mdl, 'StopFcn', '');
set_param(mdl, 'EnablePacing', 'off');                    % the opposite of what EnableHardware = 1 must give
printed = '';  err8 = '';                                 % define_constants output (char), error text (char)
if nEH == 1
    assignin('base', 'dc_text_hw1', strrep(dctxt, 'EnableHardware = 0;', 'EnableHardware = 1;'));   % in memory only
    try
        printed = evalin('base', 'evalc(dc_text_hw1)');
    catch e
        err8 = [e.identifier ' ' e.message];
    end
end
sm8  = evalin('base', 'sensor_model');                   % sensor_model after the in-memory run (flag), expected 0
pac8 = get_param(mdl, 'EnablePacing');                   % expected 'on' (define_constants' own apply_pacing call)
close_system(mdl, 0);
evalin('base', 'clear dc_text_hw1');
note8 = contains(printed, 'sensor_model is forced to 0'); % the one-line notice was printed (logical)
fprintf('d8 define_constants in memory with EnableHardware = 1 (flag line matches %d): error ''%s'', sensor_model = %g, notice printed %d, pacing %s\n', ...
        nEH, err8, sm8, note8, pac8);
if ~(nEH == 1 && isempty(err8) && sm8 == 0 && note8 && strcmp(pac8, 'on'))
    fails{end+1} = sprintf('d8: define_constants with EnableHardware = 1 did not force sensor_model = 0 ahead of apply_pacing (flag line matches %d, error ''%s'', sensor_model %g, notice %d, pacing %s)', ...
                           nEH, err8, sm8, note8, pac8); %#ok<AGROW>
end

% d: refusals
evalin('base', 'clear all');
assignin('base', 'EnableHardware', 1);
try
    apply_pacing('no_such_model_for_test');  fails{end+1} = 'd1: no error although pacing could not be set with EnableHardware = 1'; %#ok<AGROW>
catch e
    fprintf('d1 refused: %s | %s\n', e.identifier, e.message);
    if ~strcmp(e.identifier, 'apply_pacing:cannot_confirm'), fails{end+1} = ['d1: wrong error ' e.identifier]; end %#ok<AGROW>
end
evalin('base', 'clear all');
try
    apply_pacing('no_such_model_for_test');  fails{end+1} = 'd2: no error with EnableHardware undefined'; %#ok<AGROW>
catch e
    fprintf('d2 refused: %s\n', e.identifier);
    if ~strcmp(e.identifier, 'apply_pacing:noflag'), fails{end+1} = ['d2: wrong error ' e.identifier]; end %#ok<AGROW>
end
assignin('base', 'EnableHardware', [1 0]);
try
    apply_pacing('no_such_model_for_test');  fails{end+1} = 'd3: no error with a non-scalar EnableHardware'; %#ok<AGROW>
catch e
    fprintf('d3 refused: %s\n', e.identifier);
    if ~strcmp(e.identifier, 'apply_pacing:badflag'), fails{end+1} = ['d3: wrong error ' e.identifier]; end %#ok<AGROW>
end
assignin('base', 'EnableHardware', 1);
try
    apply_pacing('');  fails{end+1} = 'd4: no error with EnableHardware = 1 and no model in context'; %#ok<AGROW>
catch e
    fprintf('d4 refused: %s\n', e.identifier);
    if ~strcmp(e.identifier, 'apply_pacing:nomodel'), fails{end+1} = ['d4: wrong error ' e.identifier]; end %#ok<AGROW>
end
evalin('base', 'clear all');
try
    apply_pacing('');  fails{end+1} = 'd5: no error with EnableHardware undefined and no model in context'; %#ok<AGROW>
catch e
    fprintf('d5 refused: %s\n', e.identifier);
    if ~strcmp(e.identifier, 'apply_pacing:noflag'), fails{end+1} = ['d5: wrong error ' e.identifier]; end %#ok<AGROW>
end
assignin('base', 'EnableHardware', 0);
try
    apply_pacing('');  fprintf('d6 EnableHardware = 0, no model: silent return, as intended\n');
catch e
    fails{end+1} = ['d6: define_constants-standalone path errored: ' e.identifier]; %#ok<AGROW>
end

if ~isempty(fails)
    error('test_pacing:fail', '%d assertion(s) failed:\n  %s', numel(fails), strjoin(fails, sprintf('\n  ')));
end
fprintf('\ntest_pacing PASS\n');
end
