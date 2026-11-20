function arm_freeze_banner(mdl)
%ARM_FREEZE_BANNER  One line at every run start saying whether the arm will move (Plan 2 Task 5 / focus plan F1).
%   Called from the model's InitFcn, after apply_pacing: 'apply_pacing(bdroot); arm_freeze_banner(bdroot)'
%   (set by tools/wire_arm_freeze.m). Prints exactly one of
%     *** ARM FROZEN at arm_freeze_deg: q0_9dof(4:9) mapped and clamped = [..] (firmware deg) ... ***
%     *** ARM WILL MOVE: ... ***
%   It reads arm_freeze_user, the value the model's ArmFreezeSwitch uses, NOT arm_freeze: ROMECommand.m L56
%   (Keanu's, kept as is, author 19 Nov 2026) writes arm_freeze = 0 into the base workspace at every run
%   start, and define_constants copies the student's arm_freeze into arm_freeze_user as its last line (and
%   sets it to NaN at the start of its formulation section, so a define_constants that stopped partway
%   leaves NaN, which is refused here).
%   The FROZEN numbers are computed NOW from q0_9dof(4:9), with ROMECommand's own map and clamp
%   (ROMECommand.m L98-113: arm_sign .* deg + arm_offset_deg if arm_offset_deg exists, then arm_clamp with
%   arm_fw_switch / arm_fw_other / arm_fw_margin_deg if arm_fw_switch exists): what the robot is sent.
%   When define_constants' arm_freeze_deg differs (it is computed before the base_override re-solve,
%   TRACKER Q-F1-a), the line says so.
%   A run is REFUSED (error) when:
%     - arm_freeze_user is undefined (noflag) or not exactly 0 or 1 (badflag): the switch (u2 > 0.5) would
%       act on a value nobody chose;
%     - define_constants.m or its TA file lab_settings.m changed since define_constants last ran to its end (stale):
%       their combined SHA-256 now (settings_sha.m, F14a) differs from dc_sha_at_run, or dc_sha_at_run is missing
%       (fix round 2, ruling R1);
%     - run_mode is undefined (nomode), or a run flag in the base workspace (EnableHardware, EnableMotive,
%       motive_source, arm_freeze_user) differs from what run_mode gives (run_mode_table; mode): a flag typed in the
%       Command Window would run a mode nobody selected (focus plan F12, fix round 1). A flag named in the base cell
%       run_mode_test_override may differ: define_constants empties it at every run, and only the TEST OVERRIDE lines
%       that tests/dc_set_mode.m writes into temp copies (or a test that sets a flag by hand) add to it;
%     - the model is loaded and its freeze wiring is not the one tools/wire_arm_freeze.m builds (notwired):
%       ArmFreezeSwitch with Criteria 'u2 > Threshold' and Threshold 0.5, fed u1 by ArmFreezePosture
%       (Value q0_9dof(4:9)), u2 by ArmFreezeUser (Value arm_freeze_user), u3 by CommandGuard output 2, and
%       feeding ROMECommand input 2. Checked for both values of the flag, so the banner never states a
%       behaviour the wiring does not have.
%   A refusal of a robot run first prints its RUN VERDICT FAIL line (run_refused.m, focus plan F14a: Simulink calls no
%   StopFcn after an InitFcn error), then is rethrown unchanged (same identifier and message).
%   mdl  model name (char); default bdroot.
if nargin < 1, mdl = bdroot; end
try   % F14a: a refusal of a robot run prints its FAIL line (run_refused.m) before it is rethrown unchanged
try
    af = evalin('base', 'arm_freeze_user');        % 1 joint command held at q0_9dof(4:9), 0 controller's command (flag)
catch
    error('arm_freeze_banner:noflag', 'arm_freeze_user is not defined in the base workspace; run define_constants first. Run refused.');
end
if ~(isnumeric(af) && isscalar(af) && (af == 0 || af == 1))
    error('arm_freeze_banner:badflag', ['arm_freeze_user must be 0 or 1 (it is %s; NaN means define_constants did not ' ...
          'run to its end); check run_mode in define_constants.m and re-run it. Run refused.'], mat2str(af));
end
% STALE WORKSPACE (F1 fix round 2, controller ruling R1). define_constants stores the SHA-256 of its own file
% as dc_sha_at_run (dc_file_sha). If define_constants.m changed since (an edit not yet run, a user-block typo
% that stopped the run before arm_freeze_user was reset, a harness that restored the file), the workspace may
% belong to another version of the file, and the flag above with it.
shaRun = '';                                       % SHA-256 stored by the last complete define_constants (hex char)
try, shaRun = evalin('base', 'dc_sha_at_run'); catch, end
dcf = which('define_constants');                   % the file the model's PostLoadFcn would run (char)
shaNow = '';                                       % SHA-256 over it and its lab_settings.m now (hex char; settings_sha, F14a)
if ~isempty(dcf), try, shaNow = settings_sha(dcf); catch, end, end   % unreadable or missing lab_settings.m: '' (refused below)
shaOk = ischar(shaRun) && ~isempty(shaRun) && strcmp(shaRun, shaNow);   % workspace made by this file (logical)
if ~shaOk
    error('arm_freeze_banner:stale', ['define_constants.m or lab_settings.m changed since define_constants was last run; re-run it ' ...
          '(or reopen the model). Run refused. (%s and its lab_settings.m: SHA-256 now %s, at the last run %s)'], dcf, shaNow, mat2str(shaRun));
end
% RUN MODE (focus plan F12, fix round 1): the flags must be the ones run_mode gives.
try
    rm = evalin('base', 'run_mode');               % run mode (1-4)
catch
    error('arm_freeze_banner:nomode', 'run_mode is not defined in the base workspace; run define_constants first. Run refused.');
end
Mrm = run_mode_table(rm);                          % the flags run_mode gives (struct); run_mode_table:bad if not 1-4
ovr = {};                                          % flags a test lets differ (cell of char)
try, ovr = evalin('base', 'run_mode_test_override'); catch, end
if ~iscellstr(ovr), ovr = {}; end %#ok<ISCLSTR>
fl = {'EnableHardware', 'EnableHardware'; 'EnableMotive', 'EnableMotive'; 'motive_source', 'motive_source'; 'arm_freeze_user', 'arm_freeze'};   % base name | run_mode_table field
badM = {};                                         % mismatches (cell of char)
for k = 1:size(fl, 1)
    if any(strcmp(ovr, fl{k,2})), continue; end
    try, gv = evalin('base', fl{k,1}); catch, gv = 'undefined'; end
    if ~isequal(gv, Mrm.(fl{k,2}))
        badM{end+1} = sprintf('%s is %s, run_mode %d gives %s', fl{k,1}, mat2str(gv), Mrm.n, mat2str(Mrm.(fl{k,2}))); %#ok<AGROW>
    end
end
if ~isempty(badM)
    error('arm_freeze_banner:mode', ['the run flags do not match run_mode %d (%s): %s. Choose the mode with run_mode in ' ...
          'define_constants.m (or rome_mode(n)) and re-run it; do not type the flags. Run refused.'], Mrm.n, Mrm.banner, strjoin(badM, '; '));
end
% FROZEN ARM (Q-F7-a). A refusal of arm_freeze_user = 1 with EnableHardware = 1 and EnableMotive = 1
% (frozenfeedback, commit a6fb2a4) was REMOVED on the author's decision of 19 Nov ("Arm from model if frozen"):
% with the arm frozen the controller now reads the arm angles from its own model (JointSource on SimJoints,
% tools/wire_motive_emulator.m), not from the parked telemetry that made it wind up (1657 deg, F7 run 1).
if ~isempty(mdl) && bdIsLoaded(mdl)
    SW = [mdl '/ArmFreezeSwitch'];                 % block paths (char)
    PO = [mdl '/ArmFreezePosture'];
    US = [mdl '/ArmFreezeUser'];
    prob = {};                                     % wiring problems found (cell of char)
    if getSimulinkBlockHandle(SW) == -1
        prob{end+1} = 'no ArmFreezeSwitch';
    else
        if ~strcmp(get_param(SW, 'Criteria'), 'u2 > Threshold') || ~strcmp(strtrim(get_param(SW, 'Threshold')), '0.5')
            prob{end+1} = sprintf('switch criterion ''%s'' threshold ''%s'' (want u2 > Threshold, 0.5)', get_param(SW, 'Criteria'), get_param(SW, 'Threshold'));
        end
        want = {SW, 1, [PO '/1']; SW, 2, [US '/1']; SW, 3, [mdl '/CommandGuard/2']; [mdl '/JointGate'], 1, [SW '/1']; [mdl '/ROMECommand'], 2, [mdl '/JointGate/1']};   % dst, input, src/port (wire3 fix 1: JointGate between the switch and ROMECommand; it passes the switch output, the parked posture q0_9dof(4:9) or the last command held)
        for k = 1:size(want, 1)
            p = get_param(want{k,1}, 'PortConnectivity');
            s = p(want{k,2});
            got = '<none>';                        % source of that input, 'block/port' (char)
            if ~isempty(s.SrcBlock) && all(s.SrcBlock ~= -1), got = sprintf('%s/%d', getfullname(s.SrcBlock), s.SrcPort + 1); end
            if ~strcmp(got, want{k,3}), prob{end+1} = sprintf('%s input %d fed by %s (want %s)', want{k,1}, want{k,2}, got, want{k,3}); end %#ok<AGROW>
        end
        if getSimulinkBlockHandle(PO) ~= -1 && ~strcmp(get_param(PO, 'Value'), 'q0_9dof(4:9)')
            prob{end+1} = sprintf('ArmFreezePosture Value ''%s'' (want q0_9dof(4:9))', get_param(PO, 'Value'));
        end
        if getSimulinkBlockHandle(US) ~= -1 && ~strcmp(get_param(US, 'Value'), 'arm_freeze_user')
            prob{end+1} = sprintf('ArmFreezeUser Value ''%s'' (want arm_freeze_user)', get_param(US, 'Value'));
        end
    end
    if ~isempty(prob)
        error('arm_freeze_banner:notwired', ['the arm-freeze wiring of %s is not the one tools/wire_arm_freeze.m builds (%s), ' ...
              'so the banner cannot say what the arm will do. Run tools/wire_arm_freeze.m. Run refused.'], mdl, strjoin(prob, '; '));
    end
end
try
    eh = evalin('base', 'EnableHardware');         % 0 simulate only, 1 command the real robot (flag)
catch
    eh = NaN;                                      % not defined: apply_pacing has already refused such a run
end
fprintf('%s\n', Mrm.banner);                      % the MODE line, at every Run (focus plan F12, fix round 1)
if ~isempty(ovr), fprintf('TEST OVERRIDE (run_mode_test_override): %s may differ from the mode\n', strjoin(ovr, ', ')); end
if af == 1
    q0 = evalin('base', 'q0_9dof');                % 9x1 start configuration (m, m, rad, 6 joints rad)
    fw = rad2deg(q0(4:9)).';                       % 1x6 start joints (model deg)
    if evalin('base', 'exist(''arm_offset_deg'', ''var'')')
        fw = evalin('base', 'arm_sign(:).''') .* fw + evalin('base', 'arm_offset_deg(:).''');   % firmware deg, ROMECommand L101-103
    end
    if evalin('base', 'exist(''arm_fw_switch'', ''var'')')
        fw = arm_clamp(fw, evalin('base', 'arm_fw_switch'), evalin('base', 'arm_fw_other'), evalin('base', 'arm_fw_margin_deg'));   % ROMECommand L111-113
    end
    note = '';                                     % stale arm_freeze_deg note (char)
    try
        afd = evalin('base', 'arm_freeze_deg');    % define_constants' copy (firmware deg)
        if numel(afd) ~= 6 || max(abs(afd(:).' - fw)) > 1e-9
            note = sprintf(' (define_constants'' arm_freeze_deg %s differs: computed before the base_override re-solve)', mat2str(round(afd(:).' * 1000) / 1000));
        end
    catch
        note = ' (arm_freeze_deg not defined)';
    end
    fprintf('*** ARM FROZEN at arm_freeze_deg: q0_9dof(4:9) mapped and clamped = [%s] (firmware deg); arm_freeze_user = 1, %s sends this start posture for the whole run (EnableHardware = %g)%s ***\n', ...
            strtrim(sprintf('%.3f ', fw)), mdl, eh, note);
else
    fprintf('*** ARM WILL MOVE: arm_freeze_user = 0, %s sends the controller''s joint commands (EnableHardware = %g). To park the arm use run_mode 2 (rome_mode(2)) ***\n', ...
            mdl, eh);
end
catch err
    run_refused(err);                              % the PASS/FAIL line of a refused robot run (F14a)
    rethrow(err);
end
end
