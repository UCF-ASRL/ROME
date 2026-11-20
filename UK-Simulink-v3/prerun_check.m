function [ok, reasons, info] = prerun_check(mdl, banner_error)
%PRERUN_CHECK  Pre-run check of a robot run at the model's InitFcn (focus plan F2 + F11, wiring of prerun_check_core.m).
%   prerun_check(mdl)          the model's InitFcn form, third after apply_pacing and arm_freeze_banner:
%                                'apply_pacing(bdroot); arm_freeze_banner(bdroot); prerun_check(bdroot)'
%                              (tools/wire_prerun_check.m). A robot run (EnableHardware ~= 0) is checked by
%                              prerun_check_core.m; EVERY failing reason is printed, one per line, then the run is
%                              REFUSED: the RUN VERDICT FAIL line (run_refused.m) and error prerun_check:refused, whose
%                              message lists every reason. A passing robot run prints one PRERUN CHECK PASS line with
%                              the limits that are NOT enforced (allowed_ranges.m rows UNSOURCED / NOT ENFORCED).
%                              A simulation (EnableHardware = 0) prints nothing and is never refused.
%   [ok, reasons, info] = prerun_check(mdl, banner_error)   returns instead of erroring (tests, no printing).
%     banner_error  '' when arm_freeze_banner passed (the InitFcn order guarantees it: it is called first and its own
%                   refusal stops the InitFcn); in the returning form the caller passes it.
%
%   Inputs of the core, all read NOW from the base workspace, the model and the files (the core reads nothing):
%     values         every allowed_ranges.m ws_name present in the base workspace (missing ones are refused by the core)
%     pacing         get_param(mdl): EnablePacing, PacingRate, FastRestart
%     dc_sha_at_run  base (define_constants.m:340); dc_sha_now = settings_sha(which('define_constants')) (both files)
%     release        version('-release')
%     signoff        partF / partC = base signoff_partF / signoff_partC, set by the TA in lab_settings.m ('' = not signed)
%     frozen_fw_deg  q0_9dof(4:9) through arm_sign / arm_offset_deg and arm_clamp: arm_freeze_banner.m:127-134, the
%                    posture calibrateROMEArm parks the arm at and a frozen run sends
%     park_record    arm_park_record('read', <folder of define_constants>/lab/arm_park_record.mat), written by
%                    calibrateROMEArm.m at the end of a calibration whose encoder joints read at target
%   PARK IN EVERY ROBOT MODE. The core checks the park record only for a frozen arm (mode 2). The author, on a Mode
%   button pressed after calibration: "We need to recalibrate no matter what". Every robot mode starts the arm at
%   the same start posture q0_9dof(4:9) that calibrateROMEArm parks it at, so for an arm that moves (modes 3, 4) the
%   same park item is evaluated (the core called once more with arm_freeze_user = 1, only its 'park:' reasons kept):
%   no record, a record made before an edit of define_constants.m or lab_settings.m (a Mode button rewrites
%   define_constants.m), or a parked posture other than the start posture refuses the run in modes 2, 3 and 4.
%   LIMIT (prerun_check_core.m): the record certifies the command calibrateROMEArm sent, not the arm's posture now;
%   an arm moved by hand, a jog or a power cycle after calibration is not detected.
if nargin < 1 || isempty(mdl), mdl = bdroot; end
given = nargin >= 2;                               % returning form (tests): no printing, no error (logical)
[T, ~] = allowed_ranges();
S = struct();
V = struct();                                      % user-block values by ws_name (struct)
for i = 1:height(T)
    ws = T.ws_name{i};
    if isempty(ws) || isfield(V, ws), continue; end
    try
        if evalin('base', sprintf('exist(''%s'', ''var'')', ws)) == 1, V.(ws) = evalin('base', ws); end
    catch
    end
end
S.values = V;
P = struct('EnablePacing', '', 'PacingRate', NaN, 'FastRestart', '');   % model pacing settings (struct)
try, P.EnablePacing = get_param(mdl, 'EnablePacing'); catch, end
try, P.PacingRate = get_param(mdl, 'PacingRate'); catch, end
try, P.FastRestart = get_param(mdl, 'FastRestart'); catch, end
if ischar(P.PacingRate) || isstring(P.PacingRate), P.PacingRate = str2double(P.PacingRate); end
S.pacing = P;
if given
    S.banner_error = banner_error;
else
    S.banner_error = '';                           % InitFcn form: arm_freeze_banner ran just before and passed
end
S.dc_sha_at_run = bv('dc_sha_at_run', '');
dcf = which('define_constants');                   % the define_constants the model runs (char)
S.dc_sha_now = '';  try, S.dc_sha_now = settings_sha(dcf); catch, end
S.release = version('-release');
S.signoff = struct('partF', bv('signoff_partF', ''), 'partC', bv('signoff_partC', ''));
fw = NaN(1, 6);                                    % start posture as sent to the firmware (deg)
try
    q0 = evalin('base', 'q0_9dof');                % 9x1 start configuration (m, m, rad, 6 joints rad)
    fw = rad2deg(q0(4:9)).';                       % model deg
    if evalin('base', 'exist(''arm_offset_deg'', ''var'')')
        fw = evalin('base', 'arm_sign(:).''') .* fw + evalin('base', 'arm_offset_deg(:).''');   % firmware deg (ROMECommand map)
    end
    if evalin('base', 'exist(''arm_fw_switch'', ''var'')')
        fw = arm_clamp(fw, evalin('base', 'arm_fw_switch'), evalin('base', 'arm_fw_other'), evalin('base', 'arm_fw_margin_deg'));
    end
catch
end
S.frozen_fw_deg = fw(:).';
S.park_record = [];
parkFile = fullfile(fileparts(dcf), 'lab', 'arm_park_record.mat');   % calibrateROMEArm's record (char)
if isempty(which('arm_park_record')), addpath(fullfile(fileparts(dcf), 'lab'), '-end'); end   % lab/ is not on the student path
try, S.park_record = arm_park_record('read', parkFile); catch, end   % unreadable / foreign file: no record (refused)
[ok, reasons, info] = prerun_check_core(S);
info.park_file = parkFile;
af = fld(V, 'arm_freeze_user', NaN);               % 1 frozen arm, 0 arm moves (flag)
if ~info.skipped && isnumeric(af) && isscalar(af) && af == 0
    S1 = S;  S1.values.arm_freeze_user = 1;        % the park item of the core, for an arm that moves
    [~, r1] = prerun_check_core(S1);
    r1 = r1(startsWith(r1, 'park:'));
    r1 = strcat(r1, {' (park checked in every robot mode: the arm starts at this posture in modes 2, 3 and 4)'});
    reasons = [reasons, r1];
    ok = ok && isempty(r1);
    info.items{end+1} = 'park (every robot mode)';
end
if given || info.skipped, return; end
rm = bv('run_mode', NaN);                          % run mode (double)
[~, C] = allowed_ranges();
fprintf(['F6 limits (UNSOURCED, proposed values, allowed_ranges.m): camera wait %g s, camera dropout %g s, ' ...
         'arm-ready wait %g s, arm telemetry loss %g s\n'], C.cam_wait_s, C.cam_dropout_max_s, C.arm_wait_s, C.arm_lost_max_s);
if ok
    fprintf('PRERUN CHECK mode %s: PASS (%d items). Not enforced: %s\n', num2str(rm), numel(info.items), strjoin(info.unchecked, ', '));
    return
end
fprintf('PRERUN CHECK mode %s: REFUSED, %d reason(s):\n', num2str(rm), numel(reasons));
fprintf('  - %s\n', reasons{:});
err = MException('prerun_check:refused', 'Pre-run check failed (%d reason(s)): %s. Run refused.', numel(reasons), strjoin(reasons, ' | '));
run_refused(err);                                  % the RUN VERDICT FAIL line of a refused robot run (F14a)
throw(err);
end

function v = bv(n, d)
% Base-workspace variable n, or d when it does not exist.
try
    if evalin('base', sprintf('exist(''%s'', ''var'')', n)) == 1, v = evalin('base', n); else, v = d; end
catch
    v = d;
end
end

function v = fld(S, f, d)
% Field f of struct S, or d.
if isstruct(S) && isfield(S, f), v = S.(f); else, v = d; end
end
