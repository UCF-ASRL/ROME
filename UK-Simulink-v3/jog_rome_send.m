function out = jog_rome_send(c, target, synced, lim)
%JOG_ROME_SEND  The ONE place jogROMEArm.m sends an arm command (hw audit row 24 / mismatch 2, fixes5, 20 Nov 2026).
%   lim  = jog_rome_send('limits')                 the firmware ranges the other senders cap to, read from
%                                                   define_constants (arm_fw_switch, arm_fw_other, arm_fw_margin_deg,
%                                                   set in lab_settings.m). Read HERE because jogROMEArm has nested
%                                                   functions, so its own workspace cannot take define_constants'
%                                                   variables. lim: struct fw_switch, fw_other (1x6 deg), margin (deg).
%   sent = jog_rome_send(c, target, synced, lim)   cap and send one command line to the link c.
%     REFUSES, error jog_rome_send:notelemetry and NOTHING written, unless synced is true: the jogger must have read
%       the arm's first ARM telemetry line (its target starts from the arm's own reading). Before this fix it sent
%       zeros, and J2 = 0 is a firmware range end (otherLimits[1] = 0.0, ROME_Teensy_Code.ino:30).
%     REFUSES, error jog_rome_send:target and nothing written, a target that is not 6 finite numbers (a NaN would be
%       turned into a range end by min/max).
%     CAPS the target with arm_clamp and the same three constants as ROMECommand.m:114, jog_joint.m:25 and
%       calibrateROMEArm.m:22, so J4 (no ValidateTraj while tracking, ROME_Teensy_Code.ino:292-297) cannot be
%       driven past its range.
%     writes sprintf('ROME,0,0,0,0,%.3f,...,%.3f\n', sent) with write(c, msg, "char"), the line and call jogROMEArm
%       used before (zero wheel speeds).
%     sent  1x6 joint command actually sent (firmware deg), to be shown as the target.
%   Firmware lines: ucf-asrl/ROME 72dbcab, UK-Arduino/ROME_Teensy_Code/ROME_Teensy_Code.ino (read 20 Nov 2026).
%   Tested by tests/test_jog_send.m against the recording fake link (tests/stubs_hw/RobotLinkRecorder.m).
if nargin == 1 && (ischar(c) || isstring(c)) && strcmp(c, 'limits')
    here = fileparts(mfilename('fullpath'));
    addpath(here, fullfile(here, 'Optitrack'));    % as jog_joint.m:14
    evalc('define_constants');
    out = struct('fw_switch', arm_fw_switch, 'fw_other', arm_fw_other, 'margin', arm_fw_margin_deg);
    return
end
if ~(islogical(synced) || isnumeric(synced)) || ~isscalar(synced) || ~synced
    error('jog_rome_send:notelemetry', ['No ARM telemetry received yet: nothing is sent to the arm. The jogger starts ' ...
          'from the arm''s own reading. Is the arm calibrated (calibrateROMEArm) and the ESP32 on? Otherwise call the TA.']);
end
target = double(target(:).');
if numel(target) ~= 6 || any(~isfinite(target))
    error('jog_rome_send:target', 'the joint target must be 6 finite firmware degrees, got %s: nothing sent', mat2str(target));
end
out = arm_clamp(target, lim.fw_switch, lim.fw_other, lim.margin);
msg = sprintf('ROME,0,0,0,0,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', out);
write(c, msg, "char");
end
