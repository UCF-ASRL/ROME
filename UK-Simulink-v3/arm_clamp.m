function [deg_out, hit] = arm_clamp(deg_in, fw_lo, fw_hi, margin)
%ARM_CLAMP  Cap a firmware joint command to what the Teensy accepts.
%
%   Applied in ROMECommand after the model-to-firmware map, so it works in
%   firmware degrees whatever arm_sign / arm_offset_deg turn out to be. A
%   value outside the range is pulled to the nearer end instead of being
%   sent: the firmware would reject it (ValidateTraj returns 2) and leave the
%   arm where it was, which is worse, because the model then believes the
%   arm moved. The ranges come from limits[] / otherLimits[] in
%   ROME_Teensy_Code.ino and sit in the user block of define_constants.
%
%   margin pulls both ends in, so the arm never reaches the switch even if
%   the calibration is off by that much. It is applied on both sides of
%   every joint, negative ends included: a range of -90..90 with a 2 deg
%   margin becomes -88..88.
%
%   deg_in   1x6  joint command, firmware degrees
%   fw_lo    1x6  lower end of each firmware range (deg)
%   fw_hi    1x6  upper end of each firmware range (deg)
%   margin   1x1  safety margin inside both ends (deg), 0 if omitted
%   deg_out  1x6  command inside the ranges (deg)
%   hit      1x6  logical, true where the command was capped
if nargin < 4, margin = 0; end
deg_in  = deg_in(:).';
deg_out = min(max(deg_in, fw_lo(:).' + margin), fw_hi(:).' - margin);
hit     = deg_out ~= deg_in;
end
