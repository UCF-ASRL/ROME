function q_arcs_deg = romefw_to_arcs_deg(q_romefw_deg, map)
%ROMEFW_TO_ARCS_DEG  Lab ROME Teensy firmware joint angles -> ARCS 1.0 joint angles (input of ar3_fk).
%   q_arcs_deg = romefw_to_arcs_deg(q_romefw_deg)        default map 'rome_dirpin'
%   q_arcs_deg = romefw_to_arcs_deg(q_romefw_deg, map)   map = 'rome_dirpin' | 'arcs_caldir'
%     q_romefw_deg  6x1 joint angles as ROME_Teensy_Code.ino numbers them (deg)
%     q_arcs_deg    6x1 the same physical posture as ARCS numbers it (deg)
%   q_arcs = s .* q_romefw + c, every s(j) = +1 or -1. Analysis only; not used by the controller.
%   NOT MEASURED. Derived from the two firmwares' source; the jog test (Plan 3) decides.
%
%   Both maps put the ROME switch-side limit (limits[], ROME_Teensy_Code.ino L29, where fullCalibrate
%   sets the position at the switch, L394) on an end of the ARCS range. s(j) is the sign that makes
%   travel away from the switch in ROME numbering (towards otherLimits[], L30) travel away from that
%   end in ARCS numbering. c(j) = anchor(j) - s(j)*limits[j].
%
%   The two firmwares disagree on the homing side of J2, J5 and J6 (R5.1). The physical evidence
%   compared is the DIR-pin level each drives while homing, which assumes the lab wired the drivers as
%   the ARCS default:
%     ARCS   CalDir '001001', MotDir '000000' (ARbot.cal tuple 77-78); caldrive = (caldir==motdir)?0:1
%            (ARCS.py calRobot L4857-4880); rotdir J1..J6 = 1 1 1 0 1 1 (ARCS_teensy_sketch.ino L50-55);
%            DIR = LOW iff rotdir == caldrive (sketch LL block L363-444) -> homing DIR
%            J1..J6 = HIGH HIGH LOW LOW HIGH LOW; the switch angle is NegAngLim if caldir = 0, else
%            PosAngLim (ARCS.py L4893-4963) -> J1..J6 = -170 -129.6 143.7 -164.5 -104.15 148.1
%     ROME   homing speed = -calspeed.*negspeeds (ROME_Teensy_Code.ino L380-382; calspeed L18,
%            negspeeds L33 = -1 1 1 1 1 -1) = +1200 -800 -800 -600 -800 +800; AccelStepper DRIVER
%            (L84-91, DIR pins 1 3 5 7 9 11, L39; no setPinsInverted): speed > 0 -> DIR HIGH
%            (AccelStepper 1.24 setSpeed / step1, read from a public mirror, not the lab's installed
%            copy: NOT VERIFIED for the lab; docs/verification/2026-11-19_task5_accelstepper_source.log)
%            -> homing DIR J1..J6 = HIGH LOW LOW LOW LOW HIGH
%     Agree on J1, J3, J4; disagree on J2, J5, J6.
%   'rome_dirpin' (default, R5.1): the lab firmware's homing direction is taken at face value with the
%       vendor wiring, so on J2, J5, J6 the lab switch sits at the ARCS NON-calibration end.
%       Consistency check (same premises, so not independent evidence): with the same assumption, the DIR level that raises the angle is the same in both
%       firmwares on J1-J5 and opposite on J6, i.e. s = [1 1 1 1 1 -1] (dirpin_table log).
%   'arcs_caldir': the lab switch is the vendor's switch on every joint (the ARCS calibration end);
%       the DIR difference is then a wiring difference that negspeeds compensates.
%
%   Per-joint values (deg). ROME limits[] / otherLimits[] from ROME_Teensy_Code.ino L29-30; ARCS ends
%   from ARbot.cal tuple 35-51 (ARCS.py L7865-7881):
%     J  ROME switch..other | ARCS cal end..other | rome_dirpin: anchor  s   c     | arcs_caldir: anchor  s   c
%     1  -180 .. 160        | -170   ..  170      |             -170   +1   10     |              -170   +1   10
%     2   132 .. 0          | -129.6 ..  0        |                0   +1 -132     |            -129.6   -1    2.4
%     3   141 .. 1          |  143.7 ..  1        |              143.7 +1    2.7   |             143.7   +1    2.7
%     4  -165 .. 165        | -164.5 ..  164.5    |             -164.5 +1    0.5   |            -164.5   +1    0.5
%     5    90 .. -90        | -104.15 .. 104.15   |              104.15 +1  14.15  |           -104.15   -1  -14.15
%     6   180 .. -170       |  148.1 .. -148.1    |             -148.1 -1   31.9   |             148.1   +1  -31.9
%   The far ends do not coincide: the range widths differ (J2 132 vs 129.6, J3 140 vs 142.7,
%   J4 330 vs 329, J5 180 vs 208.3, J6 350 vs 296.2 deg). The firmware step tables also differ (ROME
%   stepsDeg L26 against ARCS StepLim/range: J4 1.092x, J5 0.457x, J6 0.937x). Whether that is a real
%   steps-per-degree error on the lab arm depends on the driver microstep settings: NOT VERIFIED. It
%   is not encoded here, because a scale is not geometry (R5.2).
if nargin < 2 || isempty(map)
    map = 'rome_dirpin';                    % default: the lab firmware's homing direction (R5.1)
end
switch map
    case 'rome_dirpin'
        s = [ 1    1    1    1    1    -1  ];   % 1x6 sign (-), table above
        c = [10 -132    2.7  0.5  14.15  31.9];  % 1x6 offset (deg), table above
    case 'arcs_caldir'
        s = [ 1   -1    1    1   -1     1  ];   % 1x6 sign (-), table above
        c = [10    2.4  2.7  0.5 -14.15 -31.9];  % 1x6 offset (deg), table above
    otherwise
        error('romefw_to_arcs_deg:map', 'unknown map ''%s'' (use ''rome_dirpin'' or ''arcs_caldir'')', map);
end
q = q_romefw_deg(:);                        % 6x1 ROME firmware joint angles (deg)
if numel(q) ~= 6
    error('romefw_to_arcs_deg:input', 'q_romefw_deg must have 6 elements, got %d', numel(q));
end
q_arcs_deg = s(:) .* q + c(:);
end
