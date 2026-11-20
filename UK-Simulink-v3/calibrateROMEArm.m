function calibrateROMEArm()

% =========================================================================
% ROME Arm Calibration Utility (Production Version)
% =========================================================================

%% Configuration
ESP32_IP = "192.168.4.1";
PORT     = 3333;

TIMEOUT_SECONDS = 120;

% The arm is parked where the model starts, in FIRMWARE degrees, from the
% same define_constants the model runs on: q0_9dof(4:9) is the start
% posture the card prints, arm_sign / arm_offset_deg the model-to-firmware
% map, arm_fw_* the ranges the command is capped to. Nothing is typed
% here.6
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, 'Optitrack'));
evalc('define_constants');  addpath(fullfile(here, 'lab'));  arm_park_record('invalidate', fullfile(here, 'lab', 'arm_park_record.mat'));   % wire3 fix I5: any older park record is void from here (before CAL_ARM); only a successful calibration below writes a valid one
homePosDeg = arm_sign(:) .* rad2deg(q0_9dof(4:9)) + arm_offset_deg(:);
[homePosDeg, hit] = arm_clamp(homePosDeg, arm_fw_switch, arm_fw_other, arm_fw_margin_deg);
homePosDeg = homePosDeg(:);
if any(hit)
    error(['the start posture maps outside the firmware range on joint(s) %s: ' ...
           'fw %s. Fix arm_home (define_constants.m) or arm_sign / arm_offset_deg (lab_settings.m, TA) ' ...
           'and re-run gate_all before calibrating.'], mat2str(find(hit)), ...
           mat2str(round(homePosDeg.', 1)));
end
fprintf('\nScenario %d start posture, firmware degrees: %s\n', scenario, ...
        mat2str(round(homePosDeg.', 1)));
fprintf('Model degrees: %s\n', mat2str(round(rad2deg(q0_9dof(4:9)).', 1)));

fprintf('\n===========================================\n');
fprintf('ROME ARM CALIBRATION\n');
fprintf('===========================================\n');

%% Connect
fprintf('Connecting to %s:%d ...\n', ESP32_IP, PORT);

try
    c = tcpclient(ESP32_IP, PORT, "ConnectTimeout", 10);
catch ME
    error('Failed to establish TCP connection: %s', ME.message);
end

% Guarantee socket cleanup if script fails or is interrupted
cleanupObj = onCleanup(@() cleanupSocket(c));

pause(0.5);
flush(c); % Clear stale startup frames
fprintf('Connected.\n');

%% 1. HOME first, so the calibration itself ends at the start posture
% Keanu's procedure (22 Sep 2026): the Teensy's fullCalibrate drives every
% joint to its limit switch and then to homePosDeg, so sending HOME before
% CAL_ARM makes calibration finish AT the scenario's start posture, one
% motion. The values are already capped to the firmware ranges above; the
% calibration path itself does not validate them.
homeCmd = sprintf('HOME,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', homePosDeg);
parkCmd = sprintf('ROME,0,0,0,0,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', homePosDeg);
fprintf('Sending HOME target configuration...\n');
fprintf('[TX] %s', homeCmd);
flush(c);
write(c, homeCmd, "char");
pause(0.2);

%% 2. Request Calibration
% Every ARM, line from here to the end is logged with a timestamp, so the
% path each joint's reading took (switch value -> home -> park) can be read
% back afterwards. calib_log_<time>.csv next to this file; summary at the end.
armLog = zeros(0, 7);                       % [t, J1..J6]
logT0  = tic;
logfile = fullfile(here, sprintf('calib_log_%s.csv', datestr(now, 'yyyymmdd_HHMMSS')));
fprintf('Sending CAL_ARM...\n');
write(c, "CAL_ARM" + newline, "char");

tic;
armReady = false;

while toc < TIMEOUT_SECONDS
    if c.NumBytesAvailable > 0
        % Read available stream data
        rawMsg = strtrim(readline(c));
        
        if startsWith(rawMsg, "ARM,")
            parts = split(rawMsg, ",");
            if numel(parts) >= 8, armLog(end+1, :) = [toc(logT0), str2double(parts(2:7)).']; end %#ok<AGROW>
            
        elseif startsWith(rawMsg, "LIMIT,") % --- NEW DEBUG CODE ---
            parts = split(rawMsg, ",");
            if numel(parts) >= 2
                fprintf('  --> [HARDWARE] Joint %s hit its limit switch!\n', parts(2));
            end
            
        % Check for hardware handshake complete string
        elseif contains(rawMsg, "ARM_READY")
            armReady = true;
            fprintf('[RX] %s\n', rawMsg);
            break;
        end
    end
    pause(0.02);
end

if isempty(armLog)
    fprintf('  (no ARM telemetry during calibration: the Teensy is silent while it calibrates)\n');
else
    fprintf('  %d ARM lines during calibration; last: %s\n', size(armLog,1), mat2str(round(armLog(end,2:7),1)));
end

if ~armReady
    error('Calibration timeout (%d s). ARM_READY acknowledgment not received.', TIMEOUT_SECONDS);
end

fprintf('Calibration routine finished on hardware.\n');

%% 3. Park command: the same target, the way the model sends it
% Keeps the 500 ms firmware watchdog from holding the arm short of the
% posture while we watch, and confirms the ESP32 split (GV / ARM lines).
fprintf('Parking the arm at the start posture...\n');
fprintf('[TX] %s', parkCmd);
write(c, parkCmd, "char");

%% 4. Watch the telemetry settle, then judge each joint
% The ARM, line comes at 20 Hz. Read it until the angles stop changing (or
% 25 s). CAVEAT: at the end of fullCalibrate the firmware FORCES the
% reported angles to homePosDeg and only refreshes them from the encoders
% once a joint is given a target it is not already at. So a reading equal
% to the target proves nothing by itself; a reading that differs does.
% Confirm the posture by eye, and use jog_joint(j, delta) for the
% direction check on any joint in doubt.
fprintf('Watching the arm settle (up to 25 s)...\n');
lastRead = NaN(1,6);  lastChange = tic;  settleTimer = tic;  statusReceived = false;
lastSend = tic;
while toc(settleTimer) < 25
    if toc(lastSend) > 0.2
        write(c, parkCmd, "char");  lastSend = tic;
    end
    if c.NumBytesAvailable > 0
        msg = strtrim(readline(c));
        if startsWith(msg, "ARM,")
            parts = split(msg, ",");
            if numel(parts) >= 8
                vals = str2double(parts(2:7)).';
                armLog(end+1, :) = [toc(logT0), vals]; %#ok<AGROW>
                statusReceived = statusReceived || contains(parts(8), "READY");
                if any(abs(vals - lastRead) > 0.3) || any(isnan(lastRead))
                    lastChange = tic;
                end
                lastRead = vals;
            end
        end
    end
    if ~any(isnan(lastRead)) && toc(lastChange) > 1.5, break; end
    pause(0.02);
end

if ~isempty(armLog)
    fid = fopen(logfile, 'w');
    fprintf(fid, 't_s,J1,J2,J3,J4,J5,J6\n');
    fprintf(fid, '%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n', armLog.');
    fclose(fid);
    fprintf('\n  telemetry log: %s  (%d lines, %.1f s)\n', logfile, size(armLog,1), armLog(end,1)-armLog(1,1));
    fprintf('  joint   first     min     max    last   target\n');
    for i = 1:6
        col = armLog(:, i+1);
        fprintf('  J%d    %6.1f  %6.1f  %6.1f  %6.1f   %6.1f\n', i, col(1), min(col), max(col), col(end), homePosDeg(i));
    end
    fprintf(['  A joint whose min..max never left its switch value did not count while\n' ...
             '  it moved (encoder). One that swept far past the target ran away.\n']);
end
if any(isnan(lastRead))
    warning('no ARM telemetry received while parking; cannot judge the posture');
else
    fprintf('\n  joint   target   reading   verdict\n');
    bad = false(1,6);
    for i = 1:6
        d1 = abs(lastRead(i) - homePosDeg(i));
        if i == 4
            v = 'open loop, reading is the command (no encoder): check by eye';
        elseif d1 < 2.0
            v = 'at target (or the forced home value: check by eye)';
        else
            v = 'NOT at target  <-- this joint did not get there'; bad(i) = true;
        end
        fprintf('  J%d    %7.1f  %8.1f   %s\n', i, homePosDeg(i), lastRead(i), v);
    end
    fprintf(['\n  Expected posture: elbow up and forward, forearm pointing down, tool at\n' ...
             '  the floor about 0.34 m in front of the base axis.\n']);
    if any(bad)
        fprintf(['  DO NOT run a robot mode (run_mode 2, 3 or 4). Joint(s) %s did not reach the target.\n' ...
                 '  Stop here and call the TA (moving a joint is a TA action).\n'], ...
                mat2str(find(bad)));
    else
        fprintf('  If the arm does not look like that, stop and call the TA.\n');
    end
end
if statusReceived
    fprintf('Arm status: READY.\n');
else
    warning('READY status not confirmed by Teensy.');
end

% The final line comes after the park record below: success only when the record was written (clarity audit A2).

% PARK RECORD (focus plan F2/F11). prerun_check.m refuses every robot run (run_mode 2, 3, 4) unless the newest record
% holds this start posture AND the settings SHA of define_constants.m + lab_settings.m as they are at the Run
% (settings_sha.m, the hash define_constants stores as dc_sha_at_run). So any edit of either file after this point,
% a Mode button included, needs a new calibration (author: "We need to recalibrate no matter what").
% Written only when every encoder joint read at target; otherwise no record and robot runs stay refused.
addpath(fullfile(here, 'lab'));  parkWritten = false;           % arm_park_record; parkWritten: true only once the record is written below
parkFile = fullfile(here, 'lab', 'arm_park_record.mat');        % parked-posture history (char)
if ~any(isnan(lastRead)) && ~any(bad)
    rec = arm_park_record('write', homePosDeg.', parkFile, settings_sha(fullfile(here, 'define_constants.m')));  parkWritten = true;
    fprintf('Park record written (entry %d): %s\n', rec.n, parkFile);
else
    fprintf('NO park record written (no telemetry, or a joint not at target): the old record was invalidated at the start, so robot runs stay refused.\n');
end
if parkWritten   % clarity audit A2: the success line only when the park record was written
    fprintf('\nROME arm calibration successfully completed.\n\n');
else
    fprintf(['\nROME arm calibration FAILED: no park record was written, so robot runs (run_mode 2, 3, 4) stay refused.\n' ...
             'Do not run a robot mode. Call the TA.\n\n']);
end

end

% Helper for safe TCP socket termination
function cleanupSocket(c)
    if isvalid(c)
        flush(c);
        clear c;
        fprintf('TCP Socket disconnected.\n');
    end
end
