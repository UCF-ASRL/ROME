function prerun_fixture(what, tmpRoot)
%PRERUN_FIXTURE  Test helper (F2/F11 wiring, 20 Nov 2026): make a TEMP COPY of ROME_merged pass prerun_check's lab items.
%   The model's InitFcn refuses every robot run (prerun_check.m) without a Part F sign-off date in lab_settings.m and
%   without a park record written by calibrateROMEArm with the current settings SHA. The fake-robot tests run robot
%   modes on temp copies; this helper supplies those two lab facts IN THE COPY ONLY, the way the TA would:
%     prerun_fixture('signoff', tmpRoot)  the copy's lab_settings.m lines "signoff_partF = '';" and
%                                         "signoff_partC = '';" get the date '2026-11-20' (each count-asserted once).
%                                         Call right after the copy is made, before any define_constants run.
%     prerun_fixture('park', tmpRoot)     appends a park record to <tmpRoot>/lab/arm_park_record.mat: the start posture
%                                         of the base workspace (q0_9dof(4:9) mapped and clamped, as calibrateROMEArm
%                                         parks it) with settings_sha(<tmpRoot>/define_constants.m) NOW. Call after the
%                                         copy's define_constants ran (load_system) and right before sim().
%   Refuses (prerun_fixture:notemp) any tmpRoot that is not under tempdir: the real tree never gets a fake sign-off
%   or a fake park record. Nothing in the shipped code reads anything this helper sets other than the two lab facts.
tr = char(java.io.File(tmpRoot).getCanonicalPath());   % (char)
td = char(java.io.File(tempdir).getCanonicalPath());
if ~startsWith(lower(tr), lower(td))
    error('prerun_fixture:notemp', 'prerun_fixture writes into temp copies only; %s is not under %s', tr, td);
end
switch what
    case 'signoff'
        f = fullfile(tr, 'lab_settings.m');
        fid = fopen(f, 'r');  t = char(fread(fid, Inf, '*uint8').');  fclose(fid);
        for nm = {'signoff_partF', 'signoff_partC'}
            old = [nm{1} ' = '''';'];
            n = numel(strfind(t, old));
            assert(n == 1, 'prerun_fixture:count', '%s: "%s" found %d times (expected 1)', f, old, n);
            t = strrep(t, old, [nm{1} ' = ''2026-11-20'';']);
        end
        fid = fopen(f, 'w');  fwrite(fid, uint8(t));  fclose(fid);
    case 'park'
        need = {'q0_9dof', 'arm_sign', 'arm_offset_deg', 'arm_fw_switch', 'arm_fw_other', 'arm_fw_margin_deg'};
        if ~all(cellfun(@(n) evalin('base', sprintf('exist(''%s'', ''var'')', n)) == 1, need))
            fprintf('prerun_fixture: no park record written (define_constants did not run to its end in the copy)\n');
            return                                     % that run is refused anyway; the harness checks the refusal
        end
        q0 = evalin('base', 'q0_9dof');                % 9x1 start configuration (m, m, rad, joints rad)
        fw = evalin('base', 'arm_sign(:).''') .* rad2deg(q0(4:9)).' + evalin('base', 'arm_offset_deg(:).''');   % firmware deg
        fw = arm_clamp(fw, evalin('base', 'arm_fw_switch'), evalin('base', 'arm_fw_other'), evalin('base', 'arm_fw_margin_deg'));
        if ~isfolder(fullfile(tr, 'lab')), mkdir(fullfile(tr, 'lab')); end
        if isempty(which('arm_park_record')), addpath(fullfile(tr, 'lab'), '-end'); end   % the copy's lab/ (not on the path)
        arm_park_record('write', fw, fullfile(tr, 'lab', 'arm_park_record.mat'), settings_sha(fullfile(tr, 'define_constants.m')));
    otherwise
        error('prerun_fixture:what', 'what must be signoff or park, not %s', what);
end
end
