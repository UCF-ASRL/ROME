function M = rome_mode(n)
%ROME_MODE  Switch ROME to run mode n (1, 2, 3 or 4) in one call (focus plan F12, author 19 Nov 2026: "SUPER easy to
%   setup"). The four Mode buttons on the top level of ROME_9DOF.slx call rome_mode(1) ... rome_mode(4)
%   (tools/wire_run_mode_buttons.m). The modes (run_mode_table.m, the only place the mapping lives):
%     1 Simulation only (end-effector dynamics sim, pose data from sim)
%     2 ground vehicle moving, frozen arm, camera off (fake camera)
%     3 ground vehicle moving, moving arm, camera off (fake camera)
%     4 ground vehicle moving, moving arm, camera on (OptiTrack)
%   Steps:
%     1. refuses while ROME_9DOF is running (rome_mode:running): the workspace must not change under a run;
%     2. set_run_mode(n, which('define_constants')): validates n (run_mode_table:bad, naming the four modes), refuses
%        while a harness holds define_constants.m (set_run_mode:locked), and rewrites ONLY the run_mode line of the
%        file that the model's PostLoadFcn and the stale-file check (arm_freeze_banner) use;
%     3. re-runs that define_constants in the base workspace, so the workspace matches the file again (without
%        it the next Run is refused: arm_freeze_banner:stale). If define_constants refuses the new mode (e.g. mode 4
%        while motive_cal.measured is false: motive_cal_check:unmeasured), the PREVIOUS mode is written back and
%        define_constants re-run with it, the label says so, and the refusal is passed on (fix round 1);
%     4. if the model is loaded, writes the mode (or "REFUSED") into the canvas label (run_mode_label.m);
%     5. prints the banner line once more, last, so it is the line a student sees.
%   M = rome_mode(n) also returns run_mode_table(n) (struct).
mdl = 'ROME_9DOF';                                        % the model the buttons live on (char)
if bdIsLoaded(mdl) && ~strcmp(get_param(mdl, 'SimulationStatus'), 'stopped')
    error('rome_mode:running', '%s is running (%s); stop it before changing the run mode.', mdl, get_param(mdl, 'SimulationStatus'));
end
dcf = which('define_constants');                          % the file the model and the stale check use (char)
if isempty(dcf)
    error('rome_mode:nodc', 'define_constants.m is not on the MATLAB path; open the project or cd to the model folder (the folder that holds define_constants.m).');
end
[M, oldMode] = set_run_mode(n, dcf);                      % validates n, rewrites only the run_mode line (struct, double)
clear('define_constants');                                % drop any cached copy of the script before re-running it
rehash;
try
    evalin('base', 'define_constants');
catch err
    back = 'the previous mode could not be determined';   % what happened to the previous mode (char)
    if isnumeric(oldMode) && isscalar(oldMode) && any(oldMode == 1:4) && oldMode ~= M.n
        try
            evalc('set_run_mode(oldMode, dcf)');
            clear('define_constants');  rehash;
            evalc('evalin(''base'', ''define_constants'')');
            back = sprintf('MODE %d restored', oldMode);
        catch err2
            back = sprintf('MODE %d NOT restored (%s)', oldMode, err2.identifier);
        end
    end
    run_mode_label(mdl, sprintf('MODE %d REFUSED by define_constants (%s); %s. See the Command Window.', M.n, err.identifier, back));
    fprintf('rome_mode: MODE %d REFUSED (%s); %s.\n', M.n, err.identifier, back);
    rethrow(err);
end
run_mode_label(mdl, sprintf('RUN MODE (set by rome_mode, %s): %s', datestr(now, 'dd-mmm-yyyy HH:MM'), M.banner)); %#ok<TNOW1,DATST>
fprintf('rome_mode: %s\n', M.banner);
end
