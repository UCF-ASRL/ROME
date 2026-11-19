function apply_pacing(mdl)
%APPLY_PACING  Real-time pacing of ROME_9DOF follows EnableHardware, at every simulation start.
%   Called from the model's InitFcn AND StartFcn (apply_pacing(bdroot)) and from define_constants.m.
%   Rule:
%     EnableHardware ~= 0 (base workspace, read NOW) : pacing ON at rate 1, confirmed by reading
%                                                      it back; if it cannot be confirmed the
%                                                      run is refused with error(). Also refused
%                                                      when Fast Restart is on (see below), with
%                                                      no model in context, and with sensor_model ~= 0 (end).
%     EnableHardware == 0                            : pacing OFF (best effort; a paced
%                                                      simulation is slow, not dangerous); with
%                                                      no model in context, a silent return
%                                                      (define_constants run standalone)
%   Why: a student can open the model, set EnableHardware = 1 in the workspace WITHOUT re-running
%   define_constants, and press Run. Pacing set only at load time would then send the robot an
%   unpaced, faster-than-real-time command stream (Task 4 review, minor 4). InitFcn runs at every
%   run start, and a parameter set there applies to that same run (measured, Task 4b:
%   docs/verification/2026-11-19_task4b_callback_order_experiment.log).
%   FAST RESTART (measured, Task 4b fix round 1, docs/verification/2026-11-19_task4b_fix1_fastrestart_experiment.log):
%   with Fast Restart on, InitFcn runs only on the FIRST run of a session and StartFcn runs on every
%   run; the second run, with EnableHardware changed to 1, ran 12 s in 0.29 s wall with pacing off.
%   EnablePacing is not changed between fast restarts, so a hardware run is refused whenever Fast
%   Restart is on; the model calls this function from StartFcn as well, so the refusal happens at
%   every start.
%   mdl  model name (char); default bdroot.
if nargin < 1, mdl = bdroot; end
try
    hw = evalin('base', 'EnableHardware');        % 0 simulate only, 1 command the real robot (flag)
catch
    error('apply_pacing:noflag', 'EnableHardware is not defined in the base workspace; run define_constants first. Run refused.');
end
if ~(isnumeric(hw) && isscalar(hw) && isfinite(hw))
    error('apply_pacing:badflag', 'EnableHardware must be a finite numeric scalar; run refused.');
end
live = hw ~= 0;                                   % true when the real robot is commanded
if isempty(mdl)
    if live
        error('apply_pacing:nomodel', 'EnableHardware = 1 but no model is in context, so pacing cannot be applied. Run refused.');
    end
    return                                        % simulation only, define_constants run standalone
end
if live && bdIsLoaded(mdl) && strcmp(get_param(mdl, 'FastRestart'), 'on')
    error('apply_pacing:fastrestart', ...
          'EnableHardware = 1 but Fast Restart is on for %s: pacing is not re-applied between fast restarts. Turn Fast Restart off. Run refused.', mdl);
end
state = {'off', 'on'};                            % EnablePacing value for live = false, true
want  = state{1 + live};                          % wanted EnablePacing (char)
try
    set_param(mdl, 'EnablePacing', want);
    if live, set_param(mdl, 'PacingRate', 1); end % 1 = simulation time runs at wall-clock time (x)
    got  = get_param(mdl, 'EnablePacing');        % EnablePacing as read back (char)
    rate = get_param(mdl, 'PacingRate');          % PacingRate as read back (x)
catch err
    if live
        error('apply_pacing:cannot_confirm', ...
              'EnableHardware = 1 but real-time pacing could not be set on %s (%s). Run refused.', mdl, err.message);
    end
    return                                        % simulation only: a paced run is just slow
end
if live && ~(strcmp(got, 'on') && isequal(double(rate), 1))
    error('apply_pacing:cannot_confirm', ...
          'EnableHardware = 1 but EnablePacing reads ''%s'' and PacingRate %s on %s. Run refused.', got, num2str(rate), mdl);
end
% SENSOR MODEL ON HARDWARE (final review I5(4), 19 Nov 2026). With sensor_model ~= 0 the SensorModel block
% adds synthetic camera noise (mocap_sigma) and joint quantisation (enc_step_rad) to the state the controller
% reads; that noise is simulation-only. define_constants forces sensor_model = 0 when EnableHardware ~= 0;
% this check covers a student who sets EnableHardware = 1 in the workspace without re-running
% define_constants. An undefined or non-numeric-scalar sensor_model is refused too: the noise cannot then be
% shown to be off (the SensorModel block takes sensor_model as a mask parameter, so the run would not
% compile without it anyway). Placed after the pacing read-back so the earlier refusals keep their ids.
if live
    try
        sm = evalin('base', 'sensor_model');      % 0 exact state to the controller, else synthetic noise (flag)
    catch                                         % undefined in the base workspace
        sm = [];                                  % empty: not a numeric scalar, refused below
    end
    smok = isnumeric(sm) && isscalar(sm) && sm == 0;   % true only when the noise is confirmed off (logical)
    if ~smok
        if isnumeric(sm) && isscalar(sm), smtxt = num2str(sm); else, smtxt = 'undefined or not a numeric scalar'; end   % sensor_model as printed (char)
        error('apply_pacing:sensormodel', ...
              ['EnableHardware = 1 but sensor_model is %s: the SensorModel block would add synthetic camera and encoder ' ...
               'noise to the state the controller reads, which is simulation-only. Set EnableHardware = 1 in define_constants.m ' ...
               'and re-run it (it then forces sensor_model = 0), or set sensor_model = 0 in the workspace. Run refused.'], smtxt);
    end
end
end
