function run_refused(err)
%RUN_REFUSED  Print the PASS/FAIL line of a robot run that was refused at its start (focus plan F14a). Never errors.
%   Called from the catch of arm_freeze_banner.m and apply_pacing.m (their InitFcn / StartFcn form), which then
%   rethrow the refusal unchanged. An InitFcn refusal stops the run before Simulink calls any StopFcn (measured
%   19 Nov 2026), so without this line a refused robot run would end with no verdict.
%   Prints only for a robot run: EnableHardware ~= 0 in the base workspace, or EnableHardware missing / not a numeric
%   scalar (fail-safe: then nobody can say it was a simulation). Stores the verdict as rome_last_verdict.
%   err  the MException of the refusal.
try
    eh = NaN;                                      % 0 simulate only, 1 command the real robot (flag)
    try, eh = evalin('base', 'EnableHardware'); catch, end
    if isnumeric(eh) && isscalar(eh) && eh == 0, return; end
    rm = NaN;  try, rm = evalin('base', 'run_mode'); catch, end   % run mode (double)
    F = struct('mode', rm, 'refused', true, 'refusal_id', err.identifier, 'err_msg', err.message, ...
               'saved_file', '', 'save_err', 'refused before the start: nothing ran, nothing to save');
    V = run_verdict_core(F);
    fprintf('%s\n', V.text);
    assignin('base', 'rome_last_verdict', V);
catch e
    fprintf('run_refused: %s\n', e.message);
end
end
