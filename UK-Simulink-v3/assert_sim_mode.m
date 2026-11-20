function assert_sim_mode(who, stage, dcFile)
%ASSERT_SIM_MODE  Refuse to simulate ROME_9DOF unless the run mode is 1, simulation only (focus plan F12, fix round 1).
%   Called by every runner that calls sim() on the real model and its own copy of the robot command block: the gate
%   (check_9dof, gate_all), the case runner (run_9dof_cases), plot_9dof and check_guard.
%   assert_sim_mode(who, 'file', dcFile)  before anything runs: the run_mode line of dcFile (char path) must be 1.
%   assert_sim_mode(who, 'base')          after define_constants and again after load_system (whose PostLoadFcn re-runs
%                                         define_constants and overwrites any assignin): the base workspace must hold
%                                         run_mode 1 and exactly mode 1's flags (run_mode_table(1)).
%   who  name of the caller, printed in the refusal (char).
%   Error assert_sim_mode:notsim otherwise. The message names the mode found and tells the student how to switch to
%   mode 1 for the run and back to the robot mode afterwards. An assignin of EnableHardware = 0 is NOT a fix: the
%   model's PostLoadFcn re-runs define_constants, which derives the flags from run_mode again.
M1 = run_mode_table(1);                                   % mode 1 flags and banner (struct)
switch stage
    case 'file'
        txt = fileread(dcFile);                           % define_constants text (char)
        tok = regexp(txt, '^run_mode[ \t]*=[ \t]*(\d+)[ \t]*;', 'tokens', 'lineanchors');
        if numel(tok) == 1, k = str2double(tok{1}{1}); else, k = NaN; end   % run mode in the file (double)
        ok = isequal(k, 1);
        found = sprintf('run_mode %s in %s', mat2str(k), dcFile);
    case 'base'
        v = struct();                                     % base values, or the char 'undefined'
        for n = {'run_mode', 'EnableHardware', 'EnableMotive', 'motive_source'}
            try, v.(n{1}) = evalin('base', n{1}); catch, v.(n{1}) = 'undefined'; end
        end
        k = v.run_mode;
        ok = isequal(k, 1) && isequal(v.EnableHardware, M1.EnableHardware) && isequal(v.EnableMotive, M1.EnableMotive) && ...
             isequal(v.motive_source, M1.motive_source);
        found = sprintf('run_mode %s, EnableHardware %s, EnableMotive %s, motive_source %s in the base workspace', ...
                        mat2str(k), mat2str(v.EnableHardware), mat2str(v.EnableMotive), mat2str(v.motive_source));
    otherwise
        error('assert_sim_mode:stage', 'stage must be ''file'' or ''base'', not %s', stage);
end
if ok, return; end
back = '';                                                % how to return to the robot mode afterwards (char)
if isnumeric(k) && isscalar(k) && any(k == 2:4)
    Mk = run_mode_table(k);
    back = sprintf(' Afterwards switch back with rome_mode(%d) (%s).', k, Mk.banner);
end
error('assert_sim_mode:notsim', ['%s simulates run mode 1 only (%s) and never commands the robot, but it found %s. ' ...
      'Set run_mode = 1 (type rome_mode(1) or press Mode 1), then run %s again.%s Run refused.'], ...
      who, M1.banner, found, who, back);
end
