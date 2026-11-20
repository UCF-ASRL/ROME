function out = run_mode_label(mdl, txt)
%RUN_MODE_LABEL  The run-mode label on the canvas of ROME_9DOF (annotation Tag rome_run_mode_label; focus plan F12).
%   run_mode_label(mdl, txt)  writes txt into the label of model mdl, if mdl is loaded and has exactly one label
%                             (rome_mode does this after each button press); the model's Dirty flag is kept.
%   run_mode_label(mdl)       writes the NEUTRAL text, which names no mode. The model's PreSaveFcn is
%                             'run_mode_label(bdroot)' (tools/wire_run_mode_buttons.m), so a saved model never carries
%                             a mode and a freshly opened one cannot show a stale mode; define_constants prints the
%                             mode at every load and arm_freeze_banner at every Run.
%   t = run_mode_label()      returns the neutral text (char), the one definition the wiring tool and tests use.
NEUTRAL = ['RUN MODE: press a Mode button (or type rome_mode(n)). define_constants prints the mode at every load ' ...
           '(run_mode in its user block; 1 sim only | 2 GV, frozen arm, fake camera | 3 GV + arm, fake camera | 4 GV + arm, cameras).'];   % (char)
if nargin == 0, out = NEUTRAL; return; end
if nargin < 2, txt = NEUTRAL; end
if isempty(mdl) || ~bdIsLoaded(mdl), return; end
h = find_system(mdl, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'annotation', 'Tag', 'rome_run_mode_label');   % label handle(s)
if numel(h) ~= 1, return; end                             % model not wired: nothing to show
d0 = get_param(mdl, 'Dirty');                             % 'on' | 'off' before the change (char)
set_param(h, 'Text', txt);
set_param(mdl, 'Dirty', d0);
end
