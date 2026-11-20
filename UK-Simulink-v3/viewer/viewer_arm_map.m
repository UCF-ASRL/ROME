function [arm_sign, arm_offset_deg] = viewer_arm_map(rmDir)
%VIEWER_ARM_MAP  Read arm_sign and arm_offset_deg from the TEXT of define_constants.m (not run, not edited).
%   Model joint (deg) -> firmware joint (deg): fw = arm_sign .* model_deg + arm_offset_deg (define_constants.m).
%   Parsing the file keeps this viewer in step with the project's map instead of a hand-copied number.
%   rmDir  the ROME_merged folder (default: parent of this viewer folder).
if nargin < 1 || isempty(rmDir), rmDir = fileparts(fileparts(mfilename('fullpath'))); end
txt = fileread(fullfile(rmDir, 'define_constants.m'));      % source text (char)
arm_sign       = grab(txt, 'arm_sign');
arm_offset_deg = grab(txt, 'arm_offset_deg');
end

function v = grab(txt, name)
tok = regexp(txt, ['^\s*' name '\s*=\s*\[([^\]]*)\]'], 'tokens', 'lineanchors', 'once');
assert(~isempty(tok), 'viewer_arm_map:parse', 'no line "%s = [...]" in define_constants.m', name);
v = sscanf(tok{1}, '%f');
assert(numel(v) == 6, 'viewer_arm_map:parse', '%s has %d numbers, need 6', name, numel(v));
v = v(:);
end
