function verify_wheelmap()
%VERIFY_WHEELMAP  The 9-DOF block must address the same motors as the robot.
%   The wheel map decides which physical motor each element of the wheel command reaches, and it depends
%   on the alpha ORDER. The expected speeds come from a source independent of the model and of
%   define_constants.m (Task 7c; before, both sides read the workspace alphas, so alphas reversed in
%   define_constants or in the model's chart passed, Task 7b breaks wm_alphas_reversed, wm_chart_order):
%     thesis Table 1 (Brayman, HUT thesis, PDF page 16, text extracted 19 Nov 2026): alpha_1..alpha_4 =
%       45, 135, 225, 315 deg, wheels 1 to 4 in that order;
%     thesis Figure 6 as recorded in CLAUDE.md section 15 and docs/HARDWARE_IMPLEMENTATION.md E8: alpha
%       counter-clockwise from body +x, drive direction [-sin(alpha), cos(alpha)]. Figure 6 was not
%       re-read for Task 7c (its reading is the 16 Sep record cited there).
%   So wheel i turns at (-sin(alpha_i) vx_b + cos(alpha_i) vy_b + l w)/r. r and l (geometry, not order) are
%   taken from define_constants. Three model-side maps are compared with it per wheel and in order:
%     1  the row formula with the workspace alphas (what define_constants hands the model);
%     2  rome_wheels executed from ukd_current.m (the block's source file);
%     3  rome_wheels executed from the UKDynamics CHART inside ROME_9DOF.slx (what the model runs).
d = fileparts(mfilename('fullpath')); cd(d); addpath(d);
wmdir = d;                                           % this folder (char); define_constants overwrites d (base length, m)
evalc('define_constants');
qd = [0.35; -0.20; 0.42];  th = 0.35;                % world base velocity (m/s, m/s, rad/s) and yaw (rad)
Rwb = [cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1];   % world to body rotation
vb  = Rwb*qd;                                        % body velocity [vx_b; vy_b; w] (m/s, m/s, rad/s)
al_thesis = deg2rad([45; 135; 225; 315]);            % wheel angles 1..4, CCW from body +x, thesis Table 1 (rad)
w_ref = arrayfun(@(i) (-sin(al_thesis(i))*vb(1) + cos(al_thesis(i))*vb(2) + l*vb(3))/r, (1:4).')*60/(2*pi);
                                                     % expected wheel speeds 1..4 (rev/min), thesis convention
w_ws  = arrayfun(@(i) (sin(alphas(i))*vb(1) + cos(alphas(i))*vb(2) + l*vb(3))/r, (1:4).')*60/(2*pi);
                                                     % row formula of the blocks with the workspace alphas (rev/min)
w_file  = block_wheels(qd, th, fileread('ukd_current.m'), 'file');   % rome_wheels from ukd_current.m (rev/min)
mdl = 'ROME_9DOF';  wasLoaded = bdIsLoaded(mdl);
if ~wasLoaded, load_system(fullfile(wmdir, [mdl '.slx'])); end
rt = sfroot;  ch = rt.find('-isa', 'Stateflow.EMChart', 'Name', 'UKDynamics');
keep = false(size(ch));  for c = 1:numel(ch), keep(c) = startsWith(ch(c).Path, [mdl '/']); end
ch = ch(keep);
assert(numel(ch) == 1, 'verify_wheelmap:chart', 'found %d UKDynamics charts in %s', numel(ch), mdl);
chartSrc = ch.Script;                                % script of the chart the model runs (char)
if ~wasLoaded, close_system(mdl, 0); end
w_chart = block_wheels(qd, th, chartSrc, 'chart');  % rome_wheels from the UKDynamics chart (rev/min)
fprintf('\n  thesis Table 1 / Fig. 6     : [%8.3f %8.3f %8.3f %8.3f]\n', w_ref);
fprintf('  row formula, workspace alphas: [%8.3f %8.3f %8.3f %8.3f]\n', w_ws);
fprintf('  ukd_current.m rome_wheels  : [%8.3f %8.3f %8.3f %8.3f]\n', w_file);
fprintf('  UKDynamics chart rome_wheels: [%8.3f %8.3f %8.3f %8.3f]\n', w_chart);
names = {'workspace alphas', 'ukd_current.m', 'UKDynamics chart'};   % model-side maps, in print order
W = [w_ws, w_file, w_chart];                         % 4 x 3 model-side speeds (rev/min)
bad = {};
for k = 1:3
    e = abs(W(:,k) - w_ref);                         % per-wheel difference, wheels 1..4 in order (rev/min)
    fprintf('  %-17s per-wheel |diff| [%.2e %.2e %.2e %.2e] rpm\n', names{k}, e);
    if any(e > 1e-9), bad{end+1} = sprintf('%s (wheels %s)', names{k}, mat2str(find(e > 1e-9).')); end %#ok<AGROW>
end
assert(isempty(bad), 'verify_wheelmap:mismatch', ...
   ['The wheel map of %s differs from thesis Table 1 / Fig. 6 (alpha 45/135/225/315 deg CCW from +x, ' ...
    'drive direction [-sin, cos]). Those speeds would reach the wrong motors.'], strjoin(bad, ', '));
fprintf('  PASS - all three model-side maps address the motors of thesis Table 1 / Fig. 6, in order.\n\n');
end

function w = block_wheels(qd, th, src, tag)
% The BLOCK's own wheel map, executed from the block's own source text (ukd_current.m or the chart script).
% Its rome_wheels local function is extracted and written to a temporary file so it can be called directly,
% with the workspace alphas. rome_wheels is called rather than the whole block because the block reports
% speeds for its INTEGRATED rate, already advanced by qdd*dt; that would compare two different velocities.
evalc('define_constants');
src = [strrep(src, char(13), '') newline];   % CRLF-safe, and a final newline so the last function closes
i0  = strfind(src, 'function w = rome_wheels(');
assert(numel(i0) == 1, 'verify_wheelmap:nofn', 'expected one rome_wheels in the %s source, found %d', tag, numel(i0));
tailsrc = src(i0:end);
i1  = strfind(tailsrc, sprintf('\nend\n'));
body = tailsrc(1:i1(1)+4);                    % up to and including its end
tmp  = fullfile(tempdir, ['blk_wheelmap_probe_' tag]);
if ~isfolder(tmp), mkdir(tmp); end
fid = fopen(fullfile(tmp, 'rome_wheels.m'), 'w');  fwrite(fid, body);  fclose(fid);
clear rome_wheels
addpath(tmp);  clean = onCleanup(@() rmpath(tmp));
rehash path
w = rome_wheels([0; 0; th], qd(:), r, l, alphas(:)) * (60/(2*pi));
end
