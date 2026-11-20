function varargout = test_heading_plot(brk)
%TEST_HEADING_PLOT  The heading error the mode-1 plots draw is the real one (F14d finding, fixed in F14a; author 19 Nov:
%   "Yeah, you can fix that one line"). Two lines were fixed: plotting.m:115 err_th wrapped to (-pi, pi], and
%   plot_9dof.m:61 the commanded yaw read from the reference quaternion's components 2 and 3, as ee_attitude.m:15 does.
%   Uses the SAVED case-1 run, runs/case_1.mat (no simulation): its t, q, qd, w, tau, manip, resid become the out that
%   plot_9dof(out) takes ("PLOT_9DOF(OUT) reuses a run"), on a TEMPORARY COPY of ROME_merged with define_constants of
%   the copy run first (mode 1, scenario 1: what plot_9dof reads from the base). plotting.m runs in the base workspace,
%   so the plotted data are read back from there (err_th, th_des: the data, not the figures).
%   Checks (all evaluated, then one error test_heading_plot:fail):
%     (i)   max |err_th| <= 2.6 deg (the saved run's own attitude error, max Eatt = 2.5443 deg, computed another way:
%           the angle of Re * R_ref'; docs/verification/2026-11-19_F14d_heading_plot.md);
%     (ii)  the plotted commanded yaw th_des equals the saved reference yaw YawRef (ee_attitude.m) within 1e-9 deg
%           after wrapping, and sweeps more than 300 deg over the lap (it is the reference, not a constant).
%   MAKE-IT-FAIL: test_heading_plot('mf_wrap') puts plotting.m:115 back to the unwrapped difference and
%   test_heading_plot('mf_comp') puts plot_9dof.m:61 back to 2*atan2(u(4), u(1)), each in the copy (one match
%   asserted); each must FAIL. test_heading_plot('list') returns the table.
%   Flat function without local functions (CLAUDE.md section 5, crash 2). Clears the base workspace, closes figures.
if nargin < 1 || isempty(brk), brk = ''; end
here = fileparts(mfilename('fullpath'));  root = fileparts(here);
cr = char([13 10]);
B = {'mf_wrap', 'i',  'plotting.m', ['err_th = rad2deg(atan2(sin(th_des - th_track), cos(th_des - th_track)));   % wrapped to (-pi, pi] (F14d/F14a, author 19 Nov)' cr], ...
                                    ['err_th = rad2deg(th_des - th_track); ' cr];
     'mf_comp', 'ii', 'plot_9dof.m', '2*atan2(u(3), u(2))];', '2*atan2(u(4), u(1))];'};
if strcmp(brk, 'list'), varargout{1} = B; return; end
tmp = tempname;  startdir = pwd;  path0 = path;
mkdir(tmp);  copyfile(root, tmp);
cleanup = onCleanup(@() cellfun(@(c) c(), {@() close('all', 'force'), @() cd(startdir), @() path(path0), ...
          @() evalin('base', 'clear all'), @() rmdir(tmp, 's')})); %#ok<NASGU>
if ~isempty(brk)
    ib = find(strcmp(B(:,1), brk));
    assert(numel(ib) == 1, 'test_heading_plot:brk', 'unknown break %s', brk);
    f = fullfile(tmp, B{ib,3});
    fid = fopen(f, 'r');  s = char(fread(fid, Inf, '*uint8').');  fclose(fid);
    n = numel(strfind(s, B{ib,4}));
    assert(n == 1, 'test_heading_plot:brk', 'break edit in %s matches %d times (expected 1)', B{ib,3}, n);
    fid = fopen(f, 'w');  fwrite(fid, uint8(strrep(s, B{ib,4}, B{ib,5})));  fclose(fid);
    fprintf('MAKE-IT-FAIL break %s (check (%s) must FAIL): %s reverted in the copy\n', brk, B{ib,2}, B{ib,3});
end
addpath(tmp, fullfile(tmp, 'Ground Vehicle'), fullfile(tmp, 'Optitrack'));
cd(tmp);  rehash;
assert(strcmp(fileparts(which('plot_9dof')), tmp) && strcmp(fileparts(which('plotting')), tmp), 'test_heading_plot:path', ...
       'plot_9dof / plotting resolve to %s / %s', which('plot_9dof'), which('plotting'));
evalin('base', 'clear all');
evalc('evalin(''base'', ''define_constants'')');
assert(isequal(evalin('base', 'run_mode'), 1) && isequal(evalin('base', 'scenario'), 1), 'test_heading_plot:mode', 'the copy is not at mode 1, scenario 1');
C = load(fullfile(tmp, 'runs', 'case_1.mat'));
assert(isequal(C.sc, 1), 'test_heading_plot:case', 'runs/case_1.mat holds scenario %d', C.sc);
out = struct('q_cmd', timeseries(C.q, C.t), 'q_dot_cmd', timeseries(C.qd, C.t), 'wheel_speeds_rpm', timeseries(C.w, C.t), ...
             'tau_arm', timeseries(C.tau, C.t), 'manip', timeseries(C.manip(:), C.t), 'resid', timeseries(C.resid(:), C.t));
set(0, 'DefaultFigureVisible', 'off');
evalc('plot_9dof(out);');
err_th = evalin('base', 'err_th');  th_des = evalin('base', 'th_des');   % (deg) and (rad), as plotting.m holds them
mx = max(abs(err_th));
dRef = rad2deg(th_des(:)) - C.YawRef(:);
dRef = max(abs(mod(dRef + 180, 360) - 180));      % plotted commanded yaw vs the saved reference yaw, wrapped (deg)
sweep = rad2deg(max(unwrap(th_des)) - min(unwrap(th_des)));   % how far the commanded yaw turns over the lap (deg)
okI = mx <= 2.6;
okII = dRef <= 1e-9 && sweep > 300;
fprintf('(i)  max |plotted heading error| %.4f deg (limit 2.6; saved max Eatt %.4f deg) -> %s\n', mx, max(C.Eatt), mat2str(okI));
fprintf('(ii) plotted commanded yaw vs saved YawRef: max %.3g deg (limit 1e-9); it turns %.2f deg over the lap (need > 300) -> %s\n', ...
        dRef, sweep, mat2str(okII));
fails = {};
if ~okI, fails{end+1} = '(i) plotted heading error above 2.6 deg'; end
if ~okII, fails{end+1} = '(ii) plotted commanded yaw is not the reference yaw'; end
if ~isempty(fails)
    error('test_heading_plot:fail', 'test_heading_plot FAILED: %s', strjoin(fails, '; '));
end
fprintf('\ntest_heading_plot PASS: (i) (ii)\n');
end
