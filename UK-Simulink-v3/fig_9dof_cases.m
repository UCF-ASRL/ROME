function fig_9dof_cases()
%FIG_9DOF_CASES  Three column-width results figures per case study from the
%   Simulink runs saved by run_9dof_cases (runs/case_<n>.mat, 'post' mode
%   adds the velocity, acceleration and attitude arrays). One figure shows
%   one thing, at the Frontiers one-column width (85 mm), as the Frontiers
%   example papers and the author's ACC 2023 paper do (author, 24 Sep 2026:
%   "the other Frontiers manuscripts have single images ... it's hard to see
%   what we are trying to show"; the 2 x 2 grids of stacked axes are gone).
%
%   fig_9dof_case_<n>_path.pdf      paths on the floor: reference dashed, end
%                                   effector, base center, start o and end []
%   fig_9dof_case_<n>_error.pdf     lead-corrected position error over
%                                   attitude error, sharing the time axis
%   fig_9dof_case_<n>_commands.pdf  joint-angle commands over wheel-speed
%                                   commands, sharing the time axis
%   Joint rates, torques, residual and conditioning are in the table and the
%   text (PAPER_WRITING_RULES.md, section 15). Prints the Table 3 line for
%   each case and writes figures/fig_9dof_cases_values.txt.
here = fileparts(mfilename('fullpath'));
if isfolder(fullfile(here, 'plot_style')), addpath(fullfile(here, 'plot_style')); else, addpath(fullfile(here, '..', '..', '..', 'plot_style')); end   % house style: plot_style/ (copied unchanged, README.md); fallback only the review folder's Research/plot_style
addpath(here);
OUT = fullfile(here, '..', 'figures', 'matlab');   % the manuscript's figures are the PowerPoint charts of code/make_results_pptx.py (24 Sep 2026); this MATLAB twin is the cross-check and the values table
if ~isfolder(OUT), mkdir(OUT); end
Cp = paper_style();
assert(~isfile(fullfile(here,'define_constants.PRISTINE')), 'a run holds define_constants.m');
K = evalc('run(fullfile(here, ''define_constants.m''))'); %#ok<NASGU>  % arm firmware map and max_rpm
lo = min(arm_fw_switch, arm_fw_other); hi = max(arm_fw_switch, arm_fw_other); mg = arm_fw_margin_deg;

fid = fopen(fullfile(OUT, 'fig_9dof_cases_values.txt'), 'w');
hdr = sprintf('%-17s %6s %9s %9s %7s %12s %9s %10s %9s\n', 'case', 'T[s]', 'resid', 'peak rpm', 'min s', 'steady[um]', 'lead[mm]', 'margin[deg]', 'peak tau');
fprintf('%s', hdr); fprintf(fid, '%s', hdr);
jl = {'q_1 base rotation','q_2 shoulder','q_3 elbow','q_4 wrist rotation','q_5 wrist bend','q_6 tool rotation'};   % AR3 joint names (Annin AR3 manual)
for sc = 1:4                         % the circle (case 5) is not in the paper (author, 25 Sep 2026)
    D = load(fullfile(here, 'runs', sprintf('case_%d.mat', sc)));
    t = D.t; q = D.q; w = D.w; E = D.E; S = D.S;
    fw = arm_sign .* rad2deg(q(:,4:9)) + arm_offset_deg;                % N x 6, firmware degrees
    room = min([min(fw,[],1) - (lo + mg), (hi - mg) - max(fw,[],1)]) + mg;   % closest approach to a limit
    line = sprintf('%-17s %6.0f %9.1e %9.1f %7.3f %12.3g %9.3f %10.0f %9.2f\n', D.name, S.T, S.max_res, ...
                   S.peak_rpm, S.min_s, 1e6*S.rmse_ee_lead, 1e3*S.rmse_ee, room, S.peak_tau);
    fprintf('%s', line); fprintf(fid, '%s', line);
    tend = t(end); th = D.T_ON;      % dotted line at the end of the reference onset

    % ---------- figure 1: paths on the floor ----------
    f = paper_figure([3.35 3.45]);
    ax = axes(f); hold(ax, 'on');
    hb = plot(ax, q(:,1), q(:,2), '-', 'Color', Cp(2,:), 'LineWidth', 1.0);
    he = plot(ax, D.P(:,1), D.P(:,2), '-', 'Color', Cp(1,:), 'LineWidth', 1.1);
    plot(ax, D.P(1,1), D.P(1,2), 'o', 'Color', Cp(1,:), 'MarkerFaceColor', 'w', 'MarkerSize', 5);
    plot(ax, D.P(end,1), D.P(end,2), 's', 'Color', Cp(1,:), 'MarkerFaceColor', Cp(1,:), 'MarkerSize', 5);
    plot(ax, q(1,1), q(1,2), 'o', 'Color', Cp(2,:), 'MarkerFaceColor', 'w', 'MarkerSize', 5);
    plot(ax, q(end,1), q(end,2), 's', 'Color', Cp(2,:), 'MarkerFaceColor', Cp(2,:), 'MarkerSize', 5);
    hr = plot(ax, D.Ref(:,1), D.Ref(:,2), '--k', 'LineWidth', 0.9);   % reference last, on top: it coincides with the end-effector line
    axis(ax, 'equal'); xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
    legend(ax, [hr he hb], {'reference', 'end effector', 'base center'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
    paper_export(f, fullfile(OUT, sprintf('fig_9dof_case_%d_path.pdf', sc)));
    close(f);

    % ---------- figure 2: tracking errors ----------
    f = paper_figure([3.35 3.0]);
    T = tiledlayout(f, 2, 1, 'TileSpacing', 'tight', 'Padding', 'compact');
    ax = nexttile(T); hold(ax, 'on');
    % the time-aligned error only (author, 24 Sep 2026 late evening); the one-sample offset is stated in the text
    plot(ax, t, 1e3*E, '-', 'Color', Cp(1,:), 'LineWidth', 1.1);
    ylabel(ax, 'position error [mm]'); finish(ax, tend, th); ylim(ax, [-0.06 1.10]*max(1e3*max(E), 1e-3)); set(ax, 'XTickLabel', []);
    ax = nexttile(T); hold(ax, 'on');
    plot(ax, t, D.EattLead, '-', 'Color', Cp(1,:), 'LineWidth', 1.1);
    ylabel(ax, 'attitude error [deg]'); finish(ax, tend, th); ylim(ax, [-0.06 1.10]*max(max(D.EattLead), 0.01)); xlabel(ax, 'time [s]');
    paper_export(f, fullfile(OUT, sprintf('fig_9dof_case_%d_error.pdf', sc)));
    close(f);

    % ---------- figure 3: commands ----------
    f = paper_figure([3.35 3.9]);
    T = tiledlayout(f, 2, 1, 'TileSpacing', 'tight', 'Padding', 'compact');
    ax = nexttile(T); hold(ax, 'on');
    for i = 1:6, plot(ax, t, rad2deg(q(:,3+i)), '-', 'Color', Cp(i,:), 'LineWidth', 1.0); end
    ylabel(ax, 'joint command [deg]'); finish(ax, tend, th); set(ax, 'XTickLabel', []);
    legend(ax, jl, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'Box', 'off');
    ax = nexttile(T); hold(ax, 'on');
    for i = 1:4, plot(ax, t, w(:,i), '-', 'Color', Cp(i,:), 'LineWidth', 1.0); end
    if max(abs(w(:))) > 0.3*D.max_rpm
        yline(ax,  D.max_rpm, '--k', 'LineWidth', 0.8); yline(ax, -D.max_rpm, '--k', 'LineWidth', 0.8);
    end
    ylabel(ax, 'wheel command [rpm]'); finish(ax, tend, th); xlabel(ax, 'time [s]');
    legend(ax, {'wheel 1', 'wheel 2', 'wheel 3', 'wheel 4'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'Box', 'off');
    paper_export(f, fullfile(OUT, sprintf('fig_9dof_case_%d_commands.pdf', sc)));
    close(f);
end
fclose(fid);
fprintf('written figures/fig_9dof_case_1..4_path.pdf, _error.pdf, _commands.pdf and fig_9dof_cases_values.txt\n');
end

function finish(ax, tend, th)
xline(ax, th, ':k');
xlim(ax, [0 tend]);
end
