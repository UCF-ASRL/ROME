function out = play_run_compare(runFile, opts)
%PLAY_RUN_COMPARE  Replay a saved run with three arms in one 3D axes: model belief and the real AR3 under both J2 maps.
%   out = play_run_compare(runFile, opts)
%   runFile  .mat with t (Nx1 s) and q (Nx9, [x y yaw J1..J6], m and model rad), e.g. runs/case_1.mat
%   opts (all optional)
%     frames    indices into t to draw (default: all)
%     outDir    folder; one PNG per drawn frame is written there (default: none)
%     visible   true (default) shows the figure; false draws off-screen (headless)
%     pause     seconds between frames when visible and no outDir (default 0.02)
%     measured  second q trace drawn as a grey ghost (model chain, dotted): struct with q (Mx9) and optional t,
%               or an Mx9 matrix. With t it is interpolated onto the run's t; without, row k is paired with
%               frame k (sizes must match). Format of a future recorder file: NOT VERIFIED (save_run.m only
%               says raw telemetry travels inside out.telemetry; no field list was relied on).
%     baseHalf  half side of the drawn base square (m), default 0.26 (illustrative, taken from the old
%               visualizer's pX/pY marks; the real footprint is NOT VERIFIED)
%   out: struct t, frames, tipA, tipB, tipC (3xK world tips), dAB, dAC, dBC (1xK tip separations, m), mapNames, fw
%   Arms. A = model belief: ik9_fk chain (frame origins 0..6; placeholder DH). B, C = real AR3: ar3_fk chain
%   (vendor DH) fed with the model joints converted model rad -> firmware deg (fw = arm_sign.*deg +
%   arm_offset_deg, parsed from define_constants.m text) -> ARCS deg by romefw_to_arcs_deg with each candidate J2
%   map ('rome_dirpin', 'arcs_caldir'; TRACKER 0, 19 Nov J2 row, undecided until bench item 24b). The AR3 base
%   frame is put at the model's arm mount (base position, 0.20 m up, turned with the yaw), the same placement as
%   docs/verification/visualizer/viz_run_case1.m. Nothing here runs the model, define_constants or sim.
if nargin < 2 || isempty(opts), opts = struct(); end
rmDir = fileparts(fileparts(mfilename('fullpath')));            % ROME_merged
addpath(rmDir, fullfile(rmDir, 'analysis'));
S = load(runFile, 't', 'q');  t = S.t(:);  q = S.q;
assert(size(q,2) == 9 && size(q,1) == numel(t), 'play_run_compare:run', 'q must be Nx9 and match t');
frames   = getopt(opts, 'frames', 1:numel(t));
outDir   = getopt(opts, 'outDir', '');
visible  = getopt(opts, 'visible', true);
pauseS   = getopt(opts, 'pause', 0.02);
bh       = getopt(opts, 'baseHalf', 0.26);
mapNames = {'rome_dirpin', 'arcs_caldir'};
[sg, off] = viewer_arm_map(rmDir);                              % 6x1 each
G = [];                                                         % ghost trace (Nx9) or empty
if isfield(opts, 'measured') && ~isempty(opts.measured)
    M = opts.measured;
    if isstruct(M)
        if isfield(M, 't') && ~isempty(M.t), G = interp1(M.t(:), M.q, t, 'linear', NaN); else, G = M.q; end
    else
        G = M;
    end
    assert(size(G,1) == numel(t) && size(G,2) == 9, 'play_run_compare:measured', 'measured must give %d x 9', numel(t));
end
if ~isempty(outDir) && ~exist(outDir, 'dir'), mkdir(outDir); end
K = numel(frames);
out = struct('t', t(frames), 'frames', frames(:)', 'mapNames', {mapNames}, 'fw', zeros(6,K));
out.tipA = zeros(3,K); out.tipB = zeros(3,K); out.tipC = zeros(3,K);
out.dAB = zeros(1,K); out.dAC = zeros(1,K); out.dBC = zeros(1,K);
fig = figure('Visible', onoff(visible), 'Color', 'w', 'Position', [100 100 900 700]);
ax = axes(fig);
xy = q(:,1:2);  half = 1.0 + bh;
lim = [min(xy(:,1))-half max(xy(:,1))+half min(xy(:,2))-half max(xy(:,2))+half];
cA = [0.85 0.33 0.10];  cB = [0.00 0.45 0.74];  cC = [0.47 0.67 0.19];  cG = [0.5 0.5 0.5];
fprintf('frame  t(s)   |A-B|(m)  |A-C|(m)  |B-C|(m)   [A model ik9_fk, B %s, C %s]\n', mapNames{1}, mapNames{2});
for n = 1:K
    k = frames(n);  qk = q(k,:)';
    PA = chainA(qk);
    fw = sg .* rad2deg(qk(4:9)) + off;
    PB = chainAR3(qk, fw, mapNames{1});  PC = chainAR3(qk, fw, mapNames{2});
    out.fw(:,n) = fw;  out.tipA(:,n) = PA(:,end);  out.tipB(:,n) = PB(:,end);  out.tipC(:,n) = PC(:,end);
    out.dAB(n) = norm(PA(:,end)-PB(:,end));  out.dAC(n) = norm(PA(:,end)-PC(:,end));  out.dBC(n) = norm(PB(:,end)-PC(:,end));
    fprintf('%5d %6.2f   %8.4f  %8.4f  %8.4f\n', k, t(k), out.dAB(n), out.dAC(n), out.dBC(n));
    cla(ax);  hold(ax, 'on');
    Rb = [cos(qk(3)) -sin(qk(3)); sin(qk(3)) cos(qk(3))];
    sq = [qk(1);qk(2)] + Rb * [bh bh -bh -bh bh; bh -bh -bh bh bh];
    plot3(ax, sq(1,:), sq(2,:), zeros(1,5), 'k-', 'LineWidth', 1.5, 'DisplayName', 'base footprint');
    hd = [qk(1);qk(2)] + Rb * [0 bh*1.6; 0 0];
    plot3(ax, hd(1,:), hd(2,:), [0 0], 'k-', 'LineWidth', 3, 'DisplayName', 'heading (+x of base)');
    plot3(ax, xy(:,1), xy(:,2), zeros(size(xy,1),1), ':', 'Color', [0.75 0.75 0.75], 'HandleVisibility', 'off');
    if ~isempty(G) && all(isfinite(G(k,:)))
        PG = chainA(G(k,:)');  RG = [cos(G(k,3)) -sin(G(k,3)); sin(G(k,3)) cos(G(k,3))];
        sg2 = [G(k,1);G(k,2)] + RG * [bh bh -bh -bh bh; bh -bh -bh bh bh];
        plot3(ax, sg2(1,:), sg2(2,:), zeros(1,5), ':', 'Color', cG, 'HandleVisibility', 'off');
        plot3(ax, PG(1,:), PG(2,:), PG(3,:), ':o', 'Color', cG, 'LineWidth', 1.5, 'DisplayName', 'measured (ghost, model chain)');
    end
    plot3(ax, PA(1,:), PA(2,:), PA(3,:), '-o', 'Color', cA, 'LineWidth', 2, 'MarkerSize', 4, 'DisplayName', 'A model belief (ik9_fk)');
    plot3(ax, PB(1,:), PB(2,:), PB(3,:), '-s', 'Color', cB, 'LineWidth', 2, 'MarkerSize', 4, 'DisplayName', ['B real AR3, ' mapNames{1}]);
    plot3(ax, PC(1,:), PC(2,:), PC(3,:), '-d', 'Color', cC, 'LineWidth', 2, 'MarkerSize', 4, 'DisplayName', ['C real AR3, ' mapNames{2}]);
    plot3(ax, PA(1,end), PA(2,end), PA(3,end), 'p', 'Color', cA, 'MarkerSize', 12, 'MarkerFaceColor', cA, 'HandleVisibility', 'off');
    plot3(ax, PB(1,end), PB(2,end), PB(3,end), 'p', 'Color', cB, 'MarkerSize', 12, 'MarkerFaceColor', cB, 'HandleVisibility', 'off');
    plot3(ax, PC(1,end), PC(2,end), PC(3,end), 'p', 'Color', cC, 'MarkerSize', 12, 'MarkerFaceColor', cC, 'HandleVisibility', 'off');
    axis(ax, 'equal');  xlim(ax, lim(1:2));  ylim(ax, lim(3:4));  zlim(ax, [0 1.2]);
    grid(ax, 'on');  view(ax, 40, 25);  xlabel(ax, 'x (m)');  ylabel(ax, 'y (m)');  zlabel(ax, 'z (m)');
    legend(ax, 'Location', 'northeastoutside', 'Interpreter', 'none');
    title(ax, sprintf('%s   t = %.2f s (frame %d)\ntip separation (m): A-B %.3f   A-C %.3f   B-C %.3f', ...
        strrep(runName(runFile), '_', '\_'), t(k), k, out.dAB(n), out.dAC(n), out.dBC(n)));
    hold(ax, 'off');
    if ~isempty(outDir)
        fn = fullfile(outDir, sprintf('%s_frame%04d_t%06.2fs.png', runName(runFile), k, t(k)));
        try
            exportgraphics(fig, fn, 'Resolution', 100);
        catch
            saveas(fig, fn);
        end
    end
    drawnow;
    if visible && isempty(outDir), pause(pauseS); end
end
if ~visible, close(fig); end
end

function P = chainA(qk)
% model chain: ik9_fk frame origins 0..6 in the base body frame -> world
[~, ~, Pb] = ik9_fk(qk);
Rb = [cos(qk(3)) -sin(qk(3)) 0; sin(qk(3)) cos(qk(3)) 0; 0 0 1];
P = Rb * Pb + [qk(1); qk(2); 0];
end

function P = chainAR3(qk, fw, mapName)
% real AR3 chain: ar3_fk frame origins in its base frame, placed at the model's arm mount (0.20 m up)
[~, Pa] = ar3_fk(romefw_to_arcs_deg(fw, mapName));
Rb = [cos(qk(3)) -sin(qk(3)) 0; sin(qk(3)) cos(qk(3)) 0; 0 0 1];
P = Rb * Pa + [qk(1); qk(2); 0.20];
end

function v = getopt(o, f, d)
if isfield(o, f) && ~isempty(o.(f)), v = o.(f); else, v = d; end
end

function s = onoff(b)
if b, s = 'on'; else, s = 'off'; end
end

function n = runName(f)
[~, n] = fileparts(char(f));
end
