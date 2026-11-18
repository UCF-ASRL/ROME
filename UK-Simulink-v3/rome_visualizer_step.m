function rome_visualizer_step(q_cmd)
% ROME_VISUALIZER_STEP Live Simulink 3D visualizer for ROME (9-DOF)
%   Takes q_cmd (9x1: [x; y; yaw; q1..q6] in meters and IK9 Model radians)
%   and updates the 3D URDF + Ground Vehicle scene each simulation step.
%   No joint or ground vehicle limits are applied.

    persistent S
    if nargin < 1 || isempty(q_cmd)
        q_cmd = [0; 0; 0; deg2rad([-3.0; 127.0; -10.0; 0.0; -27.0; 0.0])];
    end
    q_cmd = double(q_cmd(:));

    % Initialize figure and graphics handles on first call or if window was closed
    if isempty(S) || ~isfield(S, 'fig') || ~isgraphics(S.fig)
        S = initVisualizer();
    end

    % Update scene with current q_cmd (no clamping/limits)
    updateScene(S, q_cmd);
    drawnow limitrate;

    %% ====================================================================
    %  INITIALIZATION (Runs once on first time step)
    %  ====================================================================
    function S = initVisualizer()
        S.base_z0 = 0.20; % Height of arm_base_link above floor (m)

        % URDF Joint Origins: [x, y, z, roll, pitch, yaw] (m, rad)
        S.urdf_origins = [
            0,       0,        0,        pi,      0,   0;       % arm_joint_1
            0,       0.0642,  -0.16977,  pi/2,    0,  -pi/2;    % arm_joint_2
            0,      -0.305,    0.007,    0,       0,   pi;      % arm_joint_3
            0,       0,        0,        pi/2,    0,  -pi/2;    % arm_joint_4
            0,       0,       -0.22263,  pi,      0,  -pi/2;    % arm_joint_5
            0,       0,        0.041,    0,       0,   pi       % arm_joint_6
        ];

        % URDF Joint Axes in local joint frame
        S.urdf_axes = [
            0,  0,  1;  % arm_joint_1
            0,  0, -1;  % arm_joint_2
            0,  0, -1;  % arm_joint_3
            0,  0, -1;  % arm_joint_4
            1,  0,  0;  % arm_joint_5
            0,  0,  1   % arm_joint_6
        ];

        % Mapping: IK9 Model <-> Firmware <-> ROS 2 URDF (unclamped)
        S.model_sign   = [1,  -1, -1, 1, 1, 1];
        S.model_offset = [0, 180, 90, 0, 0, 0];
        S.urdf_sign    = [1,   1, -1, 1, 1, 1];
        S.urdf_offset  = [0,  42, 52, 0, 0, 0];

        % Ground Vehicle Geometry
        S.gv.r      = 0.0762;
        S.gv.l      = 0.35;
        S.gv.d      = 0.45;
        S.gv.w      = 0.45;
        S.gv.alphas = deg2rad([315, 225, 135, 45]);

        % Link Color Sequence
        S.colors = [
            0.38, 0.42, 0.48;  % base_link : Slate Gray
            0.12, 0.47, 0.71;  % link_1    : Royal Blue
            0.09, 0.63, 0.52;  % link_2    : Teal
            0.17, 0.63, 0.17;  % link_3    : Emerald Green
            0.95, 0.62, 0.07;  % link_4    : Amber / Gold
            0.84, 0.37, 0.00;  % link_5    : Coral Orange
            0.58, 0.40, 0.74   % link_6    : Amethyst Purple
        ];

        % Create or Reuse Figure Window
        fig = findobj('Type', 'figure', 'Tag', 'ROMSimulinkVisualizer');
        if isempty(fig)
            fig = figure('Name', 'ROME 9-DOF Live Simulink Visualizer', ...
                         'Tag', 'ROMSimulinkVisualizer', ...
                         'NumberTitle', 'off', ...
                         'Color', [0.95 0.95 0.96], ...
                         'Position', [80, 80, 1050, 820]);
        else
            figure(fig(1));
            fig = fig(1);
            clf(fig);
        end
        S.fig = fig;

        % 3D Viewport Axes
        ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.05, 0.06, 0.90, 0.80]);
        hold(ax, 'on');
        grid(ax, 'on');
        box(ax, 'on');
        axis(ax, 'equal');
        view(ax, -125, 22);
        xlabel(ax, 'World X (m)', 'FontWeight', 'bold');
        ylabel(ax, 'World Y (m)', 'FontWeight', 'bold');
        zlabel(ax, 'World Z (m)', 'FontWeight', 'bold');
        camlight(ax, 'headlight');
        lighting(ax, 'gouraud');
        rotate3d(ax, 'on');
        S.ax = ax;

        % Top Telemetry Bar & Quick View/Visual Controls
        S.ui.teleTxt = uicontrol(fig, 'Style', 'text', ...
            'String', 'Waiting for Simulink data...', ...
            'Units', 'normalized', 'Position', [0.05, 0.88, 0.58, 0.10], ...
            'HorizontalAlignment', 'left', 'FontName', 'FixedWidth', ...
            'FontSize', 9, 'BackgroundColor', [1 1 1]);

        S.ui.followChk = uicontrol(fig, 'Style', 'checkbox', ...
            'String', 'Follow Base', 'Value', 0, ...
            'Units', 'normalized', 'Position', [0.65, 0.94, 0.11, 0.035], ...
            'BackgroundColor', [0.95 0.95 0.96]);

        S.ui.meshChk = uicontrol(fig, 'Style', 'checkbox', ...
            'String', 'Show STL', 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.77, 0.94, 0.10, 0.035], ...
            'BackgroundColor', [0.95 0.95 0.96]);

        S.ui.triadChk = uicontrol(fig, 'Style', 'checkbox', ...
            'String', 'Axes', 'Value', 1, ...
            'Units', 'normalized', 'Position', [0.88, 0.94, 0.08, 0.035], ...
            'BackgroundColor', [0.95 0.95 0.96]);

        uicontrol(fig, 'Style', 'pushbutton', 'String', 'Side View', ...
            'Units', 'normalized', 'Position', [0.65, 0.885, 0.09, 0.04], ...
            'Callback', @(~,~) view(ax, -90, 0));
        uicontrol(fig, 'Style', 'pushbutton', 'String', '3D Iso View', ...
            'Units', 'normalized', 'Position', [0.75, 0.885, 0.09, 0.04], ...
            'Callback', @(~,~) view(ax, -125, 22));
        uicontrol(fig, 'Style', 'pushbutton', 'String', 'Clear Trail', ...
            'Units', 'normalized', 'Position', [0.85, 0.885, 0.10, 0.04], ...
            'Callback', @(~,~) clearpoints(S.gfx.eeTrail));

        % Floor Patch
        patch(ax, [-2.5 2.5 2.5 -2.5], [-2.5 -2.5 2.5 2.5], [0 0 0 0], ...
              [0.88 0.89 0.91], 'FaceAlpha', 0.25, 'EdgeColor', [0.7 0.7 0.7]);

        % Ground Vehicle Chassis Box
        S.gfx.gvBox = patch(ax, 'Vertices', zeros(8,3), ...
            'Faces', [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8], ...
            'FaceColor', [0.25 0.32 0.42], 'FaceAlpha', 0.55, ...
            'EdgeColor', [0.1 0.15 0.2], 'LineWidth', 1.2);

        S.gfx.gvHeadingX = plot3(ax, [0 0], [0 0], [0 0], 'r-', 'LineWidth', 2.0);
        S.gfx.gvHeadingY = plot3(ax, [0 0], [0 0], [0 0], 'g-', 'LineWidth', 2.0);

        % 4 Omni Wheels
        for wIdx = 1:4
            S.gfx.wheelStrut(wIdx) = plot3(ax, [0 0], [0 0], [0 0], 'k-', 'LineWidth', 2.0);
            S.gfx.wheelRim(wIdx)   = patch(ax, 'XData', [0 0 0], 'YData', [0 0 0], 'ZData', [0 0 0], ...
                                           'FaceColor', [0.18 0.18 0.18], 'EdgeColor', 'k', 'LineWidth', 1.5);
        end

        % STL Mesh Patches
        for mIdx = 1:7
            S.gfx.meshPatch(mIdx) = patch(ax, 'Faces', [], 'Vertices', [], ...
                                          'FaceColor', S.colors(mIdx, :), ...
                                          'EdgeColor', 'none', ...
                                          'FaceAlpha', 0.88, ...
                                          'AmbientStrength', 0.45, ...
                                          'DiffuseStrength', 0.75, ...
                                          'SpecularStrength', 0.25);
        end

        % Kinematic Skeleton Links
        for cIdx = 1:7
            S.gfx.skelLink(cIdx) = plot3(ax, [0 0], [0 0], [0 0], '-', ...
                                         'Color', S.colors(cIdx, :), 'LineWidth', 6.5);
        end

        % Joint Markers, End-Effector Marker & Live Trajectory Trail
        S.gfx.physJoints = plot3(ax, zeros(1,6), zeros(1,6), zeros(1,6), ...
                                 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.15 0.15 0.15], ...
                                 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
        S.gfx.eeMarker   = plot3(ax, 0, 0, 0, 'p', 'MarkerSize', 14, ...
                                 'MarkerFaceColor', [0.1 0.85 0.25], 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
        S.gfx.eeTrail    = animatedline(ax, 'Color', [0.1 0.75 0.25], 'LineWidth', 1.5);

        % Joint Labels & Axis Vectors
        for jIdx = 1:6
            S.gfx.axisLine(jIdx) = plot3(ax, [0 0], [0 0], [0 0], 'k--', 'LineWidth', 1.6);
            S.gfx.jointTxt(jIdx) = text(ax, 0, 0, 0, sprintf('J%d', jIdx), ...
                                        'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
        end

        % End-Effector RGB Triad
        S.gfx.eeTriadX = plot3(ax, [0 0], [0 0], [0 0], 'r-', 'LineWidth', 2.2);
        S.gfx.eeTriadY = plot3(ax, [0 0], [0 0], [0 0], 'g-', 'LineWidth', 2.2);
        S.gfx.eeTriadZ = plot3(ax, [0 0], [0 0], [0 0], 'b-', 'LineWidth', 2.2);

        % Auto-load STL meshes
        [S.meshes_loaded, S.mesh_data] = tryLoadStlMeshes(pwd);
        if S.meshes_loaded
            for m = 1:7
                set(S.gfx.meshPatch(m), 'Faces', S.mesh_data{m}.Faces, ...
                                        'Vertices', S.mesh_data{m}.Vertices);
            end
        end
    end

    %% ====================================================================
    %  PER-STEP SCENE UPDATE (No Limits Applied)
    %  ====================================================================
    function updateScene(S, q)
        x_gv = q(1);
        y_gv = q(2);
        th   = q(3);
        q_arm_model_deg = rad2deg(q(4:9))'; % 1x6 IK9 model degrees

        % Unclamped conversion: IK9 Model (deg) -> Firmware (deg) -> URDF (rad)
        fw_deg   = S.model_sign .* q_arm_model_deg + S.model_offset;
        urdf_deg = (fw_deg - S.urdf_offset) ./ S.urdf_sign;
        urdf_rad = deg2rad(urdf_deg);

        Rz_base = [cos(th), -sin(th), 0;
                   sin(th),  cos(th), 0;
                   0,        0,       1];

        % Forward Kinematics along URDF chain
        T_links = zeros(4, 4, 7);
        T_links(:, :, 1) = [Rz_base, [x_gv; y_gv; S.base_z0]; 0, 0, 0, 1];

        jointWorldAxes = zeros(3, 6);
        for j = 1:6
            T_origin = urdfOriginTransform(S.urdf_origins(j, :));
            T_pre    = T_links(:, :, j) * T_origin;
            jointWorldAxes(:, j) = T_pre(1:3, 1:3) * S.urdf_axes(j, :)';
            T_rot    = axisAngleTransform(S.urdf_axes(j, :), urdf_rad(j));
            T_links(:, :, j+1) = T_pre * T_rot;
        end

        T_ee = T_links(:, :, 7);
        pe   = T_ee(1:3, 4);
        Re   = T_ee(1:3, 1:3);

        showMeshes = logical(get(S.ui.meshChk, 'Value'));
        showTriads = logical(get(S.ui.triadChk, 'Value'));
        autoFollow = logical(get(S.ui.followChk, 'Value'));

        % 1. Update STL Meshes
        for m = 1:7
            if S.meshes_loaded && showMeshes
                Tm = T_links(:, :, m);
                V_local = S.mesh_data{m}.Vertices;
                V_world = (Tm(1:3, 1:3) * V_local')' + Tm(1:3, 4)';
                set(S.gfx.meshPatch(m), 'Vertices', V_world, 'Visible', 'on');
            else
                set(S.gfx.meshPatch(m), 'Visible', 'off');
            end
        end

        % 2. Update Kinematic Skeleton
        P_base      = T_links(1:3, 4, 1);
        P_J1        = T_links(1:3, 4, 2);
        P_L1_corner = T_links(1:3, 4, 2) + T_links(1:3, 1:3, 2) * [0; 0.0642; 0];
        P_J2        = T_links(1:3, 4, 3);
        P_J3        = T_links(1:3, 4, 4);
        P_J4        = P_J3 + T_links(1:3, 1:3, 5) * [0; 0; -0.085];
        P_J5        = T_links(1:3, 4, 6);
        P_J6        = T_links(1:3, 4, 7);

        set(S.gfx.skelLink(1), 'XData', [P_base(1) P_L1_corner(1)], 'YData', [P_base(2) P_L1_corner(2)], 'ZData', [P_base(3) P_L1_corner(3)]);
        set(S.gfx.skelLink(2), 'XData', [P_L1_corner(1) P_J2(1)],   'YData', [P_L1_corner(2) P_J2(2)],   'ZData', [P_L1_corner(3) P_J2(3)]);
        set(S.gfx.skelLink(3), 'XData', [P_J2(1) P_J3(1)],          'YData', [P_J2(2) P_J3(2)],          'ZData', [P_J2(3) P_J3(3)]);
        set(S.gfx.skelLink(4), 'XData', [P_J3(1) P_J4(1)],          'YData', [P_J3(2) P_J4(2)],          'ZData', [P_J3(3) P_J4(3)]);
        set(S.gfx.skelLink(5), 'XData', [P_J4(1) P_J5(1)],          'YData', [P_J4(2) P_J5(2)],          'ZData', [P_J4(3) P_J5(3)]);
        set(S.gfx.skelLink(6), 'XData', [P_J5(1) P_J6(1)],          'YData', [P_J5(2) P_J6(2)],          'ZData', [P_J5(3) P_J6(3)]);
        pTip = P_J6 + Re(:,3)*0.015;
        set(S.gfx.skelLink(7), 'XData', [P_J6(1) pTip(1)],          'YData', [P_J6(2) pTip(2)],          'ZData', [P_J6(3) pTip(3)]);

        % Update Joint Markers & EE Trail
        jPts = [P_J1, P_J2, P_J3, P_J4, P_J5, P_J6];
        set(S.gfx.physJoints, 'XData', jPts(1,:), 'YData', jPts(2,:), 'ZData', jPts(3,:));
        set(S.gfx.eeMarker,   'XData', pe(1), 'YData', pe(2), 'ZData', pe(3));
        if all(isfinite(pe))
            addpoints(S.gfx.eeTrail, pe(1), pe(2), pe(3));
        end

        axLen = 0.055;
        for jIdx = 1:6
            pJ = jPts(:, jIdx);
            uA = jointWorldAxes(:, jIdx);
            if showTriads
                pA1 = pJ - uA * axLen;
                pA2 = pJ + uA * axLen;
                set(S.gfx.axisLine(jIdx), 'XData', [pA1(1) pA2(1)], 'YData', [pA1(2) pA2(2)], 'ZData', [pA1(3) pA2(3)], 'Visible', 'on');
                set(S.gfx.jointTxt(jIdx), 'Position', (pJ + [0; 0; 0.032])', 'Visible', 'on');
            else
                set(S.gfx.axisLine(jIdx), 'Visible', 'off');
                set(S.gfx.jointTxt(jIdx), 'Visible', 'off');
            end
        end

        % End-Effector RGB Triad
        eeLen = 0.065;
        xEE = pe + Re(:,1)*eeLen;
        yEE = pe + Re(:,2)*eeLen;
        zEE = pe + Re(:,3)*eeLen;
        set(S.gfx.eeTriadX, 'XData', [pe(1) xEE(1)], 'YData', [pe(2) xEE(2)], 'ZData', [pe(3) xEE(3)]);
        set(S.gfx.eeTriadY, 'XData', [pe(1) yEE(1)], 'YData', [pe(2) yEE(2)], 'ZData', [pe(3) yEE(3)]);
        set(S.gfx.eeTriadZ, 'XData', [pe(1) zEE(1)], 'YData', [pe(2) zEE(2)], 'ZData', [pe(3) zEE(3)]);

        % 3. Update Ground Vehicle Geometry
        hx = S.gv.d / 2;
        hy = S.gv.w / 2;
        zBot = S.gv.r;
        zTop = S.base_z0;
        localCorners = [-hx, -hy, zBot;  hx, -hy, zBot;  hx,  hy, zBot; -hx,  hy, zBot;
                        -hx, -hy, zTop;  hx, -hy, zTop;  hx,  hy, zTop; -hx,  hy, zTop];
        worldCorners = (Rz_base * localCorners')' + [x_gv, y_gv, 0];
        set(S.gfx.gvBox, 'Vertices', worldCorners);

        pTop = [x_gv; y_gv; zTop];
        pX = pTop + Rz_base * [0.26; 0; 0];
        pY = pTop + Rz_base * [0; 0.26; 0];
        set(S.gfx.gvHeadingX, 'XData', [pTop(1) pX(1)], 'YData', [pTop(2) pX(2)], 'ZData', [pTop(3) pX(3)]);
        set(S.gfx.gvHeadingY, 'XData', [pTop(1) pY(1)], 'YData', [pTop(2) pY(2)], 'ZData', [pTop(3) pY(3)]);

        thDisk = linspace(0, 2*pi, 24);
        for wIdx = 1:4
            ang = S.gv.alphas(wIdx);
            wLocalCenter = [S.gv.l * sin(ang); S.gv.l * cos(ang); S.gv.r];
            wWorldCenter = [x_gv; y_gv; 0] + Rz_base * wLocalCenter;

            cHub = [x_gv; y_gv; S.gv.r];
            set(S.gfx.wheelStrut(wIdx), 'XData', [cHub(1) wWorldCenter(1)], ...
                                        'YData', [cHub(2) wWorldCenter(2)], ...
                                        'ZData', [cHub(3) wWorldCenter(3)]);

            tangentDir = Rz_base * [cos(ang); -sin(ang); 0];
            circlePts = wWorldCenter + S.gv.r * (tangentDir * cos(thDisk) + [0;0;1] * sin(thDisk));
            set(S.gfx.wheelRim(wIdx), 'XData', circlePts(1,:), 'YData', circlePts(2,:), 'ZData', circlePts(3,:));
        end

        % 4. Viewport Bounds
        if autoFollow
            span = 0.65;
            xlim(S.ax, [x_gv - span, x_gv + span]);
            ylim(S.ax, [y_gv - span, y_gv + span]);
            zlim(S.ax, [0, 0.90]);
        else
            xlim(S.ax, [-2.5, 2.5]);
            ylim(S.ax, [-2.5, 2.5]);
            zlim(S.ax, [0, 1.2]);
        end

        % 5. Live Telemetry Readout
        teleStr = sprintf([ ...
            'Base Pose:  x = %+6.3f m,  y = %+6.3f m,  yaw = %+6.1f°   |   EE World (m): [ %+6.3f, %+6.3f, %+6.3f ]\n', ...
            'Model (°):    [%6.1f %6.1f %6.1f %6.1f %6.1f %6.1f]\n', ...
            'Firmware (°): [%6.1f %6.1f %6.1f %6.1f %6.1f %6.1f]   |   URDF (°): [%6.1f %6.1f %6.1f %6.1f %6.1f %6.1f]'], ...
            x_gv, y_gv, rad2deg(th), pe(1), pe(2), pe(3), ...
            q_arm_model_deg(1), q_arm_model_deg(2), q_arm_model_deg(3), q_arm_model_deg(4), q_arm_model_deg(5), q_arm_model_deg(6), ...
            fw_deg(1), fw_deg(2), fw_deg(3), fw_deg(4), fw_deg(5), fw_deg(6), ...
            urdf_deg(1), urdf_deg(2), urdf_deg(3), urdf_deg(4), urdf_deg(5), urdf_deg(6));
        set(S.ui.teleTxt, 'String', teleStr);
    end
end

%% ========================================================================
%  KINEMATIC & STL HELPERS
%  ========================================================================
function T = urdfOriginTransform(xyz_rpy)
    x = xyz_rpy(1); y = xyz_rpy(2); z = xyz_rpy(3);
    rr = xyz_rpy(4); pp = xyz_rpy(5); yy = xyz_rpy(6);
    Rx = [1, 0, 0; 0, cos(rr), -sin(rr); 0, sin(rr), cos(rr)];
    Ry = [cos(pp), 0, sin(pp); 0, 1, 0; -sin(pp), 0, cos(pp)];
    Rz = [cos(yy), -sin(yy), 0; sin(yy), cos(yy), 0; 0, 0, 1];
    T = [Rz * Ry * Rx, [x; y; z]; 0, 0, 0, 1];
end

function T = axisAngleTransform(axVec, theta)
    v = axVec(:) / norm(axVec);
    kx = v(1); ky = v(2); kz = v(3);
    K = [0, -kz, ky; kz, 0, -kx; -ky, kx, 0];
    R = eye(3) + sin(theta)*K + (1 - cos(theta))*(K*K);
    T = [R, [0;0;0]; 0, 0, 0, 1];
end

function [foundAll, loaded] = tryLoadStlMeshes(baseDir)
    stlNames = {'base_link.STL', 'link_1.STL', 'link_2.STL', ...
                'link_3.STL', 'link_4.STL', 'link_5.STL', 'link_6.STL'};
    searchDirs = {baseDir, ...
                  fullfile(baseDir, 'meshes', 'ar4_mk3'), ...
                  fullfile(baseDir, 'ar4_mk3')};
    foundAll = false;
    loaded = cell(1, 7);
    for dIdx = 1:numel(searchDirs)
        folder = searchDirs{dIdx};
        if ~isfolder(folder), continue; end
        ok = true;
        for m = 1:7
            fPath = fullfile(folder, stlNames{m});
            if ~isfile(fPath)
                fPathLow = fullfile(folder, lower(stlNames{m}));
                if isfile(fPathLow)
                    fPath = fPathLow;
                else
                    ok = false;
                    break;
                end
            end
            try
                TR = stlread(fPath);
                if isstruct(TR)
                    loaded{m}.Faces = TR.faces;
                    loaded{m}.Vertices = TR.vertices;
                else
                    loaded{m}.Faces = TR.ConnectivityList;
                    loaded{m}.Vertices = TR.Points;
                end
            catch
                ok = false;
                break;
            end
        end
        if ok
            foundAll = true;
            break;
        end
    end
end