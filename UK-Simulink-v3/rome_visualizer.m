function rome_visualizer()
% ROME_VISUALIZER Interactive 3D kinematic visualizer for ROME
%   Combines the 3-DOF Ground Vehicle with the 6-DOF Robotic Arm using the
%   exact ROS 2 URDF (ar4_mk3) kinematic chain, STL mesh rendering, and
%   Teensy Firmware / URDF / Model angle space conversions.

    %% 1. ROME System Parameters & URDF Kinematics
    S.base_z0 = 0.20; % Height of arm_base_link above Ground Vehicle floor (m)

    % URDF Joint Origins: [x, y, z, roll, pitch, yaw] (meters and radians)
    %   RPY in URDF is fixed-axis extrinsic X-Y-Z: R = Rz(yaw)*Ry(pitch)*Rx(roll)
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

    % Teensy Firmware Limits (from ROME_Teensy_Code.ino)
    S.fw_switch = [-180.0, 132.0, 141.0, -165.0,  90.0,  180.0];
    S.fw_other  = [ 160.0,   0.0,   1.0,  165.0, -90.0, -170.0];
    S.fw_min    = min(S.fw_switch, S.fw_other);
    S.fw_max    = max(S.fw_switch, S.fw_other);
    S.fw_margin = 2.0; % deg

    % Mapping 1: Firmware <-> IK9 Model (fw_deg = arm_sign .* model_deg + arm_offset_deg)
    S.model_sign   = [1,  -1, -1, 1, 1, 1];
    S.model_offset = [0, 180, 90, 0, 0, 0];

    % Mapping 2: Firmware <-> ROS 2 URDF (fw_deg = urdf_sign .* urdf_deg + urdf_offset_deg)
    %   URDF J2=0 is vertical up (-42 deg back to +90 deg forward -> FW 0 to 132 deg)
    %   URDF J3=0 is bent 90 deg forward (+52 deg folded to -89 deg extended -> FW 0 to 141 deg)
    S.urdf_sign   = [1,  1, -1, 1, 1, 1];
    S.urdf_offset = [0, 42, 52, 0, 0, 0];

    % Ground Vehicle Geometry
    S.gv.r      = 0.0762; % Wheel radius (m)
    S.gv.l      = 0.35;   % C.G. to wheels (m)
    S.gv.d      = 0.45;   % Side length (m)
    S.gv.w      = 0.45;   % Side width (m)
    S.gv.alphas = deg2rad([315, 225, 135, 45]); % Angle from +y axis

    % Standard Engineering Color Sequence (7 links: base_link + link_1..link_6)
    S.colors = [
        0.38, 0.42, 0.48;  % base_link : Slate Gray
        0.12, 0.47, 0.71;  % link_1    : Royal Blue
        0.09, 0.63, 0.52;  % link_2    : Teal
        0.17, 0.63, 0.17;  % link_3    : Emerald Green
        0.95, 0.62, 0.07;  % link_4    : Amber / Gold
        0.84, 0.37, 0.00;  % link_5    : Coral Orange
        0.58, 0.40, 0.74   % link_6    : Amethyst Purple
    ];

    % Internal State:
    %   S.gv_pose = [x (m); y (m); yaw (rad)]
    %   S.fw_deg  = [J1; J2; J3; J4; J5; J6] stored in Teensy Firmware degrees
    %     Stowed Pose: J2 = 15 deg (pitched back), J3 = 138 deg (folded forward/down), J5 = 35 deg
    S.gv_pose = [0; 0; 0];
    S.fw_deg  = [0; 15; 138; 0; 35; 0];

    % UI State Flags
    S.unit_mode   = 'deg';      % 'deg' or 'rad'
    S.angle_space = 'firmware'; % 'firmware', 'urdf', or 'model'
    S.use_margin  = false;
    S.show_triads = true;
    S.show_meshes = true;
    S.auto_zoom   = true;
    S.meshes_loaded = false;
    S.mesh_data   = cell(1, 7);

    %% 2. Build Figure & Layout
    fig = figure('Name', 'ROME 9-DOF Kinematic Visualizer (ROS 2 URDF + STL)', ...
                 'NumberTitle', 'off', ...
                 'Color', [0.95 0.95 0.96], ...
                 'Position', [50, 50, 1420, 860]);

    % Left Panel: 3D Viewport
    ax = axes('Parent', fig, ...
              'Units', 'normalized', ...
              'Position', [0.04, 0.07, 0.56, 0.88]);
    hold(ax, 'on');
    grid(ax, 'on');
    box(ax, 'on');
    axis(ax, 'equal');
    % Side view matching your sketch (+Y of arm_base_link points LEFT)
    view(ax, -90, 0);
    xlabel(ax, 'World X (m)', 'FontWeight', 'bold');
    ylabel(ax, 'World Y (m)', 'FontWeight', 'bold');
    zlabel(ax, 'World Z (m)', 'FontWeight', 'bold');
    title(ax, 'ROME System (Ground Vehicle + Robotic Arm URDF)', 'FontSize', 13, 'FontWeight', 'bold');
    camlight(ax, 'headlight');
    lighting(ax, 'gouraud');
    rotate3d(ax, 'on');

    % Right Panel: Controls Container
    ctrlPanel = uipanel('Parent', fig, ...
                        'Title', 'ROME Configuration & Controls', ...
                        'FontSize', 11, ...
                        'FontWeight', 'bold', ...
                        'BackgroundColor', [0.95 0.95 0.96], ...
                        'Position', [0.62, 0.02, 0.36, 0.96]);

    %% 3. Top Option Toggles & View Buttons
    uicontrol(ctrlPanel, 'Style', 'text', 'String', 'Units:', ...
              'Units', 'normalized', 'Position', [0.03, 0.95, 0.12, 0.025], ...
              'HorizontalAlignment', 'left', 'BackgroundColor', [0.95 0.95 0.96], 'FontWeight', 'bold');
    S.ui.unitPopup = uicontrol(ctrlPanel, 'Style', 'popupmenu', ...
              'String', {'Degrees (deg)', 'Radians (rad)'}, ...
              'Units', 'normalized', 'Position', [0.15, 0.952, 0.25, 0.028], ...
              'Callback', @onSettingsChanged);

    uicontrol(ctrlPanel, 'Style', 'text', 'String', 'Convention:', ...
              'Units', 'normalized', 'Position', [0.42, 0.95, 0.18, 0.025], ...
              'HorizontalAlignment', 'left', 'BackgroundColor', [0.95 0.95 0.96], 'FontWeight', 'bold');
    S.ui.spacePopup = uicontrol(ctrlPanel, 'Style', 'popupmenu', ...
              'String', {'Teensy Firmware', 'ROS 2 URDF', 'IK9 Model (q)'}, ...
              'Units', 'normalized', 'Position', [0.60, 0.952, 0.37, 0.028], ...
              'Callback', @onSettingsChanged);

    S.ui.marginChk = uicontrol(ctrlPanel, 'Style', 'checkbox', ...
              'String', '2.0° Safety Margin', ...
              'Value', S.use_margin, 'BackgroundColor', [0.95 0.95 0.96], ...
              'Units', 'normalized', 'Position', [0.03, 0.915, 0.32, 0.026], ...
              'Callback', @onSettingsChanged);

    S.ui.triadChk = uicontrol(ctrlPanel, 'Style', 'checkbox', ...
              'String', 'Joint Axes', ...
              'Value', S.show_triads, 'BackgroundColor', [0.95 0.95 0.96], ...
              'Units', 'normalized', 'Position', [0.36, 0.915, 0.22, 0.026], ...
              'Callback', @onToggleVisuals);

    S.ui.meshChk = uicontrol(ctrlPanel, 'Style', 'checkbox', ...
              'String', 'Show STL Meshes', ...
              'Value', S.show_meshes, 'BackgroundColor', [0.95 0.95 0.96], ...
              'Units', 'normalized', 'Position', [0.58, 0.915, 0.25, 0.026], ...
              'Callback', @onToggleVisuals);

    S.ui.zoomChk = uicontrol(ctrlPanel, 'Style', 'checkbox', ...
              'String', 'Follow', ...
              'Value', S.auto_zoom, 'BackgroundColor', [0.95 0.95 0.96], ...
              'Units', 'normalized', 'Position', [0.84, 0.915, 0.14, 0.026], ...
              'Callback', @onToggleVisuals);

    % Camera & STL Loader Row
    uicontrol(ctrlPanel, 'Style', 'pushbutton', 'String', 'Side View (Sketch)', ...
              'Units', 'normalized', 'Position', [0.03, 0.875, 0.30, 0.032], ...
              'Callback', @(~,~) view(ax, -90, 0));
    uicontrol(ctrlPanel, 'Style', 'pushbutton', 'String', '3D Iso View', ...
              'Units', 'normalized', 'Position', [0.35, 0.875, 0.28, 0.032], ...
              'Callback', @(~,~) view(ax, -125, 22));
    uicontrol(ctrlPanel, 'Style', 'pushbutton', 'String', 'Load STL Folder...', ...
              'Units', 'normalized', 'Position', [0.65, 0.875, 0.32, 0.032], ...
              'Callback', @onSelectStlFolder);

    %% 4. Ground Vehicle Sliders (q1..q3)
    gvPanel = uipanel('Parent', ctrlPanel, 'Title', 'Ground Vehicle (3-DOF)', ...
                      'FontWeight', 'bold', 'BackgroundColor', [0.95 0.95 0.96], ...
                      'Position', [0.03, 0.70, 0.94, 0.165]);

    gvLabels = {'q1: X Pos (m)', 'q2: Y Pos (m)', 'q3: Yaw'};
    for i = 1:3
        yPos = 0.68 - (i-1)*0.31;
        S.ui.gvLbl(i) = uicontrol(gvPanel, 'Style', 'text', 'String', gvLabels{i}, ...
            'Units', 'normalized', 'Position', [0.02, yPos, 0.28, 0.25], ...
            'HorizontalAlignment', 'left', 'BackgroundColor', [0.95 0.95 0.96]);
        S.ui.gvSld(i) = uicontrol(gvPanel, 'Style', 'slider', ...
            'Units', 'normalized', 'Position', [0.31, yPos+0.02, 0.48, 0.22], ...
            'Callback', @(src,~) onSliderMoved(i, src));
        S.ui.gvEdt(i) = uicontrol(gvPanel, 'Style', 'edit', ...
            'Units', 'normalized', 'Position', [0.81, yPos, 0.17, 0.26], ...
            'Callback', @(src,~) onEditChanged(i, src));
    end

    %% 5. Robotic Arm Sliders (J1..J6)
    armPanel = uipanel('Parent', ctrlPanel, 'Title', 'Robotic Arm (6-DOF)', ...
                       'FontWeight', 'bold', 'BackgroundColor', [0.95 0.95 0.96], ...
                       'Position', [0.03, 0.33, 0.94, 0.36]);

    linkNames = {'Link 1 (Blue)', 'Link 2 (Teal)', 'Link 3 (Green)', ...
                 'Link 4 (Amber)', 'Link 5 (Coral)', 'Link 6 (Purple)'};
    for j = 1:6
        idx = j + 3;
        yPos = 0.83 - (j-1)*0.16;
        S.ui.armLbl(j) = uicontrol(armPanel, 'Style', 'text', ...
            'String', sprintf('J%d [%s]:', j, linkNames{j}), ...
            'Units', 'normalized', 'Position', [0.02, yPos, 0.26, 0.13], ...
            'HorizontalAlignment', 'left', 'BackgroundColor', [0.95 0.95 0.96], ...
            'ForegroundColor', S.colors(j+1,:) * 0.85, 'FontWeight', 'bold');
        S.ui.armSld(j) = uicontrol(armPanel, 'Style', 'slider', ...
            'Units', 'normalized', 'Position', [0.28, yPos+0.01, 0.38, 0.12], ...
            'Callback', @(src,~) onSliderMoved(idx, src));
        S.ui.armEdt(j) = uicontrol(armPanel, 'Style', 'edit', ...
            'Units', 'normalized', 'Position', [0.68, yPos, 0.14, 0.14], ...
            'Callback', @(src,~) onEditChanged(idx, src));
        S.ui.armLimLbl(j) = uicontrol(armPanel, 'Style', 'text', ...
            'String', '[min, max]', ...
            'Units', 'normalized', 'Position', [0.83, yPos, 0.16, 0.13], ...
            'HorizontalAlignment', 'right', 'FontSize', 8, ...
            'BackgroundColor', [0.95 0.95 0.96]);
    end

    %% 6. Preset Posture Buttons (Stowed Pose, Arm Ready Pose, Upright, Reset GV)
    uicontrol(ctrlPanel, 'Style', 'pushbutton', 'String', 'Stowed Pose', ...
              'Units', 'normalized', 'Position', [0.03, 0.275, 0.23, 0.045], ...
              'FontWeight', 'bold', ...
              'Callback', @(~,~) setPreset('stowed'));
    uicontrol(ctrlPanel, 'Style', 'pushbutton', 'String', 'Arm Ready Pose', ...
              'Units', 'normalized', 'Position', [0.27, 0.275, 0.23, 0.045], ...
              'FontWeight', 'bold', ...
              'Callback', @(~,~) setPreset('ready'));
    uicontrol(ctrlPanel, 'Style', 'pushbutton', 'String', 'URDF Zero (0..0)', ...
              'Units', 'normalized', 'Position', [0.51, 0.275, 0.23, 0.045], ...
              'Callback', @(~,~) setPreset('urdf_zero'));
    uicontrol(ctrlPanel, 'Style', 'pushbutton', 'String', 'Reset GV (0,0,0)', ...
              'Units', 'normalized', 'Position', [0.75, 0.275, 0.22, 0.045], ...
              'Callback', @(~,~) setPreset('reset_gv'));

    %% 7. Telemetry & Pose Readout Box
    posePanel = uipanel('Parent', ctrlPanel, 'Title', 'End-Effector Pose & Multi-Space Telemetry', ...
                        'FontWeight', 'bold', 'BackgroundColor', [0.95 0.95 0.96], ...
                        'Position', [0.03, 0.01, 0.94, 0.255]);
    S.ui.telemetryTxt = uicontrol(posePanel, 'Style', 'text', ...
                        'String', 'Initializing...', ...
                        'Units', 'normalized', 'Position', [0.02, 0.02, 0.96, 0.96], ...
                        'HorizontalAlignment', 'left', ...
                        'FontName', 'FixedWidth', 'FontSize', 8.5, ...
                        'BackgroundColor', [1 1 1]);

    %% 8. Initialize 3D Graphics Objects
    % Floor reference grid patch
    patch(ax, [-2.5 2.5 2.5 -2.5], [-2.5 -2.5 2.5 2.5], [0 0 0 0], ...
          [0.88 0.89 0.91], 'FaceAlpha', 0.25, 'EdgeColor', [0.7 0.7 0.7]);

    % Ground Vehicle Chassis (3D Box from z=r to z=0.20)
    S.gfx.gvBox = patch(ax, 'Vertices', zeros(8,3), ...
                        'Faces', [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8], ...
                        'FaceColor', [0.25 0.32 0.42], 'FaceAlpha', 0.55, ...
                        'EdgeColor', [0.1 0.15 0.2], 'LineWidth', 1.2);

    % Ground Vehicle Heading Arrows (+X_base and +Y_base)
    S.gfx.gvHeadingX = plot3(ax, [0 0], [0 0], [0 0], 'r-', 'LineWidth', 2.0);
    S.gfx.gvHeadingY = plot3(ax, [0 0], [0 0], [0 0], 'g-', 'LineWidth', 2.0);

    % 4 Omni/Mecanum Wheels
    for wIdx = 1:4
        S.gfx.wheelStrut(wIdx) = plot3(ax, [0 0], [0 0], [0 0], 'k-', 'LineWidth', 2.0);
        S.gfx.wheelRim(wIdx)   = patch(ax, 'XData', [0 0 0], 'YData', [0 0 0], 'ZData', [0 0 0], ...
                                       'FaceColor', [0.18 0.18 0.18], 'EdgeColor', 'k', 'LineWidth', 1.5);
    end

    % STL Mesh Patches (base_link + link_1..link_6)
    for mIdx = 1:7
        S.gfx.meshPatch(mIdx) = patch(ax, 'Faces', [], 'Vertices', [], ...
                                      'FaceColor', S.colors(mIdx, :), ...
                                      'EdgeColor', 'none', ...
                                      'FaceAlpha', 0.88, ...
                                      'AmbientStrength', 0.45, ...
                                      'DiffuseStrength', 0.75, ...
                                      'SpecularStrength', 0.25);
    end

    % Kinematic Skeleton Links (Standard Color Sequence)
    for cIdx = 1:7
        S.gfx.skelLink(cIdx) = plot3(ax, [0 0], [0 0], [0 0], '-', ...
                                     'Color', S.colors(cIdx, :), 'LineWidth', 6.5);
    end

    % Physical Joint Markers (J1..J6) & End-Effector Marker
    S.gfx.physJoints = plot3(ax, zeros(1,6), zeros(1,6), zeros(1,6), ...
                             'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.15 0.15 0.15], ...
                             'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
    S.gfx.eeMarker   = plot3(ax, 0, 0, 0, 'p', 'MarkerSize', 14, ...
                             'MarkerFaceColor', [0.1 0.85 0.25], 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);

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

    % Attempt to auto-discover STL files in current directory or ./meshes/ar4_mk3
    tryAutoLoadStlMeshes(pwd);

    % Sync UI controls and draw initial frame
    refreshControlBoundsAndValues();
    updateVisualization();

    %% ====================================================================
    %  STL Mesh Loading & Transformation Helpers
    %  ====================================================================
    function tryAutoLoadStlMeshes(baseDir)
        stlNames = {'base_link.STL', 'link_1.STL', 'link_2.STL', ...
                    'link_3.STL', 'link_4.STL', 'link_5.STL', 'link_6.STL'};
        searchDirs = {baseDir, ...
                      fullfile(baseDir, 'meshes', 'ar4_mk3'), ...
                      fullfile(baseDir, 'ar4_mk3')};

        foundAll = false;
        for dIdx = 1:numel(searchDirs)
            folder = searchDirs{dIdx};
            if ~isfolder(folder), continue; end
            ok = true;
            loaded = cell(1, 7);
            for m = 1:7
                fPath = fullfile(folder, stlNames{m});
                if ~isfile(fPath)
                    % Check lowercase extension fallback
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
                S.mesh_data = loaded;
                S.meshes_loaded = true;
                foundAll = true;
                break;
            end
        end

        if foundAll
            for m = 1:7
                set(S.gfx.meshPatch(m), 'Faces', S.mesh_data{m}.Faces, ...
                                        'Vertices', S.mesh_data{m}.Vertices);
            end
        end
    end

    function onSelectStlFolder(~, ~)
        folder = uigetdir(pwd, 'Select Folder Containing AR4 MK3 STL Meshes (base_link.STL .. link_6.STL)');
        if isequal(folder, 0), return; end
        tryAutoLoadStlMeshes(folder);
        if S.meshes_loaded
            updateVisualization();
        else
            errordlg('Could not find base_link.STL and link_1.STL .. link_6.STL in the selected folder.', 'STL Load Error');
        end
    end

    %% ====================================================================
    %  Angle Space Conversions (Teensy Firmware <-> URDF <-> IK9 Model)
    %  ====================================================================
    function [minFW, maxFW] = getFirmwareLimitsDeg()
        margin = S.use_margin * S.fw_margin;
        minFW = S.fw_min + margin;
        maxFW = S.fw_max - margin;
    end

    function dispVal = fwDegToDisplay(jIdx, fw_d)
        switch S.angle_space
            case 'firmware'
                degVal = fw_d;
            case 'urdf'
                degVal = (fw_d - S.urdf_offset(jIdx)) / S.urdf_sign(jIdx);
            case 'model'
                degVal = (fw_d - S.model_offset(jIdx)) / S.model_sign(jIdx);
        end
        if strcmp(S.unit_mode, 'rad')
            dispVal = deg2rad(degVal);
        else
            dispVal = degVal;
        end
    end

    function fw_d = displayToFwDeg(jIdx, dispVal)
        if strcmp(S.unit_mode, 'rad')
            degVal = rad2deg(dispVal);
        else
            degVal = dispVal;
        end
        switch S.angle_space
            case 'firmware'
                fw_d = degVal;
            case 'urdf'
                fw_d = S.urdf_sign(jIdx) * degVal + S.urdf_offset(jIdx);
            case 'model'
                fw_d = S.model_sign(jIdx) * degVal + S.model_offset(jIdx);
        end
    end

    function refreshControlBoundsAndValues()
        isRad = strcmp(S.unit_mode, 'rad');
        unitStr = 'deg';
        if isRad, unitStr = 'rad'; end

        % 1. Ground Vehicle Sliders
        set(S.ui.gvSld(1), 'Min', -2.0, 'Max', 2.0, 'Value', max(-2, min(2, S.gv_pose(1))));
        set(S.ui.gvEdt(1), 'String', sprintf('%.3f', S.gv_pose(1)));

        set(S.ui.gvSld(2), 'Min', -2.0, 'Max', 2.0, 'Value', max(-2, min(2, S.gv_pose(2))));
        set(S.ui.gvEdt(2), 'String', sprintf('%.3f', S.gv_pose(2)));

        yawLim = 180;
        yawVal = rad2deg(S.gv_pose(3));
        if isRad
            yawLim = pi;
            yawVal = S.gv_pose(3);
        end
        set(S.ui.gvLbl(3), 'String', sprintf('q3: Yaw (%s)', unitStr));
        set(S.ui.gvSld(3), 'Min', -yawLim, 'Max', yawLim, 'Value', max(-yawLim, min(yawLim, yawVal)));
        set(S.ui.gvEdt(3), 'String', sprintf('%.2f', yawVal));

        % 2. Robotic Arm Sliders
        [minFW, maxFW] = getFirmwareLimitsDeg();
        for j = 1:6
            S.fw_deg(j) = max(minFW(j), min(maxFW(j), S.fw_deg(j)));

            b1 = fwDegToDisplay(j, minFW(j));
            b2 = fwDegToDisplay(j, maxFW(j));
            lowLim  = min(b1, b2);
            highLim = max(b1, b2);
            dispVal = max(lowLim, min(highLim, fwDegToDisplay(j, S.fw_deg(j))));

            if isRad
                fmtStr = '%.3f';
                limStr = sprintf('[%.2f,%.2f]', lowLim, highLim);
            else
                fmtStr = '%.1f';
                limStr = sprintf('[%.0f,%.0f]', lowLim, highLim);
            end

            set(S.ui.armSld(j), 'Min', lowLim, 'Max', highLim, 'Value', dispVal);
            set(S.ui.armEdt(j), 'String', sprintf(fmtStr, dispVal));
            set(S.ui.armLimLbl(j), 'String', limStr);
        end
    end

    function onSettingsChanged(~, ~)
        if get(S.ui.unitPopup, 'Value') == 1
            S.unit_mode = 'deg';
        else
            S.unit_mode = 'rad';
        end

        spaceIdx = get(S.ui.spacePopup, 'Value');
        if spaceIdx == 1
            S.angle_space = 'firmware';
        elseif spaceIdx == 2
            S.angle_space = 'urdf';
        else
            S.angle_space = 'model';
        end

        S.use_margin = logical(get(S.ui.marginChk, 'Value'));
        refreshControlBoundsAndValues();
        updateVisualization();
    end

    function onToggleVisuals(~, ~)
        S.show_triads = logical(get(S.ui.triadChk, 'Value'));
        S.show_meshes = logical(get(S.ui.meshChk, 'Value'));
        S.auto_zoom   = logical(get(S.ui.zoomChk, 'Value'));
        updateVisualization();
    end

    function onSliderMoved(qIdx, src)
        applyInputChange(qIdx, get(src, 'Value'));
    end

    function onEditChanged(qIdx, src)
        val = str2double(get(src, 'String'));
        if isnan(val)
            refreshControlBoundsAndValues();
            return;
        end
        applyInputChange(qIdx, val);
    end

    function applyInputChange(qIdx, val)
        isRad = strcmp(S.unit_mode, 'rad');
        if qIdx <= 2
            S.gv_pose(qIdx) = max(-2.0, min(2.0, val));
        elseif qIdx == 3
            if isRad
                S.gv_pose(3) = max(-pi, min(pi, val));
            else
                S.gv_pose(3) = deg2rad(max(-180, min(180, val)));
            end
        else
            j = qIdx - 3;
            S.fw_deg(j) = displayToFwDeg(j, val);
        end
        refreshControlBoundsAndValues();
        updateVisualization();
    end

    function setPreset(presetName)
        switch presetName
            case 'reset_gv'
                S.gv_pose = [0; 0; 0];
            case 'stowed'
                % Stowed Pose: Upper arm tilted back (FW J2=15 deg / URDF -27 deg),
                % Forearm folded forward across upper arm (FW J3=138 deg / URDF -86 deg),
                % Wrist pitched up (FW J5=35 deg)
                S.fw_deg = [0; 15; 138; 0; 35; 0];
            case 'ready'
                % Ready Pose: Upper arm pitched forward, elbow open, wrist level
                S.fw_deg = [0; 72; 65; 0; -20; 0];
            case 'urdf_zero'
                % All URDF joint angles = 0 deg (J2 vertical up, J3 bent 90 deg forward)
                S.fw_deg = S.urdf_offset';
        end
        refreshControlBoundsAndValues();
        updateVisualization();
    end

    %% ====================================================================
    %  Exact ROS 2 URDF Forward Kinematics & Rendering
    %  ====================================================================
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

    function updateVisualization()
        x_gv = S.gv_pose(1);
        y_gv = S.gv_pose(2);
        th   = S.gv_pose(3);

        Rz_base = [cos(th), -sin(th), 0;
                   sin(th),  cos(th), 0;
                   0,        0,       1];

        % Frame 1: arm_base_link mounted at height S.base_z0 on Ground Vehicle
        T_links = zeros(4, 4, 7);
        T_links(:, :, 1) = [Rz_base, [x_gv; y_gv; S.base_z0]; 0, 0, 0, 1];

        % Convert Firmware degrees to URDF radians for the URDF kinematic chain
        urdf_deg = (S.fw_deg' - S.urdf_offset) ./ S.urdf_sign;
        urdf_rad = deg2rad(urdf_deg);

        jointWorldAxes = zeros(3, 6);
        for j = 1:6
            T_origin = urdfOriginTransform(S.urdf_origins(j, :));
            T_pre    = T_links(:, :, j) * T_origin;
            jointWorldAxes(:, j) = T_pre(1:3, 1:3) * S.urdf_axes(j, :)';
            T_rot    = axisAngleTransform(S.urdf_axes(j, :), urdf_rad(j));
            T_links(:, :, j+1) = T_pre * T_rot;
        end

        % End-Effector Pose (arm_ee_link is fixed at [0,0,0] on arm_link_6)
        T_ee = T_links(:, :, 7);
        pe   = T_ee(1:3, 4);
        Re   = T_ee(1:3, 1:3);

        % --- 1. Update STL Meshes (if loaded and enabled) ---
        for m = 1:7
            if S.meshes_loaded && S.show_meshes
                Tm = T_links(:, :, m);
                V_local = S.mesh_data{m}.Vertices;
                V_world = (Tm(1:3, 1:3) * V_local')' + Tm(1:3, 4)';
                set(S.gfx.meshPatch(m), 'Vertices', V_world, 'Visible', 'on');
            else
                set(S.gfx.meshPatch(m), 'Visible', 'off');
            end
        end

        % --- 2. Update Standard-Colored Kinematic Skeleton ---
        % Extract key physical points from the URDF link frames:
        P_base = T_links(1:3, 4, 1); % arm_base_link
        P_J1   = T_links(1:3, 4, 2); % arm_link_1 origin
        % Shoulder L-corner in arm_link_1 frame: [0; 0.0642; 0]
        P_L1_corner = T_links(1:3, 4, 2) + T_links(1:3, 1:3, 2) * [0; 0.0642; 0];
        P_J2   = T_links(1:3, 4, 3); % arm_link_2 origin
        P_J3   = T_links(1:3, 4, 4); % arm_link_3 origin
        % J4 roll split directly inline along forearm (-Z of arm_link_4 from P_J3)
        P_J4   = P_J3 + T_links(1:3, 1:3, 5) * [0; 0; -0.085];
        P_J5   = T_links(1:3, 4, 6); % arm_link_5 origin
        P_J6   = T_links(1:3, 4, 7); % arm_link_6 / arm_ee_link origin

        % Segment 1 (Slate Gray): Base pedestal to J1
        set(S.gfx.skelLink(1), 'XData', [P_base(1) P_L1_corner(1)], ...
                               'YData', [P_base(2) P_L1_corner(2)], ...
                               'ZData', [P_base(3) P_L1_corner(3)]);
        % Segment 2 (Royal Blue): Shoulder vertical column to J2
        set(S.gfx.skelLink(2), 'XData', [P_L1_corner(1) P_J2(1)], ...
                               'YData', [P_L1_corner(2) P_J2(2)], ...
                               'ZData', [P_L1_corner(3) P_J2(3)]);
        % Segment 3 (Teal): Upper arm J2 to J3
        set(S.gfx.skelLink(3), 'XData', [P_J2(1) P_J3(1)], ...
                               'YData', [P_J2(2) P_J3(2)], ...
                               'ZData', [P_J2(3) P_J3(3)]);
        % Segment 4 (Emerald Green): Elbow housing directly inline from J3 to J4
        set(S.gfx.skelLink(4), 'XData', [P_J3(1) P_J4(1)], ...
                               'YData', [P_J3(2) P_J4(2)], ...
                               'ZData', [P_J3(3) P_J4(3)]);
        % Segment 5 (Amber): Forearm tube J4 to J5 wrist center
        set(S.gfx.skelLink(5), 'XData', [P_J4(1) P_J5(1)], ...
                               'YData', [P_J4(2) P_J5(2)], ...
                               'ZData', [P_J4(3) P_J5(3)]);
        % Segment 6 (Coral Orange): Wrist J5 to J6 flange
        set(S.gfx.skelLink(6), 'XData', [P_J5(1) P_J6(1)], ...
                               'YData', [P_J5(2) P_J6(2)], ...
                               'ZData', [P_J5(3) P_J6(3)]);
        % Segment 7 (Purple): Tool flange indicator
        pTip = P_J6 + Re(:,3)*0.015;
        set(S.gfx.skelLink(7), 'XData', [P_J6(1) pTip(1)], ...
                               'YData', [P_J6(2) pTip(2)], ...
                               'ZData', [P_J6(3) pTip(3)]);

        % Update Joint Markers & Labels
        jPts = [P_J1, P_J2, P_J3, P_J4, P_J5, P_J6];
        set(S.gfx.physJoints, 'XData', jPts(1,:), 'YData', jPts(2,:), 'ZData', jPts(3,:));
        set(S.gfx.eeMarker,   'XData', pe(1), 'YData', pe(2), 'ZData', pe(3));

        axLen = 0.055;
        for jIdx = 1:6
            pJ = jPts(:, jIdx);
            uA = jointWorldAxes(:, jIdx);
            if S.show_triads
                pA1 = pJ - uA * axLen;
                pA2 = pJ + uA * axLen;
                set(S.gfx.axisLine(jIdx), 'XData', [pA1(1) pA2(1)], ...
                                          'YData', [pA1(2) pA2(2)], ...
                                          'ZData', [pA1(3) pA2(3)], 'Visible', 'on');
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

        % --- 3. Update Ground Vehicle Geometry ---
        hx = S.gv.d / 2;
        hy = S.gv.w / 2;
        zBot = S.gv.r;
        zTop = S.base_z0;
        localCorners = [-hx, -hy, zBot;
                         hx, -hy, zBot;
                         hx,  hy, zBot;
                        -hx,  hy, zBot;
                        -hx, -hy, zTop;
                         hx, -hy, zTop;
                         hx,  hy, zTop;
                        -hx,  hy, zTop];
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
            set(S.gfx.wheelStrut(wIdx), ...
                'XData', [cHub(1) wWorldCenter(1)], ...
                'YData', [cHub(2) wWorldCenter(2)], ...
                'ZData', [cHub(3) wWorldCenter(3)]);

            tangentDir = Rz_base * [cos(ang); -sin(ang); 0];
            circlePts = wWorldCenter + S.gv.r * (tangentDir * cos(thDisk) + [0;0;1] * sin(thDisk));
            set(S.gfx.wheelRim(wIdx), ...
                'XData', circlePts(1,:), 'YData', circlePts(2,:), 'ZData', circlePts(3,:));
        end

        % --- 4. Viewport Bounds ---
        if S.auto_zoom
            span = 0.58;
            xlim(ax, [x_gv - span, x_gv + span]);
            ylim(ax, [y_gv - span, y_gv + span]);
            zlim(ax, [0, 0.90]);
        else
            xlim(ax, [-2.5, 2.5]);
            ylim(ax, [-2.5, 2.5]);
            zlim(ax, [0, 1.2]);
        end

        % --- 5. Update Telemetry Box ---
        fw_d    = S.fw_deg';
        model_d = (fw_d - S.model_offset) ./ S.model_sign;
        meshStatus = 'Skeleton Only (Click "Load STL Folder..." to load CAD)';
        if S.meshes_loaded
            meshStatus = '3D STL Meshes Loaded';
        end

        teleStr = sprintf([ ...
            'STL Status:   %s\n', ...
            'pe (World m): [ %7.4f, %7.4f, %7.4f ]\n', ...
            'Re (World):   [ %6.3f  %6.3f  %6.3f ]\n', ...
            '              [ %6.3f  %6.3f  %6.3f ]\n', ...
            '              [ %6.3f  %6.3f  %6.3f ]\n', ...
            '--------------------------------------------------\n', ...
            'Firmware (°): [%6.1f %6.1f %6.1f %6.1f %6.1f %6.1f]\n', ...
            'ROS2 URDF(°): [%6.1f %6.1f %6.1f %6.1f %6.1f %6.1f]\n', ...
            'Model q4:9° : [%6.1f %6.1f %6.1f %6.1f %6.1f %6.1f]'], ...
            meshStatus, ...
            pe(1), pe(2), pe(3), ...
            Re(1,1), Re(1,2), Re(1,3), ...
            Re(2,1), Re(2,2), Re(2,3), ...
            Re(3,1), Re(3,2), Re(3,3), ...
            fw_d(1), fw_d(2), fw_d(3), fw_d(4), fw_d(5), fw_d(6), ...
            urdf_deg(1), urdf_deg(2), urdf_deg(3), urdf_deg(4), urdf_deg(5), urdf_deg(6), ...
            model_d(1), model_d(2), model_d(3), model_d(4), model_d(5), model_d(6));
        set(S.ui.telemetryTxt, 'String', teleStr);
    end
end