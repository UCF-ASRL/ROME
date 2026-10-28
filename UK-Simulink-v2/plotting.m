close all

time =  out.q_sim.Time;
q_sim = squeeze(out.q_sim.Data);
x_pos_sim = q_sim(1, :);
y_pos_sim = q_sim(2, :);
th_sim = unwrap(q_sim(3, :));

q_des = squeeze(out.q_des.Data);
x_pos_des = q_des(1, :);
y_pos_des = q_des(2, :);
th_des = (q_des(3, :));

q_track = squeeze(out.q_track.Data);
x_pos_track = q_track(1, :);
y_pos_track = q_track(2, :);
th_track = (q_track(3, :));

%% Position integrated from UK acceleration
if ~EnableMotive % Plot only if Motive is not enabled
    figure
    hold on;

    % Plot a shaded circle for the central body
    theta_circle = linspace(0, 2*pi, 100);
    r_planet = 0.05; % 5cm radius planet
    fill(r_planet*cos(theta_circle), r_planet*sin(theta_circle), [0.3 0.6 1], 'EdgeColor', 'b', 'HandleVisibility', 'off'); 

    plot(x_pos_sim, y_pos_sim, 'b', 'LineWidth', 1.5, 'DisplayName', 'Position')

    % --- Add the Initial Position Dot ---
    % Plots the first data point as a large red filled circle
    plot(x_pos_sim(1), y_pos_sim(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
    text(x_pos_sim(1) + 0.05, y_pos_sim(1), 'q_0', 'FontWeight', 'bold'); 

    xlabel('x position'); ylabel('y position'); title('Integrated Position');
    axis equal

    % Cleanup and Legend
    legend('Location', 'northeast');
    xlabel('x position (m)'); ylabel('y position (m)'); 
    title('Integrated Position');
    grid on;
    axis equal
    
    % Add 10 percent padding to plot
    xlim_current = xlim;
    ylim_current = ylim;
    x_buffer = (xlim_current(2) - xlim_current(1)) * 0.1;
    y_buffer = (ylim_current(2) - ylim_current(1)) * 0.1;
    xlim([xlim_current(1) - x_buffer, xlim_current(2) + x_buffer]);
    ylim([ylim_current(1) - y_buffer, ylim_current(2) + y_buffer]);
end

%% Desired vs tracked position
if EnableMotive % Plot only if Motive is enabled
    figure
    hold on;
    
    % Plot a shaded circle for the central body
    theta_circle = linspace(0, 2*pi, 100);
    r_planet = 0.05; % 5cm radius planet
    fill(r_planet*cos(theta_circle), r_planet*sin(theta_circle), [0.3 0.6 1], 'EdgeColor', 'b', 'HandleVisibility', 'off'); 
    text(0 + 0.1, 0, 'Focus', 'FontWeight', 'bold');

    % Plot the trajectory and starting point
    plot(x_pos_des, y_pos_des, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Desired')
    plot(x_pos_track, y_pos_track, 'b', 'LineWidth', 1.5, 'DisplayName', 'Tracked')
    
    plot(x_pos_des(1), y_pos_des(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'q_0', 'HandleVisibility', 'off');
    text(x_pos_des(1) + 0.05, y_pos_des(1), 'q_0', 'FontWeight', 'bold');

    % Cleanup and Legend
    legend('Location', 'northeast');
    xlabel('x position (m)'); ylabel('y position (m)'); 
    title('Desired vs. Tracked Position');
    grid on;
    axis equal
    
    % Add 10 percent padding to plot
    xlim_current = xlim;
    ylim_current = ylim;
    x_buffer = (xlim_current(2) - xlim_current(1)) * 0.1;
    y_buffer = (ylim_current(2) - ylim_current(1)) * 0.1;
    xlim([xlim_current(1) - x_buffer, xlim_current(2) + x_buffer]);
    ylim([ylim_current(1) - y_buffer, ylim_current(2) + y_buffer]);
end

%% Plot theta
%% Heading Plots
if ~EnableMotive
    figure; plot(time, rad2deg(th_sim), 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('\theta (deg)'); title('Integrated Heading');
else
    figure; plot(time, rad2deg(th_des), 'k--', time, rad2deg(th_track), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('\theta (deg)'); title('Desired vs. Tracked Heading');
    legend('Desired', 'Tracked', 'Location', 'best');
end

%% Extract Velocity Data
q_dot_des = squeeze(out.q_dot_des.Data);
q_dot_track = squeeze(out.q_dot_track.Data);

v_x_des = q_dot_des(1, :);
v_y_des = q_dot_des(2, :);
v_th_des = q_dot_des(3, :);

v_x_track = q_dot_track(1, :);
v_y_track = q_dot_track(2, :);
v_th_track = q_dot_track(3, :);

% Calculate Errors
err_x = x_pos_des - x_pos_track;
err_y = y_pos_des - y_pos_track;
err_th = rad2deg(th_des - th_track); 
err_vx = v_x_des - v_x_track;
err_vy = v_y_des - v_y_track;
err_vth = rad2deg(v_th_des - v_th_track);

%% Color Definitions (Vibrant)
colorX = [0 0.5 1.0];      % Bright Azure Blue
colorY = [1.0 0 0.2];      % Vibrant Red
colorTh = [0.6 0.2 0.8];   % Brighter Purple

%% --- STATES (Position & Orientation) ---

% X Position (Keep 1.5 LineWidth)
figure('Name', 'X Position Comparison');
plot(time, x_pos_track, 'Color', colorX, 'LineWidth', 1.5); hold on;
plot(time, x_pos_des, 'k--', 'LineWidth', 1.2); 
xlabel('Time (s)'); ylabel('$x$ (m)', 'Interpreter', 'latex');
title('\textbf{x Position: Desired vs. Tracked}', 'Interpreter', 'latex');
legend({'Tracked', 'Desired'}, 'Location', 'northeast'); grid on;

% Y Position (Keep 1.5 LineWidth)
figure('Name', 'Y Position Comparison');
plot(time, y_pos_track, 'Color', colorY, 'LineWidth', 1.5); hold on;
plot(time, y_pos_des, 'k--', 'LineWidth', 1.2); 
xlabel('Time (s)'); ylabel('$y$ (m)', 'Interpreter', 'latex');
title('\textbf{y Position: Desired vs. Tracked}', 'Interpreter', 'latex');
legend({'Tracked', 'Desired'}, 'Location', 'northeast'); grid on;

% Theta Position (Thinner Line + Extra Range)
figure('Name', 'Theta Comparison');
plot(time, rad2deg(th_track), 'Color', colorTh, 'LineWidth', 1.0); hold on;
plot(time, rad2deg(th_des), 'k--', 'LineWidth', 1.0); 
xlabel('Time (s)'); ylabel('$\theta$ (deg)', 'Interpreter', 'latex');
title('\textbf{$\theta$: Desired vs. Tracked}', 'Interpreter', 'latex');
legend({'Tracked', 'Desired'}, 'Location', 'northeast'); grid on;
yl = ylim; pad = diff(yl)*0.2; ylim([yl(1)-pad, yl(2)+pad]);

%% --- VELOCITIES ---

% X Velocity (Thinner Line)
figure('Name', 'X Velocity Comparison');
plot(time, v_x_track, 'Color', colorX, 'LineWidth', 1.0); hold on;
plot(time, v_x_des, 'k--', 'LineWidth', 1.0); 
xlabel('Time (s)'); ylabel('$\dot{x}$ (m/s)', 'Interpreter', 'latex');
title('\textbf{$\dot{x}$: Desired vs. Tracked}', 'Interpreter', 'latex');
legend({'Tracked', 'Desired'}, 'Location', 'northeast'); grid on;

% Y Velocity (Thinner Line)
figure('Name', 'Y Velocity Comparison');
plot(time, v_y_track, 'Color', colorY, 'LineWidth', 1.0); hold on;
plot(time, v_y_des, 'k--', 'LineWidth', 1.0); 
xlabel('Time (s)'); ylabel('$\dot{y}$ (m/s)', 'Interpreter', 'latex');
title('\textbf{$\dot{y}$: Desired vs. Tracked}', 'Interpreter', 'latex');
legend({'Tracked', 'Desired'}, 'Location', 'northeast'); grid on;

% Theta Velocity (Thinner Line + Extra Range)
figure('Name', 'Theta Velocity Comparison');
plot(time, rad2deg(v_th_track), 'Color', colorTh, 'LineWidth', 1.0); hold on;
plot(time, rad2deg(v_th_des), 'k--', 'LineWidth', 1.0); 
xlabel('Time (s)'); ylabel('$\dot{\theta}$ (deg/s)', 'Interpreter', 'latex');
title('\textbf{$\dot{\theta}$: Desired vs. Tracked}', 'Interpreter', 'latex');
legend({'Tracked', 'Desired'}, 'Location', 'northeast'); grid on;
yl = ylim; pad = diff(yl)*0.2; ylim([yl(1)-pad, yl(2)+pad]);

%% --- ERRORS (Thinner Lines + Extra Range) ---

% X Position Error
figure('Name', 'X Position Error');
plot(time, err_x, 'Color', colorX, 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('$x$ Error (m)', 'Interpreter', 'latex');
title('\textbf{x Position Error}', 'Interpreter', 'latex'); grid on;
yl = ylim; pad = diff(yl)*0.2; ylim([yl(1)-pad, yl(2)+pad]);

% Y Position Error
figure('Name', 'Y Position Error');
plot(time, err_y, 'Color', colorY, 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('$y$ Error (m)', 'Interpreter', 'latex');
title('\textbf{y Position Error}', 'Interpreter', 'latex'); grid on;
yl = ylim; pad = diff(yl)*0.2; ylim([yl(1)-pad, yl(2)+pad]);

% Theta Position Error
figure('Name', 'Theta Position Error');
plot(time, err_th, 'Color', colorTh, 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('$\theta$ Error (deg)', 'Interpreter', 'latex');
title('\textbf{$\theta$ Position Error}', 'Interpreter', 'latex'); grid on;
yl = ylim; pad = diff(yl)*0.2; ylim([yl(1)-pad, yl(2)+pad]);

% X Velocity Error
figure('Name', 'X Velocity Error');
plot(time, err_vx, 'Color', colorX, 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('$\dot{x}$ Error (m/s)', 'Interpreter', 'latex');
title('\textbf{$\dot{x}$ Velocity Error}', 'Interpreter', 'latex'); grid on;
yl = ylim; pad = diff(yl)*0.2; ylim([yl(1)-pad, yl(2)+pad]);

% Y Velocity Error
figure('Name', 'Y Velocity Error');
plot(time, err_vy, 'Color', colorY, 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('$\dot{y}$ Error (m/s)', 'Interpreter', 'latex');
title('\textbf{$\dot{y}$ Velocity Error}', 'Interpreter', 'latex'); grid on;
yl = ylim; pad = diff(yl)*0.2; ylim([yl(1)-pad, yl(2)+pad]);

% Theta Velocity Error
figure('Name', 'Theta Velocity Error');
plot(time, err_vth, 'Color', colorTh, 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('$\dot{\theta}$ Error (deg/s)', 'Interpreter', 'latex');
title('\textbf{$\dot{\theta}$ Velocity Error}', 'Interpreter', 'latex'); grid on;
yl = ylim; pad = diff(yl)*0.2; ylim([yl(1)-pad, yl(2)+pad]);

%% --- Create Performance Metrics Figure ---

% Prepare the Data
rmse_val = [sqrt(mean(err_x.^2)); sqrt(mean(err_y.^2)); sqrt(mean(err_th.^2)); ...
            sqrt(mean(err_vx.^2)); sqrt(mean(err_vy.^2)); sqrt(mean(err_vth.^2))];

mae_val  = [max(abs(err_x)); max(abs(err_y)); max(abs(err_th)); ...
            max(abs(err_vx)); max(abs(err_vy)); max(abs(err_vth))];

State_Case = {'x Position'; 'y Position'; 'theta Position'; ...
              'x Velocity'; 'y Velocity'; 'theta Velocity'};
Units      = {'m'; 'm'; 'deg'; 'm/s'; 'm/s'; 'deg/s'};

% Combine into a cell array for the UI Table
tableData = [State_Case, num2cell(rmse_val), num2cell(mae_val), Units];
columnNames = {'State Case', 'Root Mean Square Error (RMSE) ', 'Max Absolute  Error (MAE)', 'Units'};

% Create Figure and Table
fig = figure('Name', 'ROME Robot Performance Metrics', ...
             'NumberTitle', 'off', ...
             'MenuBar', 'none', ...
             'Position', [200, 200, 550, 220]); % Adjust size as needed

uit = uitable(fig, 'Data', tableData, ...
             'ColumnName', columnNames, ...
             'RowName', [], ...
             'Units', 'normalized', ...
             'Position', [0.05 0.05 0.9 0.9]);

% Auto-adjust column widths for a "proper" look
uit.ColumnWidth = {130, 100, 130, 80};