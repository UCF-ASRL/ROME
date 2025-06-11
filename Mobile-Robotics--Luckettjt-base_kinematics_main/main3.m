%% main 3 
close all;
clear all;
clc;

%% Constants
alpha = [deg2rad(315), deg2rad(45), deg2rad(135), deg2rad(225)];
r = .076;
d = .45;

%% State Coefficient matrix

A = [0 0 0;
     0 0 0;    
     0 0 0];   % 3x3


%% Control Coefficient Matrix

[H,B] = wheelDrivingSpeeds();

%% Output Coefficient Matrix 

C = eye(3);  %% 3x3
 
%% Cost function Coefficient Matrices 

R = [.7 0 0 0;
     0 .7 0 0;
     0 0 .7 0;
     0 0 0 .7]; %% 4x4

Q = [1 0 0;
     0 1 0;
     0 0 1]; %% 3x3

[P,K,L] = icare(A,B,Q,R,[],[],[]); % P = 3x3; K = 4x3; L =3x1;

%% Initial state
x0 = [0; 0; 0]; % Initial state [x; y; theta]

%% Time settings
dt = .0500;  % Time step
Tmax = 30; % Total simulation duration

%% Storage for data
time_data = 0:dt:Tmax;
desired_state_history = zeros(3, length(time_data));
control_history = zeros(4, length(time_data));
state_history = zeros(3,length(time_data));

%% Define the filename for the GIF
filename = 'robot_animation.gif';

%% Loop
for k = 1:(length(time_data)-1)
    tspan = [time_data(k), time_data(k+1)];

    % Get desired trajectory for this time step
    Ydesired = desiredTrajectory(time_data(k));

    % Calculate the control input using Controller
    u = Controller(x0, Ydesired, A, B, C, P, Q, R);

    % Bound each element of u:
    ub = .48; %m/s
    lb = -.48;

    u = max(u, lb);   % force all entries to be >= lb
    u = min(u, ub);   % force all entries to be <= ub

      % u= [1,1,1,1]';
    % Solve the differential equation for this time step using the control input u
    [~, x_out] = ode45(@(t, x)Kinematics(t, x, A, B, u), tspan, x0);

    % Store the result
    x_next = x_out(end, :)';
    state_history(:, k+1) = x_next;

    Ydesired_next = desiredTrajectory(time_data(k+1));
    desired_state_history(:, k+1) = Ydesired_next;

    control_history(:, k+1) = u;
    
    % Set the new initial condition for the next loop iteration
    x0 = x_next;

    % Visualization
    visualizeRobot(x_next(1), x_next(2), x_next(3), r, u, state_history); 
    drawnow;

    % Capture the current figure as a frame
    frame = getframe(gcf);
    im = frame2im(frame);

    % Convert RGB image to indexed image
    [imind,cm] = rgb2ind(im,256);

    % Write frames to GIF
    if k == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf, 'DelayTime',dt);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append', 'DelayTime',dt);
    end

    % pause(dt/100000000);
end

%% Plots

figure;

subplot(3,1,1);
plot(time_data, state_history(1, :));
xlabel('Time (s)');
ylabel('X Position (m)');
title('Robot Trajectory');

subplot(3,1,2);
plot(time_data, state_history(2, :));
xlabel('Time (s)');
ylabel('Y Position (m)');

subplot(3,1,3);
plot(time_data, state_history(3, :));
xlabel('Time (s)');
ylabel('\Theta Orientation (rad)');

figure;
plot(state_history(1, :), state_history(2, :));
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robot Path');

u_min = -.48;
u_max = .48;

figure;
plot(time_data, control_history(1, :),'--');
hold on;
% Draw horizontal lines for bounds
yline(u_min, 'r--', 'LineWidth', 1.2, 'DisplayName','Lower Bound');
yline(u_max, 'r--', 'LineWidth', 1.2, 'DisplayName','Upper Bound');
plot(time_data, control_history(2, :));
plot(time_data, control_history(3, :),'--');
plot(time_data, control_history(4, :));
xlabel('Time (s)');
ylabel('wheel speeds ');
title('Robot wheel speeds (m/s)');
legend('Vw1', 'Vw2','Vw3','Vw4');

control_history = control_history/r;

figure;
plot(time_data, control_history(1, :),'--');
hold on;
plot(time_data, control_history(2, :));
plot(time_data, control_history(3, :),'--');
plot(time_data, control_history(4, :));
xlabel('Time (s)');
ylabel('wheel speeds ');
title('Robot wheel speeds (rad/s)');
legend('Vw1', 'Vw2','Vw3','Vw4');

% Convert rad/s to RPM
control_history_rpm = (control_history * 60) / (2 * pi);

% Plotting in RPM
figure;
plot(time_data, control_history_rpm(1, :), '--');
hold on;
plot(time_data, control_history_rpm(2, :));
plot(time_data, control_history_rpm(3, :), '--');
plot(time_data, control_history_rpm(4, :));
xlabel('Time (s)');
ylabel('Wheel speeds (RPM)');
title('Robot wheel speeds (RPM)');
legend('RPMw1', 'RPMw2', 'RPMw3', 'RPMw4');

figure;
plot(state_history(1, :), state_history(2, :), 'b-', 'LineWidth', 1.5);
hold on;
plot(desired_state_history(1, :), desired_state_history(2, :), 'r--', 'LineWidth', 1.5);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robot Path: Computed vs. Desired');
legend('Computed Trajectory', 'Desired Trajectory');
grid on;

%% to Generate CSV for Arduino
% Save the wheel speeds to a CSV file
M = [control_history', time_data'];  % Combine the transposed wheel speeds with time data

% Write this matrix to a CSV file
csvwrite('wheelSpeeds.csv', M);

% Read the matrix back from the CSV to ensure everything is correct
M = csvread('wheelSpeeds.csv');

% Open a text file to write the arrays for Arduino
fid = fopen('arduinoArrays.txt', 'wt');

% Write each control array to the text file
for j = 1:4  % Loop over each wheel speed
    fprintf(fid, 'float u%d_values[] = {', j);  % Start the array definition
    fprintf(fid, '%d', M(1,j));  % First element without a comma
    for i = 2:size(M,1)  % For each time step
        fprintf(fid, ', %d', M(i,j));  % Append each subsequent element with a comma
    end
    fprintf(fid, '};\n');  % Close the array definition
end

% Now handle the time values in the same way
fprintf(fid, 'float timeValues[] = {%d', M(1,end));  % Start the array for time data
for i = 2:size(M,1)
    fprintf(fid, ', %d', M(i,end));
end
fprintf(fid, '};\n');  % Close the array definition

% Close the text file
fclose(fid);
fclose(fid);
