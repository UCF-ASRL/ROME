%% main
clc;
clear all;
close all;

%% constants
m = 18; % mass kg
r = .05; % wheel radius meters
l = .35; % distance from robot C.G to wheels meters
d = .45;% length of robot side(s) meters
w = .45;% width of robot side(s) meters 
Iyy = (1/12)*m*(d^2+w^2); % base moment of interia


%% Initial conditions 
q0 = [0; -2; 0];
q_dot0 = [1; 0; 0];

%% Time settings
dt = .1;  % Time step
Tmax = 11; % Total simulation duration

%% Storage for data
time_data = 0:dt:Tmax;

%% Define the filename for the GIF
filename = 'robot_animation.gif';

%% Initialize parameters
N = length(time_data);
q_history = zeros(3, N); % q = [x; y; theta]
q_dot_history = zeros(3, N);
q_ddot_history = zeros(3,N);
u_history = zeros(4, N); % 4-wheeled robot
q = q0; % initial pose [x; y; theta]
q_dot = q_dot0; % Initial velocity
q_history(:, 1) = q;
q_dot_history(:, 1) = q_dot;

for k = 1:(N-1)
    tspan = [time_data(k), time_data(k+1)];
    
    % Calculate desired trajectory
    [A, b] = ConstraintEqns(time_data(k));
    desired_state_history(:,k+1) = b;

    % Calculate control input using ControllerLaw
    u = ControllerLaw(A,b,q,q_dot);
    u_history(:, k) = u;

    % Integrate the dynamics to get q_dot and q_ddot
   [~, q_out] = ode45(@(t, q_combined) combinedDynamics(t, q_combined), tspan, [q; q_dot]);
    
    % Update the state variables
    q = q_out(end, 1:3)'; % Extract next position and orientation
    q_dot = q_out(end, 4:6)'; % Extract next velocities
    q_ddot = Dynamics(q,q_dot,u,time_data(k));

    q = real(q);
    q_dot = real(q_dot);
    q_ddot = real(q_ddot);

    % Store the results
    q_history(:, k+1) = q;
    q_dot_history(:, k+1) = q_dot;
    q_ddot_history(:,k+1) = q_ddot;

    %     % Visualization
    visualizeRobot(q(1), q(2), q(3), r, u, q_history); 
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

    pause(dt);
end

xdesired = desired_state_history(1,:);
ydesired = desired_state_history(2,:);

% Extract states from the results
x = q_history(1, :);
y = q_history(2, :);
theta = q_history(3, :);

x = real(x);
y = real(y);
theta = real(theta);
u_history = real(u_history);

error_x = xdesired - x;
error_y = ydesired - y;

%% Plot the results
figure()
plot(time_data,x);
title('x vs. Time');
xlabel('Time (s)');
ylabel('x (m)');

figure()
plot(time_data, y);
title('y vs. Time');
xlabel('Time (s)');
ylabel('y (m)');

figure()
plot(time_data, theta);
title('theta vs. Time');
xlabel('Time (s)');
ylabel('theta (rad)');


figure;
plot(x,y);
hold on
plot(xdesired,ydesired,'o--');
xlabel('X (m)');
ylabel('Y (m)');
title('Measured vs. Desired Trajectory');
legend('Measured Trajectory', 'Desired Trajectory')


figure
plot(time_data,u_history(1,:),'-o');
hold on 
plot(time_data,u_history(2,:));
plot(time_data,u_history(3,:),'-o');
plot(time_data,u_history(4,:));
xlabel('Time (seconds)');
ylabel('Wheel Torques (Nm)');
title(['Control Torques']);
legend('u_1','u_2','u_3','u_4');

