clc;
clear all;
close all;

%% Constants
m = 35; % mass (kg)
r = 0.075; % wheel radius (meters)
d = 0.45; % length of robot side(s) (meters)
w = 0.45; % width of robot side(s) (meters)
l = d/(sqrt(2)); % distance from robot C.G to wheels (meters)
Iyy = (1/12)*m*(d^2 + w^2); % base moment of inertia (kg*m^2)
g = 9.81; % gravity (m/s^2)
mu = 0.1; % coefficient of friction


% Initial conditions
q0    = [8; 0; 0];   % [x; y; theta]
qdot0 = [0.0; 0.0; 0];
X0    = [q0; qdot0]; % 6x1 combined state

%% Time settings
dt = .0500;  % Time step
Tmax = 90; % Total simulation duration

%% Storage for data
time_data = 0:dt:Tmax;

% Preallocate histories
numSteps = length(time_data);
q_history      = zeros(3, numSteps);
qdot_history   = zeros(3, numSteps);
u_history      = zeros(4, numSteps-1);   
WheelSpeeds_history = zeros(4, numSteps-1);
desired_state_history = zeros(3, numSteps);

% Initial conditions (assumes q0 and qdot0 are known)
q_history(:,1)    = q0;
qdot_history(:,1) = qdot0;

% For visualization / animation
filename = 'robotAnimation.gif';

for k = 1:(length(time_data)-1)
    tspan = [time_data(k), time_data(k+1)];
    
    % ---------------------------------------------------------
    % 1) Compute desired trajectory at next time (for info/logging)
    Ydesired_next = desiredTrajectory(time_data(k+1));
    desired_state_history(:, k+1) = Ydesired_next;
    
    % ---------------------------------------------------------
    % 2) Compute control input

    qk      = q_history(:, k);
    qdotk   = qdot_history(:, k);
    [A, b] = ConstraintEqns(qk, qdotk, time_data(k));
    u       = ControllerLaw(A, b, qk, qdotk, m, Iyy, r, l, mu, g);
    u_history(:, k) = real(u);  % store for post-processing
    
    % ---------------------------------------------------------
    % 3) Integrate the full 6D state [q; qdot] from t1 to t2
    x0 = [qk; qdotk];
    
    odefun = @(t, x) fullStateDynamics(t, x, u, m, Iyy, r, d, l, mu, g);
    [~, xout] = ode45(odefun, tspan, x0);
    
    % The final state at t2
    xf = xout(end, :)';
    q_next    = real(xf(1:3));
    qdot_next = real(xf(4:6));
    
    % Store them
    q_history(:, k+1)    = q_next;
    qdot_history(:, k+1) = qdot_next;

    % ---------------------------------------------------------
    % 4) From the final qdot, you can compute WheelSpeeds 

    WheelSpeeds = real(InverseKinematics(qdot_next));
    WheelSpeeds_history(:, k) = WheelSpeeds;
    
    %---------------------------------------------------------
    % 5) Visualization at the new state
    visualizeRobot(q_next(1), q_next(2), q_next(3), r, u, q_history);
    drawnow;

    % (Optional) write to GIF
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if k == 1
        imwrite(imind, cm, filename, 'gif','Loopcount', inf, 'DelayTime', dt);
    else
        imwrite(imind, cm, filename, 'gif','WriteMode', 'append','DelayTime', dt);
    end

    %  real-time pause
    % pause(dt);

    % ---------------------------------------------------------
    % 6)  *All intermediate* points, 
    %    store tout and xout in arrays for post-processing:
    %        time_log{k} = tout;
    %        state_log{k} = xout; 
end



%% Post-Simulation Analysis
xdesired = desired_state_history(1, :);
ydesired = desired_state_history(2, :);

% Extract states from the results
x = q_history(1, :);
y = q_history(2, :);
theta = q_history(3, :);

% x = real(x);
% y = real(y);
% theta = real(theta);
% 
% u_history = real(u_history);

error_x = xdesired - x;
error_y = ydesired - y;

%% Plot the Results
figure();
plot(time_data, x);
title('x vs. Time');
xlabel('Time (s)');
ylabel('x (m)');

figure();
plot(time_data, y);
title('y vs. Time');
xlabel('Time (s)');
ylabel('y (m)');

figure();
plot(time_data, theta);
title('theta vs. Time');
xlabel('Time (s)');
ylabel('theta (rad)');

figure;
plot(x, y);
hold on;
plot(xdesired, ydesired, 'o--');
xlabel('X (m)');
ylabel('Y (m)');
title('Measured vs. Desired Trajectory');
legend('Measured Trajectory', 'Desired Trajectory');

figure;
plot(time_data(1:end-1), u_history(1, :), '-o');
hold on;
plot(time_data(1:end-1), u_history(2, :));
plot(time_data(1:end-1), u_history(3, :), '-o');
plot(time_data(1:end-1), u_history(4, :));
xlabel('Time (s)');
ylabel('Wheel Torques (Nm)');
title('Control Torques');
legend('u_1', 'u_2', 'u_3', 'u_4');
xlabel('Time (s)');
ylabel('Wheel Torques (Nm)');
title('Control Torques');
legend('u_1', 'u_2', 'u_3', 'u_4');

