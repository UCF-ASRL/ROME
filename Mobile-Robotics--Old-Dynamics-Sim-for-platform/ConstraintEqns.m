function [A, b] = ConstraintEqns(q,qdot,t)

 % Circle parameters
    R     = 4;         % Radius of the circle
    T     = 30;        % Time for one complete revolution
    omega = 2*pi/T;    % Angular speed
    phi   = 0;         % Phase offset (ensures x=1, y=0 at t=0)
    a = 2;
    % Desired positions along the circle
    x_d = a * R * cos(omega*t + phi);
    y_d = R * sin(omega*t + phi);
    theta_d = 0; % constant orientation
    
    % Desired velocities (first derivative)
    xdot_d = - a * R * omega * sin(omega*t + phi);
    ydot_d =  R * omega * cos(omega*t + phi);
    dtheta_d = 0;
    
    % Desired accelerations (feedforward term)
    ddx_d = -a * R * omega^2 * cos(omega*t + phi);
    ddy_d = -R * omega^2 * sin(omega*t + phi);
    ddtheta_d = 0;
    
    % PD gains 
    Kp = 2;   % proportional gain
    Kd = 1;   % derivative gain
    
    % states extracted from input vectors
    x = q(1);
    y = q(2);
    theta = q(3);
    
    xdot = qdot(1);
    ydot = qdot(2);
    dtheta = qdot(3);
    
    % Position errors
    ex = x_d - x;
    ey = y_d - y;
    etheta = theta_d - theta;
    
    % Velocity errors
    ex_dot = xdot_d - xdot;
    ey_dot = ydot_d - ydot;
    etheta_dot = dtheta_d - dtheta;
    
    % Construct the PD control law 
    b1 = ddx_d + Kp*ex + Kd*ex_dot;
    b2 = ddy_d + Kp*ey + Kd*ey_dot;
    b3 = ddtheta_d + Kp*etheta + Kd*etheta_dot;
    
    % b vector and A matrix 
    b = [b1; b2; b3];
    A = eye(3);


% R     = 1;         % Radius of the circle
% T     = 20;        % Time for one complete revolution
% omega = 2*pi/T;    % Angular speed
% phi   = 0;         % Phase offset
% 
% % Desired accelerations:
% ddx_d = -R * omega^2 * cos(omega*t + phi);
% ddy_d = -R * omega^2 * sin(omega*t + phi);
% ddth_d = 0;  % no desired angular acceleration
% 
% A = eye(3);
% 
%  % A = [2*q(1), 0, 0;
%  %      0, 2*q(2), 0];
%  %      % 0 , 0, 0];
% 
%  % ddx_d = 2*qdot(1)^2;
%  % ddy_d = 2*qdot(2)^2;
% 
% % b is a 3x1 vector of the desired accelerations (with sign such that
% % A*ddq + b = 0 forces ddq = -b when following the desired trajectory)
% b = [ddx_d; ddy_d; ddth_d];
% 
% %     % A = [2*q(1), 2*q(2), 0];
% %     % b = -2*(qdot(1)^2 + qdot(2)^2);
% %     % 
% %   
% %    
% %     % ddx_d = .01;
% %     % ddy_d = 0;
% %     % % %   ddx_d = .2;
% %     % % % ddy_d = .2;
% %     % % % Constraint matrix and RHS
% %     % A = eye(3); % Identity matrix for simple constraints
% %     % b = [ddx_d; ddy_d; 0];
 end

