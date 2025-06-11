function Yd = desiredTrajectory(t) % currently a circle
    R = 4; % Radius of the circle. Adjust this as needed.
    T = 25; % Time taken for one complete circle
    omega = 2*pi/T;
    phi = 0; % Starting phase. This can be adjusted.
   a = 2;
    % Desired positions along the circle
    x_d = a * R * cos(omega*t + phi);
    y_d = R * sin(omega*t + phi);
    theta_d = 0; % constant orientation
    % x_d = (1/2)*t;
    % y_d = 0;
    % theta_d = 1/10*(t^2);
    % x_d = cos(t);
    % y_d = sin(t);

    Yd = [x_d; y_d; theta_d];

    %   % Waypoints in time
    % t_way = [0, 4, 8, 12, 16, 20];  %  6 waypoints at these times
    % 
    % % Corresponding x- and y-waypoints
    % x_way = [0,  2,  2,  -2,  -2,  0]; 
    % y_way = [0,  2,  -2,  2,  2,  0];
    % 
    % % Define persistent spline structures so we only compute them once
    % persistent sx sy
    % if isempty(sx) || isempty(sy)
    %     % Build cubic spline "pp" structures for x(t) and y(t)
    %     sx = spline(t_way, x_way);
    %     sy = spline(t_way, y_way);
    % end
    % 
    % % Evaluate splines at time t
    % x_d = ppval(sx, t);
    % y_d = ppval(sy, t);
    % 
    % % Optionally define theta from the trajectory slope or just set to 0
    % % For simplicity, we set it to 0 here
    % theta_d = 0; 
    % 
    % % Return the desired state as a 3x1 vector
    % Yd = [x_d; y_d; theta_d];

%     %   Square trajectory parameters
%     side_length = 1; % Length of each side of the square
%     T = 20; % Time taken for one complete square path (perimeter)
%     speed = 4 * side_length / T; % Speed along each side of the square
%     theta_d = 0;
%         Moving along the second side (up)
%         x_d = side_length;
%         y_d = speed * (t - T/4);
% else if t < 3*T/4
%         Moving along the third side (left)
%         x_d = side_length - speed * (t - T/2);
%         y_d = side_length;
%     else
%         Moving along the fourth side (down)
%         x_d = 0;
%         y_d = side_length - speed * (t - 3*T/4);
%     end
% 
%     Yd = [x_d; y_d; theta_d];
  % % Define the base parameters for the trajectory
  %   T = 45; % Time taken for one complete circle
  %   omega = 2 * pi / T; % Angular velocity
  %   phi = 0; % Starting phase
  % 
  %   % Define the time-dependent radius R(t)
  %   %R_t = 1 + 0.1 * t; %  Linearly increasing radius
  %    R_t = 1 + 0.05 * t^2; %  Quadratically increasing radius
  %   % R_t = 1 + 0.5 * sin(0.1 * t); %  Sinusoidal radius variation
  % 
  %   % Compute the desired trajectory
  %   x_d = R_t * cos(omega * t + phi);
  %   y_d = R_t * sin(omega * t + phi);
  %   theta_d = 0; % Orientation remains constant for simplicity
  % 
  %   % Output the desired trajectory
  %   Yd = [x_d; y_d; theta_d];
end