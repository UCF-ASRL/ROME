function Yd = desiredTrajectory(t) % currently a circle
    R = 1; % Radius of the circle. Adjust this as needed.
    T = 15; % Time taken for one complete circle
    omega = 2*pi/T;
    phi = 0; % Starting phase. This can be adjusted.
    a = 1;
    b = 1;

    x_d = a * R * cos(omega * t + phi);
    y_d = b * R * sin(omega * t + phi);
    theta_d = -omega*t; % Keeping orientation constant
    % x_d = 2;
    % y_d = 2;
    % theta_d = 0;

    Yd = [x_d; y_d; theta_d];

    %   Square trajectory parameters
    % side_length = 1; % Length of each side of the square
    % T = 20; % Time taken for one complete square path (perimeter)
    % speed = 4 * side_length / T; % Speed along each side of the square
    % theta_d = 0;
    %     Moving along the second side (up)
    %     x_d = side_length;
    %     y_d = speed * (t - T/4);
    % elseif t < 3*T/4
    %     Moving along the third side (left)
    %     x_d = side_length - speed * (t - T/2);
    %     y_d = side_length;
    % else
    %     Moving along the fourth side (down)
    %     x_d = 0;
    %     y_d = side_length - speed * (t - 3*T/4);
    % end
    % 
    % Yd = [x_d; y_d; theta_d];
end