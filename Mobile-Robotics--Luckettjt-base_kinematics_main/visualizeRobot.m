function visualizeRobot(x, y, theta, r, u, trajectory)
    % visualizeRobot - Visualizes the robot's position, orientation, and wheel states
    %
    % Syntax: visualizeRobot(x, y, theta, r, u, trajectory)
    %
    % Inputs:
    %    x - X position of the robot in meters
    %    y - Y position of the robot in meters
    %    theta - Orientation angle of the robot in radians
    %    r - Wheel radius in meters
    %    u - Control input vector (wheel speeds) [4x1]
    %    trajectory - History of positions [2xN]

    clf;  % Clear current figure
    hold on;  % Hold on to plot multiple elements
    axis equal;  % Maintain aspect ratio
    grid on;  % Enable grid

    %% 1. Define and Rotate the Robot Body (Square)
    base_width = 0.4;   % Width of the robot body in meters
    base_height = 0.4;  % Height of the robot body in meters

    % Define the square's corner points relative to the center (0,0)
    square = [
        -base_width/2, -base_height/2;
         base_width/2, -base_height/2;
         base_width/2,  base_height/2;
        -base_width/2,  base_height/2
    ];

    % Create rotation matrix based on current heading angle theta
    R = [cos(theta), -sin(theta);
         sin(theta),  cos(theta)];

    % Rotate the square
    rotated_square = (R * square')';

    % Translate the square to the robot's current position (x, y)
    rotated_square(:,1) = rotated_square(:,1) + x;
    rotated_square(:,2) = rotated_square(:,2) + y;

    % Plot the rotated robot body as a filled polygon
    fill(rotated_square(:,1), rotated_square(:,2), [0.7, 0.7, 0.7], 'EdgeColor', 'k');

    %% 2. Plot the Heading Vector
    arrow_length = 1.5 * r;  % Length of the heading arrow
    quiver(x, y, -arrow_length*sin(theta), arrow_length*cos(theta), 'r', 'LineWidth', 1.5, 'MaxHeadSize', 1);

    %% 3. Plot the Trajectory
    if ~isempty(trajectory)
        plot(trajectory(1,:), trajectory(2,:), 'b', 'LineWidth', 1.2);
    end

    %% 4. Plot Current Robot Position
    plot(x, y, 'go', 'MarkerSize', 8, 'LineWidth', 1.5);

    %% 5. Define and Rotate Wheels and Velocity Vectors
    wheel_radius = 0.025;  % Radius of the wheel in meters
    wheel_angles_deg = [45, 135, 225, 315];  % Original wheel angles in degrees
    wheel_vector_angles_deg = [315, 45, 135, 225];  % Direction of wheel velocity vectors in degrees

    % Convert theta from radians to degrees for consistency
    theta_deg = rad2deg(theta);

    for i = 1:4
        % **a. Rotate Wheel Angles by theta**
        rotated_wheel_angle = wheel_angles_deg(i) + theta_deg;

        % **b. Calculate Wheel Positions with Rotation**
        wheel_x = x + (base_width/2) * cosd(rotated_wheel_angle);
        wheel_y = y + (base_height/2) * sind(rotated_wheel_angle);

        % **c. Draw the Wheel as a Filled Circle**
        rectangle('Position', [wheel_x - wheel_radius, wheel_y - wheel_radius, 2*wheel_radius, 2*wheel_radius], ...
                  'Curvature', [1, 1], 'FaceColor', [0, 0, 0]);

        %% **d. Plot Velocity Vectors for Wheels**

        % Scale factors for better visualization
        scaleFactor1 = 4;
        scaleFactor2 = 4;

        % **i. Rotate Wheel Vector Angles by theta**
        rotated_wheel_vector_angle = wheel_vector_angles_deg(i) + theta_deg;

        % **ii. Velocity Vector (Green) Based on Wheel Speed**
        quiver(wheel_x, wheel_y, ...
               scaleFactor1 * u(i) * cosd(rotated_wheel_vector_angle), ...
               scaleFactor1 * u(i) * sind(rotated_wheel_vector_angle), ...
               'g', 'LineWidth', 1, 'MaxHeadSize', 0.5);

        % **iii. Driving Force Vector (Blue) Based on Wheel Speed and Radius**
        DrivingForce = u * r;  % Assuming proportional to wheel speed
        quiver(wheel_x, wheel_y, ...
               scaleFactor2 * DrivingForce(i) * cosd(rotated_wheel_vector_angle), ...
               scaleFactor2 * DrivingForce(i) * sind(rotated_wheel_vector_angle), ...
               'b', 'LineWidth', 1, 'MaxHeadSize', 0.5);
    end

    %% 6. Adjust Plot Limits
    % Center the plot around the robot's current position
    xlim([x - 1.5, x + 1.5]);
    ylim([y - 1.5, y + 1.5]);

    %% Finalize Visualization
    drawnow;  % Update the figure window
end
