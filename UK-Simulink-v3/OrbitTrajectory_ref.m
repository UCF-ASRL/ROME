function [q_des, q_dot_des, q_ddot_des] = OrbitTrajectory(t, elements, mu, time_scale, dist_scale)
    % elements = [a, e, i, Omega, omega, nu0]
    
    % Extract and Scale Elements
    a_raw = elements(1);
    e = elements(2);
    i = elements(3);
    Omega = elements(4);
    omega = elements(5);
    nu0 = elements(6); 
    
    a = a_raw * dist_scale;
    scaled_t = t * time_scale;
    
    % Solve for current True Anomaly (nu)
    p = a * (1 - e^2);
    n = sqrt(mu / a^3);
    
    M = n * scaled_t; 
    
    % Iterative Newton-Raphson for Eccentric Anomaly
    E = M; 
    for j = 1:5
        E = E - (E - e*sin(E) - M) / (1 - e*cos(E));
    end
    
    % Solve for current True Anomaly (nu)
    p = a * (1 - e^2);
    n = sqrt(mu / a^3);
    
    % Convert nu0 to an initial Eccentric Anomaly (E0)
    E0 = 2 * atan(sqrt((1-e)/(1+e)) * tan(nu0/2));
    % Convert E0 to an initial Mean Anomaly (M0)
    M0 = E0 - e*sin(E0);
    
    % Current Mean Anomaly is initial + progress
    M = M0 + (n * scaled_t); 
    
    % Iterative Newton-Raphson for Eccentric Anomaly (E)
    E = M; 
    for j = 1:5
        E = E - (E - e*sin(E) - M) / (1 - e*cos(E));
    end
    
    % True Anomaly calculation (Don't add nu0 here!)
    nu = 2 * atan2(sqrt(1+e)*sin(E/2), sqrt(1-e)*cos(E/2));
    
    % Perifocal Position and Velocity (PQW Frame)
    r_mag = p / (1 + e*cos(nu));
    
    r_pqw = [r_mag * cos(nu); 
             r_mag * sin(nu); 
             0];
         
    v_pqw = sqrt(mu/p) * [-sin(nu); 
                          e + cos(nu); 
                          0] * time_scale; 

    % Perifocal Acceleration
    accel_pqw = (-mu / r_mag^3) * r_pqw * (time_scale^2);

    % Rotation Matrix (Perifocal to ECI)
    cO = cos(Omega); sO = sin(Omega);
    co = cos(omega); so = sin(omega);
    ci = cos(i);     si = sin(i);
    
    R = [cO*co-sO*so*ci, -cO*so-sO*co*ci,  sO*si;
         sO*co+cO*so*ci, -sO*so+cO*co*ci, -cO*si;
         so*si,           co*si,           ci];

    % Transform to ECI
    r_eci = R * r_pqw;
    v_eci = R * v_pqw;
    a_eci = R * accel_pqw;

    % Project to 2D Floor (x, y, theta)
    x = r_eci(1);
    y = r_eci(2);
    
    % Get the raw angle (-pi to pi)
    theta_raw = atan2(v_eci(2), v_eci(1));
    
    % Calculate the "Mean Motion" progress
    total_progress = n * scaled_t; 
    
    % Offset the raw angle based on progress
    revolutions = floor((total_progress + nu0) / (2*pi));
    theta = theta_raw + (revolutions * 2*pi);
    
    % Small logic gate to handle the jump-point precisely
    if (theta - total_progress) < -pi
        theta = theta + 2*pi;
    elseif (theta - total_progress) > pi
        theta = theta - 2*pi;
    end
    % =====================================================================
    % FRAME NOTE (Stage 1) -- read before restoring the -theta outputs.
    %
    % heading_offset is DEAD CODE today. theta is computed and offset just
    % below, then thrown away: the outputs at the end of this function force
    % q_des(3) = 0. So the offset currently changes nothing.
    %
    % It stops being harmless the moment those outputs change. The Stage 1
    % heading task (block "Heading Task Null Space") builds its reference as
    %
    %     th_ref = atan2(-q(2), -q(1))
    %
    % in the MATHS convention, 0 = +x. That matches the way
    % InverseKinematics uses q(3) as the argument of a standard rotation
    % matrix. Restoring the commented "-theta" outputs below would push a
    % ROBOT-convention heading (0 = North, and negated) into q_des while the
    % heading task and the wheel map carry on reading q(3) as maths
    % convention. The two would then disagree by pi/2 AND in sign, and
    % nothing in the model would flag it -- the run would simply track the
    % wrong heading.
    %
    % If you restore them: apply the same offset and the same sign inside
    % HeadingTask, then re-run check_stage1 before going near the hardware.
    %
    % See docs/HARDWARE_IMPLEMENTATION.md, Part F 1.4 and Part A6.
    % =====================================================================
    % Mismatch between Math (0=East) and Robot (0=North)
    heading_offset = -pi/2;
    theta = theta + heading_offset;
    
    % Heading tangent to the velocity vector
    x_dot = v_eci(1);
    y_dot = v_eci(2);
    
    % Angular velocity calculation (The "Derivative")
    v_sq = x_dot^2 + y_dot^2;
    if v_sq > 1e-6
        % This formula gives the continuous derivative of atan2(y,x)
        % theta_dot = (x*y_ddot - y*x_ddot) / (x^2 + y^2)
        theta_dot = (v_eci(1)*a_eci(2) - v_eci(2)*a_eci(1)) / v_sq;
    else
        theta_dot = 0;
    end

    % =====================================================================
    % FRAME NOTE (Stage 1) -- theta is zeroed below, deliberately.
    %
    % With the Stage 1 wide constraint A = [I_2 0], Constraints reads only
    % rows 1:2 of q_des, so q_des(3) is never used. Zeroing it costs
    % nothing, and the heading is shaped instead by HeadingTask through the
    % applied force Q. Putting a heading back into a third row of A would
    % square A up again and reinstate the collapse Stage 1 removed.
    %
    % CONSEQUENCE FOR THE HARDWARE: InitialConditions seeds the integrator
    % with q0(3) = 0, so model heading zero is world +x. THE ROBOT MUST BE
    % PLACED POINTING ALONG WORLD +x AT t = 0. Any placement error rotates
    % every heading the task computes, and nothing measures it -- the base
    % has no heading feedback until Stage 2 closes the OptiTrack loop.
    % =====================================================================
    % Assign outputs
    % q_des = [x; y; -theta]; 
    % q_dot_des = [x_dot; y_dot; -theta_dot];
    % q_ddot_des = [a_eci(1); a_eci(2); 0]; 
    q_des = [x; y; 0]; 
    q_dot_des = [x_dot; y_dot; 0];
    q_ddot_des = [a_eci(1); a_eci(2); 0]; 
end