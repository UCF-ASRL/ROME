function dq_combined = combinedDynamics(t, q_combined)

    % Split q_combined into q and q_dot
    q = q_combined(1:3);
    q_dot = q_combined(4:6);

    % Calculate control input inside the function
    [A, b] = ConstraintEqns(t);
    u = ControllerLaw(A, b, q, q_dot);
    
    % Convert the robot's velocities (q_dot) to wheel speeds
    WheelSpeeds = InverseKinematics(q_dot);
    
    % Convert wheel speeds back to robot's velocities (update q_dot)
    q_dot = ForwardKinematics(WheelSpeeds);
    
    % Compute q_ddot using the Dynamics function
    q_ddot = Dynamics(q, q_dot, u, t);
    
    % The derivative of q is q_dot, and the derivative of q_dot is q_ddot
    dq_combined = [q_dot; q_ddot];
end