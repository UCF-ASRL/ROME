function ddx = fullStateDynamics(t, x, u, m, Iyy, r, d, l, mu, g)
    % x(1:3) = q,   x(4:6) = qdot
    q    = x(1:3);
    qdot = x(4:6);
    
    % -- 1) Accelerations from existing dynamics function
    qddot = Dynamics(t, q, qdot, u, m, Iyy, r, d, l, mu, g);
    
    % -- 2) Build dx/dt
    ddx = [qdot;  % derivative of position is velocity
           qddot];% derivative of velocity is acceleration
end
