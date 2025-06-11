function ddq = Dynamics(t, q, qdot, u, m, Iyy, r, d, l,mu,g)



 Q = [m*qdot(1)*qdot(3)*sin(q(3)) - m*qdot(2)*qdot(3)*cos(q(3));
      m*qdot(1)*qdot(3)*cos(q(3)) + m*qdot(2)*qdot(3)*sin(q(3));
      0];
 C = [1/(r*sqrt(2)), 1/(r*sqrt(2)), -1/(r*sqrt(2)), -1/(r*sqrt(2));
      -1/(r*sqrt(2)), 1/(r*sqrt(2)), 1/(r*sqrt(2)), -1/(r*sqrt(2));
      -d/(r*Iyy), -d/(r*Iyy), -d/(r*Iyy), -d/(r*Iyy)];

 M = [m*cos(q(3)), m*sin(q(3)), 0;
     -m*sin(q(3)), m*cos(q(3)), 0;
      0, 0, 1];
 % M = diag([m, m, (Iyy*r)/d]);
 % R = [cos(q(3)), sin(q(3)), 0;
 %     -sin(q(3)), cos(q(3)), 0;
 %      0, 0, 1];


    %% Total Constriant Forces
    Qc = C*u;

    %% Generalized Accelerations
    q_ddot = (inv(M)*Q + inv(M)*Qc);

    ddq = [q_ddot];
end
