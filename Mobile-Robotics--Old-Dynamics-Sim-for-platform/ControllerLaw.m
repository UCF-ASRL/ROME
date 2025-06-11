%% Define Controller Law
function u = ControllerLaw(A, b, q, qdot, m, Iyy, r, d, l, mu, g)


 
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
% 
% N = [.10,0,0;
%      0,.10,0;
%      0,0,.10];

% G = A*(inv((N^(1/2))*M));
 
    %% Udwadia-Kalaba Control Law
error = (b - A * (inv(M) * Q))
% disp(['Constraint violation error at time step: ', num2str(error)]);
K = (pinv(C))*(M^(1/2))*(pinv(A*(M^(-1/2))));

tau = (M^(1/2))*(pinv(A*(M^(-1/2))))*(b - A*(inv(M))*Q);
   
u = (pinv(C))*(M^(1/2))*(pinv(A*(M^(-1/2))))*(b - A*(inv(M))*Q);
    % u = (pinv(C))*(N^(-1/2))*(pinv(G))*(b - A*(inv(M))*Q);
end
