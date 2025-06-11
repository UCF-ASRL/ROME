function q_ddot = Dynamics(q,q_dot,u,t)
%% constants
m = 18; %kg
r = .05; %meters
l = .35; % meters
d = .45;
w = .45;
Iyy = (1/12)*m*(d^2+w^2);
alpha = [deg2rad(315), deg2rad(45), deg2rad(135), deg2rad(225)];

%% incoming parameters

x = q(1);
y = q(2);
theta = q(3);
x_dot = q_dot(1);
y_dot = q_dot(2);
theta_dot = q_dot(3);

M = [m*r*cos(theta), m*r*sin(theta), 0;
     2*x, 2*y, 0;
     0, 0, (Iyy*r)/(l)];

Q = [m*r*sin(theta)*theta_dot*x_dot - m*r*cos(theta)*theta_dot*y_dot;
     -2*(x_dot^2) - 2*(y_dot^2);
     0];

C = [sin(alpha(1)) sin(alpha(2)) sin(alpha(3)) sin(alpha(4));
     cos(alpha(1)) cos(alpha(2)) cos(alpha(3)) cos(alpha(4));
      1 1 1 1];


Q_c =C*u; 


q_ddot = inv(M)*(Q) + inv(M)*(Q_c);

end