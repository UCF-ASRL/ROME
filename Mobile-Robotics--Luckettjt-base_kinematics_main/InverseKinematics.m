function WheelSpeeds = InverseKinematics(q_dot)
%% constants
alpha = [deg2rad(315), deg2rad(45), deg2rad(135), deg2rad(225)];
r = .05;
d = .45;

%% incoming parameters
Vx = q_dot(1);
Vy = q_dot(2);
theta_dot = q_dot(3);


B = [(r/4)*cos(alpha(1)) (r/4)*cos(alpha(2)) (r/4)*cos(alpha(3)) (r/4)*cos(alpha(4));
     (r/4)*sin(alpha(1)) (r/4)*sin(alpha(2)) (r/4)*sin(alpha(3)) (r/4)*sin(alpha(4));
     r/(4*d) -r/(4*d) r/(4*d) -r/(4*d)];

Xdot = [Vx;Vy;theta_dot];

WheelSpeeds = pinv(B)*Xdot;

end