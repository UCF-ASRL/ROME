function WheelSpeeds = InverseKinematics(q_dot)
%% constants
alpha = [deg2rad(315), deg2rad(45), deg2rad(135), deg2rad(225)];
r = .075;
d = .45;

%% incoming parameters
Vx = q_dot(1);
Vy = q_dot(2);
theta_dot = q_dot(3);

gamma = 0;
beta = [315, 45, 135 , 225];

H = zeros(4,3);
r = .076;
d =.45;


h1 = (1/r)*[1 , tand(gamma)]*[cosd(beta(1)), sind(beta(1));-sind(beta(1)), cosd(beta(1))]*[-d/sqrt(2) 1 0;d/sqrt(2) 0 1];
h2 = (1/r)*[1 , tand(gamma)]*[cosd(beta(2)), sind(beta(2));-sind(beta(2)), cosd(beta(2))]*[-d/sqrt(2) 1 0;-d/sqrt(2) 0 1];
h3 = (1/r)*[1 , tand(gamma)]*[cosd(beta(3)), sind(beta(3));-sind(beta(3)), cosd(beta(3))]*[ d/sqrt(2) 1 0;-d/sqrt(2) 0 1];
h4 = (1/r)*[1 , tand(gamma)]*[cosd(beta(4)), sind(beta(4));-sind(beta(4)), cosd(beta(4))]*[ d/sqrt(2) 1 0;d/sqrt(2) 0 1];

H = [h1;h2;h3;h4];
H = [H(1,2), H(1,3), H(1,1);
     H(2,2), H(2,3), H(2,1); 
     H(3,2), H(3,3), H(3,1);
     H(4,2), H(4,3), H(4,1)];


Xdot = [Vx;Vy;theta_dot];

WheelSpeeds = H*Xdot;

end
