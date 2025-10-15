function WheelSpeeds = InverseKinematics(q_dot)
%% constants
% alpha = [deg2rad(315), deg2rad(45), deg2rad(135), deg2rad(225)];
alpha = [deg2rad(45), deg2rad(135), deg2rad(225), deg2rad(315)];

r = 0.0762;
d = .45;

%% incoming parameters
Vx = q_dot(1);
Vy = q_dot(2);
theta = q_dot(3);


% B = [(r/4)*cos(alpha(1)) (r/4)*cos(alpha(2)) (r/4)*cos(alpha(3)) (r/4)*cos(alpha(4));
%      (r/4)*sin(alpha(1)) (r/4)*sin(alpha(2)) (r/4)*sin(alpha(3)) (r/4)*sin(alpha(4));
%      r/(4*d) -r/(4*d) r/(4*d) -r/(4*d)];
% 
% Xdot = [Vx;Vy;theta_dot];
% 
% WheelSpeeds = pinv(B)*Xdot;


%% different derivation - keanu
%% Source: https://eprints.undip.ac.id/79371/1/C14-Development_of_Omni-Wheeled_Mobile_Robot_Based-on_Inverse_Kinematics_and_Odometry.pdf

%% IK transform matrix from body velocities to angular wheel speeds
A = (1/r)*[sin(alpha); cos(alpha); repmat(r, 1, 4)]; 
Xdot = [Vx;Vy;theta];
WheelSpeeds = A.'*Xdot; % Transpose

end