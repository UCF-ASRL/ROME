function [H,B] = wheelDrivingSpeeds()

gamma = 0;
beta = [315, 45, 135 , 225];

H = zeros(4,3);
r = .076;
d =.45;

W = [8 0 0 0;
     0 8 0 0;
     0 0 8 0;
     0 0 0 8];

h1 = (1/r)*[1 , tand(gamma)]*[cosd(beta(1)), sind(beta(1));-sind(beta(1)), cosd(beta(1))]*[-d/sqrt(2) 1 0;d/sqrt(2) 0 1];
h2 = (1/r)*[1 , tand(gamma)]*[cosd(beta(2)), sind(beta(2));-sind(beta(2)), cosd(beta(2))]*[-d/sqrt(2) 1 0;-d/sqrt(2) 0 1];
h3 = (1/r)*[1 , tand(gamma)]*[cosd(beta(3)), sind(beta(3));-sind(beta(3)), cosd(beta(3))]*[ d/sqrt(2) 1 0;-d/sqrt(2) 0 1];
h4 = (1/r)*[1 , tand(gamma)]*[cosd(beta(4)), sind(beta(4));-sind(beta(4)), cosd(beta(4))]*[ d/sqrt(2) 1 0;d/sqrt(2) 0 1];

H = [h1;h2;h3;h4];
B = inv(H' * inv(W) * H) * H' * W;
% B = pinv(H)/r;

B = [B(2,1),B(2,2),B(2,3),B(2,4);
     B(3,1),B(3,2),B(3,3),B(3,4);
     B(1,1),B(1,2),B(1,3),B(1,4);]
end

% function [H, B] = wheelDrivingSpeedsSymbolic()
%     %% Declare symbolic variables
%     % To work fully symbolically, declare the parameters as symbolic.
%     syms r d beta1 beta2 beta3 beta4 real
% 
%     % Option 1: Leave parameters symbolic.
%     % beta = [beta1, beta2, beta3, beta4];
%     %
%     % Option 2: To substitute the given values, uncomment these lines:
%      gamma = 0;
%     % beta1 = 315; beta2 = 45; beta3 = 135; beta4 = 225;
%     % r = 0.076;
%     % d = 0.45;
% 
%     % For clarity, we use a beta vector:
%     beta = [beta1, beta2, beta3, beta4];
% 
%     %% Define the weighting matrix W
%     % The original code used a diagonal matrix with 15 on the diagonal.
%     W = diag([15, 15, 15, 15]);
% 
%     %% Build the kinematic matrix H symbolically
%     % Each row is computed via a series of rotations and transformations.
%     % Note: tand, cosd, and sind are used so that angles are in degrees.
%     %
%     % The basic building block is:
%     %   (1/r)*[1, tand(gamma)] * [cosd(beta), sind(beta); -sind(beta), cosd(beta)]
%     % which is then multiplied by a 2x3 matrix that depends on d.
% 
%     % For wheel 1:
%     M1 = [-d/sqrt(2), 1, 0;
%            d/sqrt(2), 0, 1];
%     H1 = (1/r) * [1, tand(gamma)] * [cosd(beta(1)), sind(beta(1)); -sind(beta(1)), cosd(beta(1))] * M1;
% 
%     % For wheel 2:
%     M2 = [-d/sqrt(2), 1, 0;
%           -d/sqrt(2), 0, 1];
%     H2 = (1/r) * [1, tand(gamma)] * [cosd(beta(2)), sind(beta(2)); -sind(beta(2)), cosd(beta(2))] * M2;
% 
%     % For wheel 3:
%     M3 = [ d/sqrt(2), 1, 0;
%           -d/sqrt(2), 0, 1];
%     H3 = (1/r) * [1, tand(gamma)] * [cosd(beta(3)), sind(beta(3)); -sind(beta(3)), cosd(beta(3))] * M3;
% 
%     % For wheel 4:
%     M4 = [ d/sqrt(2), 1, 0;
%            d/sqrt(2), 0, 1];
%     H4 = (1/r) * [1, tand(gamma)] * [cosd(beta(4)), sind(beta(4)); -sind(beta(4)), cosd(beta(4))] * M4;
% 
%     % Stack the rows to form the complete H matrix (4x3):
%     H = [H1; H2; H3; H4]
% 
%     %% Compute the transformation matrix B symbolically
%     % Following the original code:
%     %   B = inv(H' * inv(W) * H) * H' * W;
%     B_temp = inv(H.' * inv(W) * H) * H.' * W;
% 
%     % Reorder the rows of B as in the original code:
%     B = [B_temp(2, :);
%          B_temp(3, :);
%          B_temp(1, :)]
%end