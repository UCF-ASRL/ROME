function [PositionXYZ, VelocityXYZ] = PERI2XYZ(a, e, incl, RA, w, theta);

% h = Angular Momentum of the Orbit
% e = eccentricity of the Orbit
% RA = Right Ascension of the Orbit
% incl = Inclination of the orbit
% w = Argument of perigee of the Orbit
% TA = Theta
% MU = Gravitational Paramter of the orbit
deg2rad = pi/180;

incl = incl*deg2rad;
RA = RA*deg2rad;
w = w*deg2rad;
%theta = theta*deg2rad;

h = sqrt(a*(1-e^2));
r = h^2/(1+e*cosd(theta));
%r = h^2/(1+e*cos(theta));

% Position in perifocal coordinate system
PeriPos = [r * cos(theta);  r * sin(theta); 0];

%Velocity in perifocal coordinate system
PeriVel = 1/h * [-sind(theta); e + cosd(theta); 0];
%PeriVel = 1/h * [-sin(theta); e + cos(theta); 0];

%Creating the transformation matrix from Perifocal Frame to XYZ
COmega_3 = [cos(RA) sin(RA) 0; -sin(RA) cos(RA) 0; 0 0 1];
Ci_1 = [1 0 0;0 cos(incl) sin(incl);0 -sin(incl) cos(incl)];
Comega_3 = [cos(w) sin(w) 0; -sin(w) cos(w) 0; 0 0 1];   

C = (Comega_3*Ci_1*COmega_3)';

%The position and velocity vectors in the XYZ frame
PositionXYZ = C * PeriPos;
VelocityXYZ = C * PeriVel;

end