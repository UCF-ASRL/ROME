close all;
clear all;
clc;
%Attempt at compiling path generation code

% time - time of the entire experiment(seconds)
time = 60;
% res - resolution of the path(how many position steps per second)
res = 50;
% a = semi-major axis(dimensionless)
a = 75;
% e = eccentricity of the Orbit(dimensionless)
e = .5;
% incl = Inclination of the orbit(deg)
incl = 10;
% RA = Right Ascension of the Orbit(deg)
RA = 0;
% w = Argument of periapsis of the Orbit(deg)
w = 0;
% Theta = Theta(deg)
theta = 10;
% Z-offset from base of arm
z_off = 0;

% t - time column
time_steps = time*res;
t=zeros(time_steps,1);
t(1,1)=.02;
for i=2:time_steps
    t(i,1) = t(i-1) + 60/time_steps;
end 
theta_steps = 2*pi/time_steps;
theta_vec(1,1) = 0;
for i=2:time_steps
    theta_vec(i,1) = theta_vec(i-1,1)+theta_steps;
end

for i=1:time_steps
    [PosXYZ(:,i), VelXYZ(:,i)] = PERI2XYZ(a, e, incl, RA, w, theta_vec(i,1));
end
%[PosXYZ(:,i), VelXYZ(:,i)] = PERI2XYZ(a, e, incl, RA, w, theta(i,1));
PosXYZ_horiz = PosXYZ';
VelXYZ_horiz = VelXYZ';

%Setting up position arrays for plots
x = PosXYZ_horiz(:,1);
y = PosXYZ_horiz(:,2);
z = PosXYZ_horiz(:,3);
z = z(:)+z_off;
%Define periapsis
periapsis = PosXYZ_horiz(1,:);
%length of coordinate axes
L = 10;
%Plot the orbit
plot3(x,y,z,'k-o', "MarkerSize", .1) 
%Define radius line, object position at time step, setup slider to follow
%orbit over time
hold on;
x_lim = -a:.1:a;
y_lim = -a:.1:a;
%define ref plane
[X,Y] = meshgrid(x_lim,y_lim);
Z = zeros(size(X));
surf(X,Y,Z, 'FaceColor', 'blue', 'FaceAlpha', .2, 'EdgeColor', 'none');
%line to follow satellite from origin
arrow = quiver3(0,0,0, x(1), y(1), z(1), 'bo', 'MarkerSize',4);
%slider to control object across orbit
obj = plot3(x(1), y(1), z(1), 'bo'); % moveable object
slider = uicontrol('Style', 'slider', 'Min', 0, 'Max', length(PosXYZ_horiz),'Value', 1, 'Units', 'normalized', 'Position',[.2 .02 .6 .05]);
addlistener(slider, 'Value', 'PostSet', @(src,evt) updateObj(slider, obj, arrow, x, y, z));
function updateObj(slider, obj, arrow, x, y, z)
    val = round(slider.Value);
    set(obj, 'XData', x(val), 'YData', y(val), "ZData", z(val));
    set(arrow, 'UData', x(val), 'VData', y(val), "WData", z(val));
   
    drawnow;
end 
%Define coordinate axes
quiver3(0,0,0, L,0,0, 'r', 'LineWidth',.5) % x-axis
quiver3(0,0,0, 0,L,0, 'g', 'LineWidth',.5) % y-axis
quiver3(0,0,0, 0,0,L, 'b', 'LineWidth',.5) % z-axis
plot3(periapsis(1), periapsis(2), periapsis(3), 'ko', 'MarkerSize',4); %black dot at periapsis
plot3(0, 0, 0, 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k'); % black dot at origin

%Set angular trajectory
AngPos = zeros(length(PosXYZ_horiz), 3);
AngVel = zeros(length(VelXYZ_horiz), 3);
for i=1:length(PosXYZ_horiz)
    AngPos(i,:) = 0;
    AngVel(i,:) = 0;
end
%XYZ positions and velocities sent to ar2trajvelz3
%traj = [PosXYZ_horiz, VelXYZ_horiz];
PosTraj = [PosXYZ_horiz, AngPos];
VelTraj = [VelXYZ_horiz, AngVel];
%arbitrary initial theta for stepper motors
initialtheta = [0.01, -90, 90, 0.01, 0.01, 0.01];

%% 
%Apply inverse kinematics to calculate ang positions and velocities for
%motors(joints) and then output a 3000x13 path to input into Compilation
%code to run orbital experiments on AR3. path is in rad pathdeg is in deg.
[path, pathdeg] = ar3TrajVelZ3(t, initialtheta, PosTraj, VelTraj);

%save file
Filename = ['t', num2str(time), '_a', num2str(a), '_e', num2str(e), '_incl', num2str(incl), '_RA', num2str(RA), '_w', num2str(w), '_th', num2str(theta), '.mat'];
save(fullfile(pwd, 'TestPaths', Filename), "path", "pathdeg");