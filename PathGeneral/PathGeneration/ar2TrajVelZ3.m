function [path,pathDeg] = ar2TrajVelZ3(time,initialtheta,refTraj)
initialtheta = [0.01, -90, 90, 0.01, 0.01, 0.01];

tic;

%created for the trajectory tracking problem
%-------------------------------------------------

% This function takes a generated trajectory with 6 states, x,y,z pos and
% xdot, ydot, zdot velocity. The script extracts all z axis motion and
% generates a trajectory in joint space. The inverse jacobian is used to
% generate a joint velocity command, and using intergration, the joint
% states are generated.

%-------------------------------------------------

%all angle units are in rads


% SERIAL LINK FOR ROBOTICS TOOLBOX BY PETER CORKE: ONLY HERE SO I DONT LOSE
% IT
% 
% L(1) = Link([0 169.77 64.2 -1.5707], 'R');
% L(2) = Link([0 0 305 0], 'R');
% L(3) = Link([-1.5707 0 0 1.5707], 'R');
% L(4) = Link([0 -222.63 0 -1.5707], 'R');
% L(5) = Link([0 0 0 1.5707], 'R');
% L(6) = Link([pi -36.25 0 0], 'R');
% 
% Robot = SerialLink(L);
% Robot.name = 'AR2_Robot';

%-------------------------------------------------
%time of manuever in seconds

t=60;
%t=time;


radToDeg=180/pi;
degToRad=pi/180;

%cannonical units
%load('refTraj_10inc_expirement.mat');
chi=length(refTraj);

%convert canonical units to mm and mm/s - here for converting
scale=1;
refTraj=refTraj*scale;


% 
scaleV=1;
%refTraj(:,2)=refTraj(:,2)*scaleV;
%convert velocities 3000x6 reftraj version
for i=4:6
    refTraj(:,i)=refTraj(:,i)*scaleV;
end
%% 

%initial theta positions at start of maneuver -- find different initial
%angle
theta=cell(chi,6);
theta{1}=initialtheta';
theta{1} = (pi/180)*[0;-70;90;0;12;0];
%% 

%theta0(3)=theta0(3)-90;
%theta0(6)=theta0(6)+180;

velE_ref = zeros(chi,3);
velE_ref(:,3) = refTraj(:,2);
%% 

%timesteps between points
ts=t/chi;

tv=zeros(chi,1);
for i=1:t/ts
    tv(i)=ts*i;
end
 

%% 

%calculate joint Velocities
thetad=cell(chi,6);
for j=1:chi
    thetad{j}(1:6)=0;
end
thetadtemp=cell(chi,1);

%% 

for i=1:chi
%       Calculate new theta dots
        thetadtemp{i} = trajectoryIK3(velE_ref(i,:)',theta{i});
        thetad{i}(2)=thetadtemp{i}(1);
        thetad{i}(3)=thetadtemp{i}(2);
        thetad{i}(5)=thetadtemp{i}(3);
%       Calculate new theta to feed back
        if i == chi
            break
        end
        
%       Forward Integrate to find Angles
        theta{i+1}=theta{i}+(thetad{i}*ts); 
        
end
%-------------------------------------------------
%% 

theta=theta';
%% 

theta=cell2mat(theta);
%theta=theta';

thetad=thetad';
%% 

thetad=cell2mat(thetad);
%thetad=thetad';
%%
% Converted to Path File Needed For Control.
%path=[tv theta thetad];
length(time)
length(theta)
length(thetad)
path=[time theta thetad];
pathDeg=path;
pathDeg(:,2:end)=path(:,2:end)*180/pi;

% downsample
% oL = length(unnamed);
% downsampled = interp1(1:oL, unnamed, linspace(1,oL,3000));
% downsampled=downsampled'

validity=validateTrajectory(path);
isvalid=validity(1);

    if isvalid == 0
        disp("The path is not valid! Run validateTrajectory to check why.");
    else
        disp("The trajectory is valid!");
    end

toc;
end