clear all;
close all;
clc;
%Run the matlab code
%The following 3 MATLAB blocks are not all supposed to be used. This script
%is almost complete, but still incomplete. During the testing phase the
%following 3 blocks are to be used as needed(only one at a time). 

%Note: Each row consists of a time point, 6 motor positions(deg), and 6
%motor velocities(accelstepper values(NOT deg/s)).

%% 1. Loads pregenerated paths. 

% Path generated before any of the current lab members:
data = load('Single Line to Move Ramp Function.mat');
% The following is loading a joint trajectory from the orbital path gen
% code. This is the naming convention chosen. 
% data = load('t60_a75_e1_incl0_RA0_w0_th10.mat');

% .mat files may have multiples parts such as refTraj, path, pathdeg,
% etc... be wary of this when assigning variables:
% path = data.refTraj;
% path = data.path;

%% 2. Moves each motor certain amount of degrees set by the numerator in d#
% This hasn't been tested yet. After proving 3. works, try this, then 1.
% when orbital path generation is working.

% d1=45/3000;
% d2=30/3000;
% d3=30/3000;
% d5=-45/3000;
% d6=45/3000;
% path = zeros(3000,13);
% %path(1,:) =[0,0, 90, 90, 1, 0, 0, 500, 0, 0, 0, 0, 0]; 
% path(1,:) =[0,0, 0, 0, 0, 0, 0, 500, 0, 0, 0, 0, 0]; 
% dt = 60/3000;
% for i=2:3000 
%     path(i,1) = path(i-1,1) + dt; 
%     path(i,2) = path(i-1,2) + d1;
%     % path(i,3) = path(i-1,3) + d2;
%     % path(i,4) = path(i-1,4) + d3;
%     % %path(i,5) = path(i-1,5) + d4;
%     % path(i,6) = path(i-1,6) + d5;
%     % path(i,7) = path(i-1,7) + d6;
%     path(i,8) = (path(i,2)-path(i-1,2))/dt;
%     % path(i,9) = (path(i,3)-path(i-1,3))/dt;
%     % path(i,10) = (path(i,4)-path(i-1,4))/dt;
%     % %path(i,11) = (path(i,5)-path(i-1,5))/dt;
%     % path(i,12) = (path(i,6)-path(i-1,6))/dt;
%     % path(i,13) = (path(i,7)-path(i-1,7))/dt;
% end

%% 3. Not a path, but points(deg) that the motors will travel to. 
% path = zeros(2,13);
% path(:,1) = 0; 
% path(1,:) = [0,10,110,80,0,20,-20,700,700,700,700,0,700];
% path(2,:) = [0,20,100,70,0,30,-30,700,700,700,700,0,700];
%path(3,:) = [0,0,0,0,0,0,0,200,0,0,0,0,0];

%% Setup, calibration, and stream data over serial
%Setup serial object by defining serialport then using that in AR3Serial
serialtest = serialport('COM5', 19200);
AR3Serialtest = AR3Serial(serialtest);
%Calibrate
AR3Serialtest.calibrate;
display("calibrating");
pause(25);
display("done calibrating");
%Stream data to Teensy over serial
trajectoryTrackingArmNewMotorInterface(path,AR3Serialtest);