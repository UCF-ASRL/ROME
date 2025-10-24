clear all;
close all;
clc;
%Run th00. e matlab code
%data = load('Single Line to Move Ramp Function.mat');
%data = load('t60_a75_e0_incl0_RA0_w0_th0.mat');


%path = data.refTraj;
%path = data.pathDeg;

 path = zeros(5,13);
 path(:,1) = 0; 
 path(1,:) = [0,10,110,30,0,20,-20,700,700,700,700,0,700];
 %path(3,:) = [0,0,0,0,0,0,0,200,0,0,0,0,0];
%Setup serial object by defining serialport then using that in AR3Serial
serialtest = serialport('COM5', 19200);
AR3Serialtest = AR3Serial(serialtest);

AR3Serialtest.calibrate;
display("calibrating");
pause(45);
%disp(AR3Serialtest.read_all)
%Stepper Control function... needs path(.mat data file) and a serial
%object(AR3Serial)            
trajectoryTrackingArmNewMotorInterface(path,AR3Serialtest);