clear all;
close all;
clc;
%Run the matlab code
%data = load('Single Line to Move Ramp Function.mat');
data = load('path_first try_50hz.mat');
%path = data.refTraj;
path = data.path;
%% 
%{
% Number of samples
numPoints = 3000;
totalTime = 60;  % seconds

% Generate time vector (column 1)
time = linspace(0, totalTime, numPoints)';  % 3000x1

% Preallocate path matrix
path = zeros(3000, 13);
path(:,1) = time;

% Generate stepper 6 position (column 6) as a triangle wave: 0 -> 360 -> 0 every 10s
cycleTime = 10;  % seconds per cycle
numCycles = totalTime / cycleTime;
freq = 1 / cycleTime;

% Triangle wave that oscillates between 0 and 360 degrees
position = pi/6 * sawtooth(2 * pi * freq * time, 0.5) + pi/6;  % 0 to 360
 
% Velocity: derivative of position (numerical)
velocity = [0; diff(position) ./ diff(time)];  % prepend 0 to match size

% Assign to columns 6 and 13
path(:,5) = position;
path(:,11) = velocity; %velocity;
path(3000,13) = 0;
%}

%Setup serial object by defining serialport then using that in AR3Serial
serialtest = serialport('COM5', 19200);
AR3Serialtest = AR3Serial(serialtest);

AR3Serialtest.calibrate;
display("calibrating");
%pause(55);
%Stepper Control function... needs path(.mat data file) and a serial
%object(AR3Serial)
trajectoryTrackingArmNewMotorInterface(path,AR3Serialtest);