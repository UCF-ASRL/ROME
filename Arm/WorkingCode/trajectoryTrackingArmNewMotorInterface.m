function trajectoryTrackingArmNewMotorInterface(path,AR3Serial)
% This function takes a path, specified as 6 angle states followed by six
% angular velocity states (rad,rad/s) for each joint.
% Stepper1 is a stepper motor object for one of
% the joints on the arm. Arms must first be calibrated.
%{
validity = validateTrajectory(path);
if validity(1) == 0
  disp('The given trajectory is invalid');
  return
end
%}
% Move to initial position
%Starts at (1,2) because (1,1) is time. From validateTrajectory.m 
% "trajectory is specified as a 13 column array, with the first column = time,
%   the subsequent 6 columns = angular position of each joint in radians,
%   and the final 6 columns = angular velocity of each joint in
%   radians/sec." 

pause(5);
statesArray = [path(1,2),path(1,3),path(1,4)...
               path(1,5),path(1,6),path(1,7)...
               .25,.25,.25,.25,.25,.25];
disp(class(AR3Serial));
AR3Serial.updateStates(statesArray);
pause(10);

% Main Loop
index = 1;
tic;

disp('Start')
while(toc <= path(end,1))
    %if (toc >= path(index,1))
    while (index < size(path,1) && toc >= path(index,1))
        fprintf("Update States #");
        disp(index)
        fprintf("toc: %.3f, path(index,1): %.3f\n", toc, path(index,1))
        statesArray = [path(index,2),path(index,3),path(index,4)...
                       path(index,5),path(index,6),path(index,7)...
                       path(index,8),path(index,9),path(index,10)...
                       path(index,11),path(index,12),path(index,13)];
        fprintf("Sending update #%d\n", index);
        AR3Serial.updateStates(statesArray);
        fprintf("Finished update #%d\n", index);
        index = index + 1;
    end
end
end
