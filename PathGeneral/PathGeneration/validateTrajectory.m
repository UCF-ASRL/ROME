function output = validateTrajectory(trajectory)
%VALIDATETRAJECTORY Takes a trajectory in joint frames and determines validity.
%   trajectory is specified as a 13 column array, with the first column = time,
%   the subsequent 6 columns = angular position of each joint in radians,
%   and the final 6 columns = angular velocity of each joint in radians/sec.
%   total will return 1 if entire trajectory is valid, 0 otherwise. The
%   remaining 6 return values will return 1 if the trajectory is valid for that
%   joint, otherwise the difference between the trajectory and the limit for the
%   largest trajectory overshoot for that joint.

% steps/radian for each joint
STEPPER_CONSTANT(1) = 2561.45838425888;
STEPPER_CONSTANT(2) = [3168.63019224010];
STEPPER_CONSTANT(3) = [3212.65619120146];
STEPPER_CONSTANT(4) = [2639.07836747402];
STEPPER_CONSTANT(5) = [1248.229482];
STEPPER_CONSTANT(6) = [1224.46625127950];

% Speed limit in steps/second
speedLimit = 1000;
degToRad = (pi/180);

% Counter clockwise position limits from the zero
CCWLimit(1) = (170)*degToRad;
CCWLimit(2) = (0)*degToRad;
CCWLimit(3) = (141)*degToRad;
CCWLimit(4) = (155)*degToRad;
CCWLimit(5) = (105)*degToRad;
CCWLimit(6) = (155)*degToRad;

% CCWLimit(2) = (42)*degToRad;
% CCWLimit(3) = (89)*degToRad;

% Clockwise position limits from the zero
CWLimit(1) = -(170)*degToRad;
CWLimit(2) = -(132)*degToRad;
CWLimit(3) = -(0)*degToRad;
CWLimit(4) = -(155)*degToRad;
CWLimit(5) = -(105)*degToRad;
CWLimit(6) = -(155)*degToRad;

% CWLimit(2) = -(90)*degToRad;
% CWLimit(3) = -(51)*degToRad;

J = zeros(6,1);

for i = 1:6
  for j = 1:length(trajectory(:,i+1))
    if trajectory(j,i+1) >= CCWLimit(i)
      diff = trajectory(j,i+1) - CCWLimit(i);
      if diff > abs(J(i))
        J(i) = diff;
      end
    elseif trajectory(j,i+1) <= CWLimit(i)
      diff = trajectory(j,i+1) - CWLimit(i);
      if abs(diff) > abs(J(i))
        J(i) = diff;
      end
    end
  end
end

JDot = zeros(6,1);
for i = 7:12
  for j = 1:length(trajectory(:,i+1))
    if abs(trajectory(j,i+1)*STEPPER_CONSTANT(i-6)) >= speedLimit
      diff = abs(trajectory(j,i+1)*STEPPER_CONSTANT(i-6)) - speedLimit;
      if diff > JDot(i-6)
        JDot(i-6) = diff;
      end
  end
end

total = 1;
for i = 1:6
  if (abs(J(i)) > 0) || (JDot(i) > 0)
    total = 0;
    break
  end
end

output = [[total,J(1),J(2),J(3),J(4),J(5),J(6),JDot(1),JDot(2),JDot(3),JDot(4),JDot(5),JDot(6)];
          [total*pi/180, J(1),J(2),J(3),J(4),J(5),J(6),JDot(1),JDot(2),JDot(3),JDot(4),JDot(5),JDot(6)]*180/pi;
                                                               ];
end
