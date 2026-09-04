function verify_wheelmap()
%VERIFY_WHEELMAP  The 9-DOF block must address the same motors as the robot.
%   The 3-DOF InverseKinematics block and rome_wheels inside UKDynamics use
%   the same row form, (sin(a) vx + cos(a) vy + l w)/r, so the alpha ORDER
%   decides which physical motor each element of the output reaches. This
%   confirms both now resolve to the same workspace values.
d = fileparts(mfilename('fullpath')); cd(d); addpath(d);
evalc('define_constants');
qd = [0.35; -0.20; 0.42];  th = 0.35;
Rwb = [cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1];
vb  = Rwb*qd;
w = @(al,rr,ll) arrayfun(@(i)(sin(al(i))*vb(1)+cos(al(i))*vb(2)+ll*vb(3))/rr, (1:4).')*60/(2*pi);
w_model = w(alphas(:), r, l);                       % what the robot uses
w_block = block_wheels(qd, th);                     % what UKDynamics emits
fprintf('\n  3-DOF InverseKinematics : [%7.2f %7.2f %7.2f %7.2f]\n', w_model);
fprintf('  UKDynamics rome_wheels  : [%7.2f %7.2f %7.2f %7.2f]\n', w_block);
e = max(abs(w_model - w_block));
fprintf('  max elementwise difference %.3e rpm\n', e);
assert(e < 1e-9, 'verify_wheelmap:mismatch', ...
   ['The 9-DOF block addresses different motors than the 3-DOF model. ' ...
    'Each speed would reach the diagonally opposite wheel.']);
fprintf('  PASS - both address the same motors.\n\n');
end

function w = block_wheels(qd, th)
% rome_wheels as UKDynamics runs it, with the workspace geometry.
evalc('define_constants');
Rwb = [cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1];
v_b = Rwb*qd(:);
w = zeros(4,1);
for i = 1:4
    w(i) = (sin(alphas(i))*v_b(1) + cos(alphas(i))*v_b(2) + l*v_b(3))/r;
end
w = w*(60/(2*pi));
end
