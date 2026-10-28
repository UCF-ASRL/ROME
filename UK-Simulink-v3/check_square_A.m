function T = check_square_A()
%CHECK_SQUARE_A  Effect of the shape of the constraint matrix on the UK solve,
%on the 3-DOF base q = [x y theta].
%
%   With A square and invertible the fundamental equation reduces to
%   qdd = A \ b, independent of M and Q (Udwadia and Kalaba, 1996). With a
%   wide A the free directions follow M and Q. The five cases below measure
%   both statements. Errors if any measured value contradicts them.

mb = 18.0;                          % base mass (kg), define_constants.m
Ib = (1/12)*mb*(0.45^2 + 0.45^2);   % base yaw inertia (kg m^2)
b3 = [0.3; -0.2; 0.5];              % 3x1 constraint right-hand side (m/s^2, rad/s^2)
b2 = b3(1:2);                       % 2x1 right-hand side for the wide case

uk = @(M, Q, A, b) M\Q + (M\A.')*((A*(M\A.'))\(b - A*(M\Q)));
                                    % square-root-free fundamental equation

T = struct('name', {}, 'value', {});

% 1  A = I3, three very different mass matrices
Ms = {diag([mb mb Ib]), diag([1 1 1]), diag([500 0.2 40])};
d = 0;
for k = 1:3
    d = max(d, norm(uk(Ms{k}, [1; -2; 3], eye(3), b3) - b3));
end
T(end+1) = struct('name', 'A = I3, three mass matrices: max |qdd - b|', 'value', d);

% 2  A = I3, large applied torque on theta
d = norm(uk(diag([mb mb Ib]), [0; 0; 99], eye(3), b3) - b3);
T(end+1) = struct('name', 'A = I3, Q3 = 99 N m: |qdd - b|', 'value', d);

% 3  A = [I2 0], applied torque swept 0 to 50 N m
Aw = [eye(2) zeros(2,1)];
tau = linspace(0, 50, 6);
qx = zeros(2, numel(tau));  qth = zeros(1, numel(tau));
for k = 1:numel(tau)
    qdd = uk(diag([mb mb Ib]), [0; 0; tau(k)], Aw, b2);
    qx(:,k) = qdd(1:2);  qth(k) = qdd(3);
end
T(end+1) = struct('name', 'A = [I2 0], torque sweep: spread of x,y rows', ...
                  'value', max(max(qx,[],2) - min(qx,[],2)));
T(end+1) = struct('name', 'A = [I2 0], torque sweep: spread of theta row (rad/s^2)', ...
                  'value', max(qth) - min(qth));

% 4  A = [I2 0], diagonal M against the identity metric
Q = [2; -1; 4];
d = norm(uk(diag([mb mb Ib]), Q, Aw, b2) - uk(eye(3), diag([mb mb Ib])\Q, Aw, b2));
T(end+1) = struct('name', 'A = [I2 0], diagonal M vs identity metric: |difference|', 'value', d);

% 5  A = [I2 0], M with normalized x-theta coupling 0.9
Mc = diag([mb mb Ib]);
Mc(1,3) = 0.9*sqrt(mb*Ib);  Mc(3,1) = Mc(1,3);
d = norm(uk(Mc, Q, Aw, b2) - uk(eye(3), Mc\Q, Aw, b2));
T(end+1) = struct('name', 'A = [I2 0], coupled M (0.9) vs identity metric: |difference|', 'value', d);

for k = 1:numel(T)
    fprintf('  %-66s %10.3g\n', T(k).name, T(k).value);
end

if T(1).value > 1e-12 || T(2).value > 1e-12 || T(3).value > 1e-12 || ...
   T(4).value < 1 || T(5).value > 1e-12 || T(6).value < 1e-3
    error('check_square_A:fail', 'A measured value contradicts the stated result.');
end
end
