function verify_scenarios()
%VERIFY_SCENARIOS  Check every reference against the manuscript and the CW
%   equations. Errors on failure.
%
%   Checks, in order:
%     1  scenario 1 (ellipse) still reproduces OrbitTrajectory exactly
%     2  V-bar holds the radial coordinate identically zero, Eq. (24)
%     3  R-bar holds the in-track coordinate identically zero, Eq. (25)
%     4  NMC keeps the 2:1 in-track to radial axis ratio, Eq. (26)
%     5  NMC satisfies the UNFORCED Clohessy-Wiltshire equations exactly.
%        The manuscript states this is what makes it a natural motion, so it
%        is the strongest available check on the reference.
%     6  vel and acc are the true derivatives of pos in every scenario
%     7  u_des is a unit quaternion and psi_dot matches the yaw it implies
%
%   Parameters are the manuscript's Table of simulation parameters:
%     V-bar  y0  = 100 m,  v0 = 0.50 m/s
%     R-bar  x0  =  50 m,  v0 = 0.25 m/s
%     NMC    rho =  50 m,  phi = 0
%     chief mean motion n = 1.078e-3 rad/s (circular orbit, a = 7000 km)

here = fileparts(mfilename('fullpath'));  cd(here);  addpath(here);
evalc('define_constants');

n_chief = 1.078e-3;         % chief mean motion (rad/s), manuscript Sec. 5
VB  = [100,  0.50, 0, 0];   % V-bar  [y0 v0 . .]
RB  = [ 50,  0.25, 0, 0];   % R-bar  [x0 v0 . .]
NMC = [ 50, n_chief, 0, 0]; % NMC    [rho n phi .]
zw  = z_work;
D   = 1;  S = 1;            % unscaled, so the comparison is against the
                            %   manuscript's own numbers

fprintf('\n=======================================================\n');
fprintf('  RPO REFERENCE VERIFICATION\n');
fprintf('=======================================================\n');
fails = 0;

%% 1 -- scenario 1 still matches the 3-DOF model
tt = linspace(0, T_scaled, 400);
d  = 0;
for t = tt
    [q_des, q_dot_des] = OrbitTrajectory_ref(t, elements, mu, time_scale, dist_scale);
    [p, ~, V] = EndEffectorTrajectory(t, 1, elements, mu, time_scale, dist_scale, zw, [0 0 0 0]);
    d = max([d, norm(p(1:2)-q_des(1:2)), norm(V(1:2)-q_dot_des(1:2))]);
end
fails = fails + rep('orbit still matches OrbitTrajectory', d, 1e-12, d);

%% 2 -- V-bar: radial identically zero
tt = linspace(0, 100, 400);   r_max = 0;  err = 0;
for t = tt
    [p,~,V] = EndEffectorTrajectory(t, 2, elements, mu, S, D, zw, VB);
    r_max = max(r_max, abs(p(1)));
    err   = max(err, abs(p(2) - (VB(1) - VB(2)*t)));   % against Eq. (24)
end
fails = fails + rep('V-bar radial identically zero', r_max, 1e-14, r_max);
fails = fails + rep('V-bar in-track matches Eq. (24)', err, 1e-12, err);

%% 3 -- R-bar: in-track identically zero
i_max = 0;  err = 0;
for t = tt
    [p,~,~] = EndEffectorTrajectory(t, 3, elements, mu, S, D, zw, RB);
    i_max = max(i_max, abs(p(2)));
    err   = max(err, abs(p(1) - (RB(1) - RB(2)*t)));   % against Eq. (25)
end
fails = fails + rep('R-bar in-track identically zero', i_max, 1e-14, i_max);
fails = fails + rep('R-bar radial matches Eq. (25)', err, 1e-12, err);

%% 4 and 5 -- NMC geometry and CW satisfaction
Tp = 2*pi/n_chief;  tt = linspace(0, Tp, 2000);
ell = 0;  cw = 0;
for k = 1:numel(tt)
    t = tt(k);
    [p,~,V,A] = EndEffectorTrajectory(t, 4, elements, mu, S, D, zw, NMC);
    % The 2:1 axis ratio, tested exactly rather than by sampled maxima: for
    % pos = [rho*cos; -2*rho*sin] the ellipse identity below is an identity
    % at every instant, so it needs no sample to land on a peak.
    ell = max(ell, abs((p(1)/NMC(1))^2 + (p(2)/(2*NMC(1)))^2 - 1));
    % Unforced Clohessy-Wiltshire, in-plane (manuscript Eq. 28):
    %   xddot - 3 n^2 x - 2 n ydot = 0
    %   yddot            + 2 n xdot = 0
    r1 = A(1) - 3*n_chief^2*p(1) - 2*n_chief*V(2);
    r2 = A(2) + 2*n_chief*V(1);
    cw = max(cw, max(abs([r1 r2])));
end
fails = fails + rep('NMC 2:1 ellipse identity (x/rho)^2+(y/2rho)^2=1', ell, 1e-12, ell);
fails = fails + rep('NMC satisfies unforced CW exactly', cw, 1e-12, cw);

%% 5b -- the circle is the e = 0 case of the orbit propagation
% Scenario 5 is written out longhand. Running scenario 1 with zero
% eccentricity must give the same path, which checks both against each other.
els0 = elements; els0(2) = 0;              % same semi-major axis, e = 0
R0   = els0(1)*dist_scale;                 % circular radius (m)
w0   = sqrt(mu/els0(1)^3)*time_scale;      % mean motion x time scale (rad/s)
dc   = 0;
for t = linspace(0, 2*pi/w0, 500)
    [pa,~,Va] = EndEffectorTrajectory(t, 1, els0, mu, time_scale, dist_scale, zw, [0 0 0 0]);
    [pb,~,Vb] = EndEffectorTrajectory(t, 5, elements, mu, 1.0, 1.0, zw, [R0, w0, 0, 0]);
    dc = max([dc, norm(pa(1:2)-pb(1:2)), norm(Va(1:2)-Vb(1:2))]);
end
fails = fails + rep('circle equals the e = 0 orbit', dc, 1e-9, dc);

%% 6 -- vel and acc are the true derivatives
h = 1e-5;  worst_v = 0;  worst_a = 0;
sets = {2, VB, 50; 3, RB, 50; 4, NMC, Tp/3; 1, [0 0 0 0], 5; 5, [1.2, 2*pi/50, 0, 0], 12};
for k = 1:size(sets,1)
    sc = sets{k,1};  pr = sets{k,2};  t0 = sets{k,3};
    Sk = S; Dk = D;
    if sc == 1, Sk = time_scale; Dk = dist_scale; end   % ellipse carries its own
    f = @(tq) getfield_local(tq, sc, elements, mu, Sk, Dk, zw, pr);
    [p0,v0,a0] = f(t0);  %#ok<ASGLU>
    [pp,vp] = f(t0+h);   [pm,vm] = f(t0-h);
    worst_v = max(worst_v, norm((pp-pm)/(2*h) - v0(1:3)));
    worst_a = max(worst_a, norm((vp(1:3)-vm(1:3))/(2*h) - a0(1:3)));
end
fails = fails + rep('V_des is d/dt of p_des', worst_v, 1e-6, worst_v);
fails = fails + rep('A_des is d/dt of V_des', worst_a, 1e-6, worst_a);

%% 7 -- attitude is a unit quaternion and its rate is consistent
qn = 0;  wr = 0;
for t = linspace(0.1, 90, 300)
    [~,u,V] = EndEffectorTrajectory(t, 2, elements, mu, S, D, zw, VB);
    qn = max(qn, abs(norm(u)-1));
    [pp,up] = EndEffectorTrajectory(t+h, 2, elements, mu, S, D, zw, VB); %#ok<ASGLU>
    [~,um]  = EndEffectorTrajectory(t-h, 2, elements, mu, S, D, zw, VB);
    psip = 2*atan2(up(4), up(1));   psim = 2*atan2(um(4), um(1));
    dpsi = atan2(sin(psip-psim), cos(psip-psim))/(2*h);
    wr = max(wr, abs(dpsi - V(6)));
end
fails = fails + rep('u_des is a unit quaternion', qn, 1e-14, qn);
fails = fails + rep('omega_z matches the yaw it implies', wr, 1e-5, wr);

fprintf('\n=======================================================\n');
if fails == 0
    fprintf('  RESULT: PASS\n');
else
    fprintf('  RESULT: FAIL - %d check(s) missed\n', fails);
end
fprintf('=======================================================\n\n');
if fails > 0
    error('verify_scenarios:failed','%d reference check(s) failed.', fails);
end
end


function bad = rep(name, val, tol, shown)
ok = val < tol;
if ok, t = '[OK]'; else, t = '[--]'; end
fprintf('  %s %-42s %.3e\n', t, name, shown);
bad = ~ok;
end


function [p,V,A] = getfield_local(t, sc, elements, mu, S, D, zw, par)
[p,~,V,A] = EndEffectorTrajectory(t, sc, elements, mu, S, D, zw, par);
end
