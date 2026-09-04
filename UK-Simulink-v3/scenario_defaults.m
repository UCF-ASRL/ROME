function [par, D, S, tf, name] = scenario_defaults(scenario)
%SCENARIO_DEFAULTS  Parameters and tabletop scaling for each reference.
%
%   Scenario parameters are the manuscript's, unchanged. The scale factors
%   are what carries an operational trajectory onto a 4 m floor: the
%   references run to 100 m over one chief orbit, and ROME works in metres
%   over tens of seconds.
%
%   D and S were chosen so the commanded path spans about 1.2 m and the run
%   lasts about 50 s, then confirmed against the wheel-speed ceiling with
%   check_9dof. The shape is preserved exactly under this similarity; only
%   the size and the clock change.
%
%     par   1x4  scenario parameters, at the manuscript's own scale
%     D     1x1  dist_scale
%     S     1x1  time_scale
%     tf    1x1  run length on the table (s)
%     name  char label for reporting
%
%   MANUSCRIPT VALUES, Table of simulation parameters
%     V-bar  y0  = 100 m,   v0  = 0.50 m/s,  horizon 100 s
%     R-bar  x0  =  50 m,   v0  = 0.25 m/s,  horizon 100 s
%     NMC    rho =  50 m,   phi = 0,         horizon T = 2*pi/n
%     chief mean motion n = 1.078e-3 rad/s, from a circular orbit at
%     a = 7000 km. Chief period T = 5.83e3 s.

n_chief = 1.078e-3;             % chief mean motion (rad/s)

switch scenario
    case 1
        % The scaled elliptical orbit already flown by the 3-DOF model. The
        % orbital elements and both scale factors are the values that model
        % uses, so selecting scenario 1 reproduces its commanded path.
        par  = [0 0 0 0];
        D    = 1;
        S    = 0.40;
        tf   = 2*pi*sqrt(1.4^3/1.0)/0.40;
        name = 'orbit (elliptical)';

    case 2
        % V-bar approach. 100 m of in-track closing becomes 1.2 m of table.
        par  = [100, 0.50, 0, 0];       % [y0 v0 . .]
        D    = 0.012;                   % 100 m -> 1.20 m
        S    = 2.0;                     % 100 s -> 50 s
        tf   = 100/S;
        name = 'V-bar';

    case 3
        % R-bar approach. 50 m of radial descent becomes 1.2 m.
        par  = [50, 0.25, 0, 0];        % [x0 v0 . .]
        D    = 0.024;                   % 50 m -> 1.20 m
        S    = 2.0;                     % 100 s -> 50 s
        tf   = 100/S;
        name = 'R-bar';

    case 4
        % Natural-motion circumnavigation. The in-track semi-axis is 2*rho,
        % so 2*50 m sets the extent: 100 m -> 1.2 m. One chief period of
        % 5.83e3 s is compressed to about 50 s.
        par  = [50, n_chief, 0, 0];     % [rho n phi .]
        D    = 0.012;                   % 2*rho = 100 m -> 1.20 m
        S    = (2*pi/n_chief)/50;       % one full period in 50 s
        tf   = (2*pi/n_chief)/S;
        name = 'NMC';

    case 5
        % Circular path: the e = 0 special case, kept as the simplest
        % possible reference. R = 1.2 m at 2*pi/50 rad/s is one revolution in
        % 50 s, matching the horizon of the RPO cases.
        par  = [1.2, 2*pi/50, 0, 0];    % [R w . .]
        D    = 1.0;
        S    = 1.0;
        tf   = 50;
        name = 'circle';

    otherwise
        error('scenario_defaults:unknown', ...
              ['scenario must be 1 (elliptical orbit), 2 (V-bar), ' ...
               '3 (R-bar), 4 (NMC) or 5 (circle).']);
end
end
