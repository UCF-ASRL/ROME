function [zmin, sdmin, sdf] = link_clearance(Q, hx, hy)
%LINK_CLEARANCE  Lowest link point and distance to the base square, for one arm posture.
%   [zmin, sdmin, sdf] = link_clearance(Q, hx, hy)
%     Q      3x7 origins of arm frames 0..6 in the base body frame (m); z of the body frame = world z
%            (planar base at z = 0)
%     hx,hy  half sides of the base rectangle, body x and y (m): d/2, w/2
%     zmin   lowest z over the segments shoulder (frame 1) to flange (frame 6), 21 points each (m);
%            the J1 column (frame 0 to 1) is left out
%     sdmin  smallest SIGNED horizontal distance to the rectangle over the segments from the elbow
%            (frame 2) to the flange (m): positive outside, negative inside (depth). The J1 column, the
%            shoulder and the upper arm are left out: the upper arm's shoulder end sits over the base by
%            construction, so the unsigned distance would be 0 on every sample and could not fail.
%     sdf    the same signed distance for the flange alone (m)
%   The one definition of these metrics: used by clearance_9dof (gate) and analysis/commanding_check.
sgrid = linspace(0, 1, 21);                 % 1x21 sample fractions along each link segment (-)
zmin = inf;  sdmin = inf;                   % lowest z (m), smallest signed distance (m)
for i = 2:6                                 % segment frame i-1 -> frame i (columns i, i+1)
    X = Q(:, i) + (Q(:, i+1) - Q(:, i)) * sgrid;   % 3x21 points on the segment (m)
    zmin = min(zmin, min(X(3, :)));
    if i >= 3                               % elbow (frame 2) onwards
        dx = abs(X(1, :)) - hx;  dy = abs(X(2, :)) - hy;   % 1x21 excess over the half sides (m)
        sd = sqrt(max(dx, 0).^2 + max(dy, 0).^2) + min(max(dx, dy), 0);   % signed distance (m)
        sdmin = min(sdmin, min(sd));
    end
end
dxf = abs(Q(1, 7)) - hx;  dyf = abs(Q(2, 7)) - hy;   % flange excess over the half sides (m)
sdf = sqrt(max(dxf, 0)^2 + max(dyf, 0)^2) + min(max(dxf, dyf), 0);   % flange signed distance (m)
end
