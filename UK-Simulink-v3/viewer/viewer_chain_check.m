function [tipModel, tipAR3] = viewer_chain_check(q9, qArcsDeg, rmDir)
%VIEWER_CHAIN_CHECK  Rebuild both arm chains from the SAME tables the functions use, return the two tips.
%   The tables are read from the text of ik9_fk.m and analysis/ar3_fk.m (nothing copied by hand), the chain
%   product is written again here. test_play_run_compare compares these tips with ik9_fk and ar3_fk (1e-12).
%   q9         9x1 [x y yaw J1..J6] (m, rad);  qArcsDeg 6x1 ARCS angles (deg)
%   tipModel   3x1 world tip of the model chain;  tipAR3 3x1 tip in the AR3 base frame
if nargin < 3 || isempty(rmDir), rmDir = fileparts(fileparts(mfilename('fullpath'))); end
tx = fileread(fullfile(rmDir, 'ik9_fk.m'));
tk = regexp(tx, 'dh\s*=\s*(\[[^\]]*\]);', 'tokens', 'once');
dh = str2num(tk{1}); %#ok<ST2NM> 6x4 [a alpha d offset]
tk = regexp(tx, 'T=\[cos\(th\)[^\]]*?0 0 1 ([0-9.]+);', 'tokens', 'once');
h0 = str2double(tk{1});   % mount height (m)
q9 = q9(:);  th = q9(3);
T = [cos(th) -sin(th) 0 q9(1); sin(th) cos(th) 0 q9(2); 0 0 1 h0; 0 0 0 1];
for i = 1:6
    ti = q9(3+i) + dh(i,4);  al = dh(i,2);
    T = T * [cos(ti) -sin(ti)*cos(al)  sin(ti)*sin(al) dh(i,1)*cos(ti); ...
             sin(ti)  cos(ti)*cos(al) -cos(ti)*sin(al) dh(i,1)*sin(ti); ...
             0 sin(al) cos(al) dh(i,3); 0 0 0 1];
end
tipModel = T(1:3,4);
ta = fileread(fullfile(rmDir, 'analysis', 'ar3_fk.m'));
alpha = vtab(ta, 'alpha_deg');  a = vtab(ta, 'a_m');  d = vtab(ta, 'd_m');  off = vtab(ta, 'theta_off_deg');
T = eye(4);
for i = 1:6
    t = deg2rad(qArcsDeg(i) + off(i));  al = deg2rad(alpha(i));
    T = T * [cos(t) -sin(t)*cos(al)  sin(t)*sin(al) a(i)*cos(t); ...
             sin(t)  cos(t)*cos(al) -cos(t)*sin(al) a(i)*sin(t); ...
             0 sin(al) cos(al) d(i); 0 0 0 1];
end
tipAR3 = T(1:3,4);
end

function v = vtab(txt, name)
tok = regexp(txt, ['^\s*' name '\s*=\s*([^;]*);'], 'tokens', 'lineanchors', 'once');
assert(~isempty(tok), 'viewer_chain_check:parse', 'table %s not found', name);
v = str2num(tok{1}); %#ok<ST2NM>
assert(numel(v) == 6, 'viewer_chain_check:parse', 'table %s has %d entries', name, numel(v));
end
