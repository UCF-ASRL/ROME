function test_play_run_compare()
%TEST_PLAY_RUN_COMPARE  Not registered in test_suite. Run: matlab -batch "addpath(<RM>/viewer,<RM>/analysis,<RM>); cd(<RM>); addpath(<RM>/tests/viewer); test_play_run_compare"
%   1  the chain rebuilt from the source tables (viewer_chain_check) has the same tip as ik9_fk and ar3_fk, 1e-12,
%      at 5 random configurations; also P(:,end) of both functions equals their tip.
%   2  q = 0: arms B and C differ by the map difference (printed). It is NOT only J2: J5 and J6 differ too.
%   3  headless run on runs/case_1.mat writes the requested PNGs.
%   4  make-it-fail: a temp copy of viewer_chain_check.m with one link offset dropped must FAIL check 1.
rmDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(rmDir, fullfile(rmDir, 'analysis'), fullfile(rmDir, 'viewer'));
ok = true;
fprintf('--- check 1: chain tips, 5 random configurations\n');
[pass1, e1] = check1(rmDir);
fprintf('max |tip difference| = %.3g (limit 1e-12)  -> %s\n', e1, passstr(pass1));
ok = ok && pass1;

fprintf('--- check 2: q = 0, the two maps\n');
[sg, off] = viewer_arm_map(rmDir);
fprintf('arm_sign %s  arm_offset_deg %s  (parsed from lab_settings.m text)\n', mat2str(sg'), mat2str(off'));
pass2a = isequal(sg', [1 -1 -1 1 1 1]) && isequal(off', [0 180 90 0 0 0]);
fprintf('parsed map equals the values read on 19 Nov (L121-122): %s\n', passstr(pass2a));
fw0 = sg .* 0 + off;
qB = romefw_to_arcs_deg(fw0, 'rome_dirpin');  qC = romefw_to_arcs_deg(fw0, 'arcs_caldir');
dq = qB - qC;
fprintf('ARCS deg, rome_dirpin: %s\nARCS deg, arcs_caldir: %s\nB minus C: %s\n', mat2str(qB', 6), mat2str(qC', 6), mat2str(dq', 6));
pass2b = max(abs(dq' - [0 225.6 0 0 28.3 63.8])) < 1e-12;   % hand arithmetic from the table: J2 48-(-177.6), J5 14.15-(-14.15), J6 31.9-(-31.9)
fprintf('difference = [0 225.6 0 0 28.3 63.8] (J2, J5 and J6 differ; J1, J3, J4 agree): %s\n', passstr(pass2b));
[~, PB] = ar3_fk(qB);  [~, PC] = ar3_fk(qC);
fprintf('AR3 tip at q=0: B %s  C %s  |B-C| = %.4f m\n', mat2str(PB(:,end)', 4), mat2str(PC(:,end)', 4), norm(PB(:,end)-PC(:,end)));
pass2 = pass2a && pass2b;  ok = ok && pass2;

fprintf('--- check 3: headless run on case_1.mat\n');
d = fullfile(rmDir, '..', 'docs', 'verification', 'viewer_frames');   % kept: these are the deliverable PNGs
fr = [1 156 311 466 621];
o = play_run_compare(fullfile(rmDir, 'runs', 'case_1.mat'), struct('frames', fr, 'outDir', d, 'visible', false));
L = dir(fullfile(d, '*.png'));
pass3 = numel(L) == numel(fr) && all([L.bytes] > 1000) && numel(o.dAB) == numel(fr) && all(isfinite([o.dAB o.dAC o.dBC]));
fprintf('%d PNG written (%s) in %s, sizes %s bytes -> %s\n', numel(L), strjoin({L.name}, ', '), d, mat2str([L.bytes]), passstr(pass3));
ok = ok && pass3;

fprintf('--- check 4: make-it-fail (temp copy of the helper with the d4 link offset dropped)\n');
tmp = fullfile(tempdir, ['viewer_break_' char(java.util.UUID.randomUUID)]);  mkdir(tmp);
s = fileread(fullfile(rmDir, 'viewer', 'viewer_chain_check.m'));
tag = '0 sin(al) cos(al) dh(i,3); 0 0 0 1];';
assert(contains(s, tag), 'break target missing in the helper');
s = strrep(s, tag, '0 sin(al) cos(al) 0; 0 0 0 1];');
fid = fopen(fullfile(tmp, 'viewer_chain_check.m'), 'w');  fwrite(fid, s);  fclose(fid);
addpath(tmp, '-begin');  rehash;
[pass1b, e1b] = check1(rmDir);
rmpath(tmp);  rehash;
fprintf('with the break: max |tip difference| = %.3g, check 1 %s (it must FAIL) -> %s\n', e1b, passstr(pass1b), passstr(~pass1b));
ok = ok && ~pass1b;
if ok, fprintf('TEST_PLAY_RUN_COMPARE: ALL PASS\n'); else, error('test_play_run_compare:fail', 'TEST_PLAY_RUN_COMPARE: FAIL'); end
end

function [pass, emax] = check1(rmDir)
rng(7);  emax = 0;
for n = 1:5
    q9 = [randn(2,1); 2*pi*rand-pi; (rand(6,1)-0.5)*2*pi];   % x y yaw J1..J6
    qa = (rand(6,1)-0.5)*200;                                 % ARCS deg
    [pe, ~, Pm] = ik9_fk(q9);  [T06, Pa] = ar3_fk(qa);
    [tm, ta] = viewer_chain_check(q9, qa, rmDir);
    Rb = [cos(q9(3)) sin(q9(3)) 0; -sin(q9(3)) cos(q9(3)) 0; 0 0 1];
    emax = max([emax, norm(tm - pe), norm(ta - T06(1:3,4)), norm(Pm(:,end) - Rb*(pe-[q9(1);q9(2);0])), norm(Pa(:,end) - T06(1:3,4))]);
end
pass = emax < 1e-12;
end

function s = passstr(b)
if b, s = 'PASS'; else, s = 'FAIL'; end
end
