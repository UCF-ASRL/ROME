function check_guard()
%CHECK_GUARD  Does CommandGuard block what it claims to block?
%   Unit tests on the function, then a model run whose reference deliberately
%   exceeds the wheel ceiling, checking that the guard reports it. Errors on
%   the first failure.

here = fileparts(mfilename('fullpath'));  cd(here);
addpath(here, fullfile(here,'Ground Vehicle'), fullfile(here,'Optitrack'));
evalc('define_constants');          % arm_home, max_rpm, alphas, r, l
home = arm_home(:);
j = [0.1; -0.2; 0.3; 0; 0.7; 0];
nvec = [-1; 1; -1; 1];              % left null vector of the wheel map:
                                    %   any achievable wheel set has nvec'*w = 0

%% 0. soft start ramps the wheel command in over 40 steps (2 s at 20 Hz)
% The arm is NOT ramped: it is already at the commanded pose and ramping it
% would walk it away from there.
clear CommandGuard
wfull = [100; 90; 60; 70];   % satisfies -w1+w2-w3+w4 = 0, i.e. achievable
seen  = zeros(1,3);
for k = 1:40
    [wr, jr, ~] = CommandGuard(wfull, j, 120, home);
    if k == 1,  seen(1) = max(abs(wr));  assert(isequal(jr, j), 'joints ramped'); end
    if k == 20, seen(2) = max(abs(wr)); end
    if k == 40, seen(3) = max(abs(wr)); end
end
assert(abs(seen(1) - 100/40) < 1e-9, 'step 1 should be 1/40 of full, got %.3f', seen(1));
assert(abs(seen(2) - 100/2)  < 1e-9, 'step 20 should be half, got %.3f', seen(2));
assert(abs(seen(3) - 100)    < 1e-9, 'step 40 should be full, got %.3f', seen(3));
[wr, ~, ~] = CommandGuard(wfull, j, 120, home);
assert(abs(max(abs(wr)) - 100) < 1e-9, 'ramp did not stay at full after 40 steps');
assert(abs(nvec.'*wr) < 1e-9, 'ramp broke the wheel consistency condition');
fprintf('  [OK] soft start 1/40 -> half -> full          over 40 steps\n');

%% 1. clean command passes through untouched, once the ramp is complete
clear CommandGuard
for k = 1:40, CommandGuard([10; -20; 30; -40], j, 120, home); end   % finish ramp
[w, jo, f] = CommandGuard([10; -20; 30; -40], j, 120, home);
assert(isequal(w, [10; -20; 30; -40]), 'clean wheel command was altered');
assert(isequal(jo, j), 'clean joint command was altered');
assert(f == 0, 'clean command raised flag %d', f);
fprintf('  [OK] clean command passes through            flag 0\n');

%% 2. over-limit command is scaled as a whole, direction preserved
wi = [200; -300; 10; 119.9];
[w, ~, f] = CommandGuard(wi, j, 120, home);
assert(max(abs(w)) <= 120 + 1e-12, 'not limited: %s', mat2str(w.', 4));
assert(abs(max(abs(w)) - 120) < 1e-9, 'did not scale to the ceiling: %s', mat2str(w.', 4));
assert(norm(cross3(w, wi)) < 1e-9, 'direction changed: %s vs %s', ...
       mat2str(w.', 4), mat2str(wi.', 4));
assert(f == 1, 'scaling raised flag %d, expected 1', f);
fprintf('  [OK] over-limit command scaled to the ceiling flag 1, peak %.1f rpm\n', max(abs(w)));

%% 2b. scaling preserves the wheel consistency condition; clipping does not
wa = [100; -80; 60; -40];  wa(4) = wa(1) - wa(2) + wa(3);   % make it achievable
assert(abs(nvec.'*wa) < 1e-9, 'test vector is not achievable');
[w, ~, ~] = CommandGuard(wa*4, j, 120, home);               % 4x over the ceiling
clipped = min(max(wa*4, -120), 120);
fprintf('  [OK] scaled  |n''w| = %.2e rpm   (clipping the same command: %.1f rpm)\n', ...
        abs(nvec.'*w), abs(nvec.'*clipped));
assert(abs(nvec.'*w) < 1e-9, 'scaling broke the consistency condition');
assert(abs(nvec.'*clipped) > 1, 'clipping was expected to break it');

%% 3. non-finite wheel: base stopped, last finite joint command held
[w, jo, f] = CommandGuard([1; NaN; 3; 4], j + 0.5, 120, home);
%#ok<*NASGU>
assert(isequal(w, zeros(4,1)), 'wheels not stopped on NaN: %s', mat2str(w.'));
assert(isequal(jo, j), 'joints not held on NaN');          % the hold from case 2
assert(f == 2, 'NaN raised flag %d, expected 2', f);
fprintf('  [OK] NaN blocked, base stopped, arm held     flag 2\n');

%% 4. infinite joint is blocked the same way
[w, jo, f] = CommandGuard([1; 2; 3; 4], [Inf; 0; 0; 0; 0; 0], 120, home);
assert(isequal(w, zeros(4,1)) && isequal(jo, j) && f == 2, 'Inf joint not blocked');
fprintf('  [OK] Inf joint blocked                       flag 2\n');

%% 4b. before any finite command, the hold is arm_home, not zeros
clear CommandGuard
[w, jo, f] = CommandGuard([NaN; 0; 0; 0], j, 120, home);
assert(isequal(jo, home), 'first-call hold was %s, expected arm_home', mat2str(jo.', 3));
assert(isequal(w, zeros(4,1)) && f == 2, 'first-call block failed');
fprintf('  [OK] first command non-finite: arm held at home, not 0 rad\n');

%% 5. nothing non-finite ever leaves the guard, over random input
clear CommandGuard
rng(7);  worst = 0;
for k = 1:2000
    wi = 1e3*randn(4,1);  ji = randn(6,1);
    if mod(k, 7) == 0,  wi(randi(4)) = NaN;  end
    if mod(k, 11) == 0, ji(randi(6)) = Inf;  end
    [w, jo, ~] = CommandGuard(wi, ji, 120, home);
    assert(all(isfinite(w)) && all(isfinite(jo)), 'non-finite output at draw %d', k);
    worst = max(worst, max(abs(w)));
end
assert(worst <= 120 + 1e-9, 'output exceeded the ceiling: %.12f', worst);
fprintf('  [OK] 2000 random draws, none non-finite      max |rpm| %.1f\n', worst);

%% 5b. joint cap: firmware degrees, each joint to its own range, margin inside
sw = arm_fw_switch;  ot = arm_fw_other;  mg = arm_fw_margin_deg;
lo = min(sw, ot);  hi = max(sw, ot);
assert(isequal(sw, [-180 132 141 -165 90 180]) && isequal(ot, [160 0 1 165 -90 -170]), ...
       'arm_fw_switch / arm_fw_other no longer match limits[] / otherLimits[] in ROME_Teensy_Code.ino');
[d, hit] = arm_clamp([0 90 90 1 0 0], sw, ot, mg);            % firmware home, firmware order
assert(isequal(d, [0 90 90 1 0 0]) && ~any(hit), 'firmware home was altered');
[d, hit] = arm_clamp([-200 -53 150 0 95 0], sw, ot, mg);      % outside on 1,2,3,5
assert(isequal(d, [-180+mg 0+mg 141-mg 0 90-mg 0]), 'cap gave %s', mat2str(d));
assert(isequal(hit, logical([1 1 1 0 1 0])), 'hit flags %s', mat2str(hit));
[d, hit] = arm_clamp([160 132 141 165 90 180], ot, sw, mg);   % exactly on the ends, arrays swapped
assert(all(hit) && isequal(d, hi - mg), 'ends not pulled in by the margin: %s', mat2str(d));
[d, hit] = arm_clamp(lo, sw, ot, mg);                         % negative ends too
assert(all(hit) && isequal(d, lo + mg), 'negative ends not pulled in: %s', mat2str(d));
m2f = arm_sign .* rad2deg(arm_home(:).') + arm_offset_deg;    % the start pose, mapped
[~, hit] = arm_clamp(m2f, sw, ot, mg);
assert(~any(hit), 'arm_home maps outside the firmware range: %s', mat2str(round(m2f)));
fprintf('  [OK] joint cap: margin %g deg inside both ends   arm_home -> fw %s\n', mg, mat2str(round(m2f)));

%% 6. in the model: a reference that needs more than the ceiling
orig = claim_constants();                  % see the function, foot of file
restore = onCleanup(@() release_constants(orig));
wf('define_constants.m', regexprep(orig, 'size_factor\s*=\s*[0-9.]+;', 'size_factor = 4.0;', 'once'));
if bdIsLoaded('ROME_9DOF'), close_system('ROME_9DOF', 0); end
evalin('base', 'clear guard_flag');
load_system(fullfile(here, 'ROME_9DOF.slx'));
set_param('ROME_9DOF', 'StopFcn', '');
out = sim('ROME_9DOF', 'SimulationMode', 'normal', 'ReturnWorkspaceOutputs', 'on');
close_system('ROME_9DOF', 0);
gf = squeeze(out.guard_flag.Data);
w  = squeeze(out.wheel_speeds_rpm.Data);
mr = evalin('base','max_rpm');
assert(max(abs(w(:))) > mr, 'this case was meant to exceed the ceiling %g, peak was %.1f', mr, max(abs(w(:))));
assert(any(gf == 1), 'guard never reported saturation while the solve asked for %.1f rpm', max(abs(w(:))));
fprintf('  [OK] model run at size_factor = 4            solve asked %.1f rpm, guard saturated on %d of %d steps\n', ...
        max(abs(w(:))), sum(gf == 1), numel(gf));

fprintf('\n  CommandGuard: all checks passed.\n');
end

function wf(p, s)
f = fopen(p, 'w');  fwrite(f, s);  fclose(f);
end

function c = cross3(a, b)
% Collinearity test for two 4-vectors: the component of a orthogonal to b.
a = a(:);  b = b(:);
c = a - b*((b.'*a)/(b.'*b));
end


function orig = claim_constants()
% Take custody of define_constants.m, leaving a pristine copy on disk.
%
% define_constants.m is a tracked source file, and this harness rewrites it.
% onCleanup restores it when the function returns or errors, but a hard kill
% -- the process killed under memory pressure, a crash of the LXE engine --
% skips onCleanup entirely and leaves the file modified with nothing to record
% that fact. The next run would then read the wrong scenario, or the wrong
% dist_scale, and report a result for a case nobody asked for.
%
% So: the unmodified text is written to define_constants.PRISTINE before the
% first rewrite, and that file's existence is the signal that a previous run
% did not finish. Finding one here is refused rather than repaired, because
% the pristine copy is the only record of what the file looked like, and
% overwriting it silently would destroy that record.
lock = 'define_constants.PRISTINE';
if isfile(lock)
    error('constants:interrupted', ...
        ['a previous run was interrupted and left define_constants.m ' ...
         'modified.\n  The unmodified text is in %s.\n  Restore it with:  ' ...
         'copyfile(''%s'', ''define_constants.m''); delete(''%s'')\n  ' ...
         'Compare them first if you have edited define_constants.m since.'], ...
        lock, lock, lock);
end
orig = fileread('define_constants.m');
wf(lock, orig);
end

function release_constants(orig)
% Put define_constants.m back, then drop the pristine copy -- in that order, so
% an interruption between the two still leaves the record behind.
wf('define_constants.m', orig);
lock = 'define_constants.PRISTINE';
if isfile(lock), delete(lock); end
end
