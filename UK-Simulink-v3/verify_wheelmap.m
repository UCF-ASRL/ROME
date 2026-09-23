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
% The BLOCK's own wheel map, executed from the block's own source text.
%
% ukd_current.m is the verbatim copy of the UKDynamics chart. Its rome_wheels
% local function is extracted here and written to a temporary file so it can
% be called directly, with the workspace alphas. Re-typing the formula in this
% file instead would compare the file with itself and could not catch a wrong
% alpha ORDER -- the failure this check exists for, which sends each speed to
% the diagonally opposite motor.
%
% rome_wheels is called rather than ukd_current as a whole because the block
% reports speeds for its INTEGRATED rate qd_int, already advanced by qdd*dt;
% that would compare two different velocities.
evalc('define_constants');
src = fileread('ukd_current.m');
src = strrep(src, char(13), '');   % CRLF-safe before searching
i0  = strfind(src, 'function w = rome_wheels(');
assert(~isempty(i0), 'verify_wheelmap:nofn', 'rome_wheels not found in ukd_current.m');
tailsrc = src(i0:end);
i1  = strfind(tailsrc, sprintf('\nend\n'));
body = tailsrc(1:i1(1)+4);                    % up to and including its end
tmp  = fullfile(tempdir, 'blk_wheelmap_probe');
if ~isfolder(tmp), mkdir(tmp); end
fid = fopen(fullfile(tmp, 'rome_wheels.m'), 'w');  fwrite(fid, body);  fclose(fid);
addpath(tmp);  clean = onCleanup(@() rmpath(tmp));
rehash path
w = rome_wheels([0; 0; th], qd(:), r, l, alphas(:)) * (60/(2*pi));
end

