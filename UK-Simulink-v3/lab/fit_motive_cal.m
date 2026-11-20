function cal = fit_motive_cal(recs, marks, opts)
%FIT_MOTIVE_CAL  Least-squares Motive -> floor transform (rotation, translation, scale) from taped marks.
%   cal = FIT_MOTIVE_CAL(recs, marks)
%   cal = FIT_MOTIVE_CAL(recs, marks, opts)
%     recs   cell array (or struct array), one record_motive recording per mark, in the same order as
%            the rows of marks. The rigid body sits still on the mark while it is recorded; the mean
%            of its accepted frames is the mark's Motive position (ok frames only, rec.ok).
%     marks  table, one row per mark, with variables
%              x        floor x of the rigid-body pivot (m)
%              y        floor y of the pivot (m)
%              heading  floor heading of the body x axis (rad)
%              z        OPTIONAL, pivot height above the floor (m); default 0. Only the translation
%                       z component depends on it; the floor-plane pose does not.
%     opts   optional struct, both fields PROVISIONAL and UNMEASURED ([Claude's call, author to confirm], TRACKER):
%              sigma_mark_m  0.005  uncertainty of a taped mark's floor coordinates (m) (tape placement, Plan 3 Task 1 brief)
%              k             3      fit_ok when rms_pos_m <= k * rms_expected_m
%   Model: floor = scale * R * p_motive + t   (p_motive 3x1 in Motive units).
%   Method: Procrustes / Kabsch with scale, closed form: S. Umeyama, "Least-squares estimation of
%   transformation parameters between two point patterns", IEEE Trans. Pattern Anal. Mach. Intell.
%   13(4):376-380, April 1991, DOI 10.1109/34.88573. Citation checked against Crossref
%   (api.crossref.org/works/10.1109/34.88573, 19 Nov 2026): title, author S. Umeyama, journal, volume 13,
%   issue 4, pages 376-380, issued 1991-04. The paper's own text was NOT opened (paywalled); the
%   algorithm below is written from the formulas as I know them and is checked by tests/test_lab_frame.m
%   against synthetic data with known R, t, scale, not against the paper.
%   cal fields (the three cal-contract fields of Optitrack/motive_to_floor.m plus the measured flag, then
%   diagnostics). There is no yaw_axis field: the yaw axis is the floor z axis by the floor-frame convention,
%   and motive_to_floor is the only conversion (F5a, 19 Nov 2026):
%     R          3x3 rotation, Motive frame -> floor frame (proper, det = +1)
%     t          3x1 m, floor-frame position of the Motive origin
%     scale      m per Motive unit
%     measured   ALWAYS false. A fit does not make a transform measured: only the author's sign-off after
%                the Part C visit sets it (lab_signoff / define_constants, Plan 3 Task 4).
%     fit_ok     true when the residual gate passes: rms_pos_m <= k * rms_expected_m, with
%                rms_expected_m = sqrt(sigma_mocap_m^2 + sigma_mark_m^2) * sqrt((3n - 7)/n) (7 fitted parameters;
%                sigma_mocap_m = scale * rms over marks of (per-mark std / sqrt(frames)))
%     fit_gate_m, rms_expected_m, sigma_mocap_m, sigma_mark_m, k   the gate and its inputs
%     n          number of marks
%     pos_resid_m      n x 1, |floor mark - transformed mark| per mark (m)
%     rms_pos_m, max_pos_m   root-mean-square and maximum of pos_resid_m (m)
%     heading_resid_rad  n x 1, (body heading from the quaternion, through R) - mark heading, wrapped (rad)
%     psi_offset_rad     circular mean of heading_resid_rad (rad). Not zero if the rigid body's x axis
%                        is not the chassis x axis the heading line was taped for; it is reported, not removed.
%     mark_std_motive    n x 1, per-mark standard deviation of the recorded position, rms over x y z (Motive units)
%     n_frames           n x 1, accepted frames used per mark
%   Needs at least 3 non-collinear marks; warns below 6 (the lab-visit checklist asks for 6).
%   No network, no model, no define_constants.
if nargin < 3 || isempty(opts)
    opts = struct();
end
if ~isfield(opts, 'sigma_mark_m'), opts.sigma_mark_m = 0.005; end   % mark coordinate uncertainty (m), provisional
if ~isfield(opts, 'k'), opts.k = 3; end                             % gate factor (-), provisional
if iscell(recs)
    recC = recs(:).';                       % recordings as a 1xn cell
elseif isstruct(recs)
    recC = num2cell(recs(:).');
else
    error('fit_motive_cal:recs', 'recs must be a cell array or a struct array of recordings');
end
if ~istable(marks)
    error('fit_motive_cal:marks', 'marks must be a table');
end
vn = marks.Properties.VariableNames;        % variable names of the marks table (cellstr)
for k = {'x', 'y', 'heading'}
    if ~any(strcmp(vn, k{1}))
        error('fit_motive_cal:marks', 'marks table has no variable "%s" (needs x, y, heading; z optional)', k{1});
    end
end
n = numel(recC);                            % number of marks (count)
if height(marks) ~= n
    error('fit_motive_cal:count', 'marks has %d rows but recs has %d recordings', height(marks), n);
end
if n < 3
    error('fit_motive_cal:fewMarks', 'need at least 3 marks, got %d', n);
end
if n < 6
    warning('fit_motive_cal:fewMarks', 'only %d marks; the lab checklist asks for 6 well-spread marks', n);
end
mx = marks.x(:).';                          % mark floor x (m), 1xn
my = marks.y(:).';                          % mark floor y (m), 1xn
if any(strcmp(vn, 'z'))
    mz = marks.z(:).';                      % mark pivot height above floor (m), 1xn
else
    mz = zeros(1, n);
end
mh = marks.heading(:).';                    % mark floor heading (rad), 1xn
F = [mx; my; mz];                           % target points, floor frame (m), 3xn
if ~all(isfinite(F(:))) || ~all(isfinite(mh))
    error('fit_motive_cal:marks', 'marks contain a non-finite value');
end
P = zeros(3, n);                            % source points, mean Motive position of each recording (Motive units), 3xn
nFrames = zeros(n, 1);                      % accepted frames per recording (count)
markStd = zeros(n, 1);                      % per-mark position std, rms over xyz (Motive units)
for i = 1:n
    r = recC{i};                            % recording i (struct)
    ok = logical(r.ok(:).');                % accepted frames (1xN logical)
    nFrames(i) = nnz(ok);
    if nFrames(i) < 1
        error('fit_motive_cal:noFrames', 'recording %d has no accepted frame (rec.ok all false)', i);
    end
    pp = r.pose7(1:3, ok);                  % accepted positions (Motive units), 3xM
    if ~all(isfinite(pp(:)))
        error('fit_motive_cal:nonfinite', 'recording %d has non-finite positions in accepted frames', i);
    end
    P(:, i) = mean(pp, 2);
    if nFrames(i) > 1
        markStd(i) = sqrt(mean(var(pp, 0, 2)));
    end
end
% degeneracy: the marks must span a plane in both frames
sF = svd(F - mean(F, 2));                   % singular values of the centred floor marks (m)
sP = svd(P - mean(P, 2));                   % singular values of the centred Motive marks (Motive units)
if sF(2) < 1e-3 * sF(1) || sP(2) < 1e-3 * sP(1)
    error('fit_motive_cal:collinear', 'marks are (nearly) collinear: singular values floor %s, Motive %s', ...
          mat2str(sF.', 4), mat2str(sP.', 4));
end
% --- Umeyama closed form -------------------------------------------------------------------------
muP = mean(P, 2);                           % centroid of the Motive points (Motive units), 3x1
muF = mean(F, 2);                           % centroid of the floor points (m), 3x1
Pc = P - muP;                               % centred Motive points (Motive units)
Fc = F - muF;                               % centred floor points (m)
varP = sum(Pc(:).^2) / n;                   % (1/n) sum |p_i - muP|^2 (Motive units^2)
Sigma = (Fc * Pc.') / n;                    % cross-covariance, floor (rows) by Motive (columns), 3x3
[U, D, V] = svd(Sigma);                     % Sigma = U D V'
Sg = eye(3);                                % reflection guard
Sg(3, 3) = sign(det(U * V.'));              % -1 when U*V' is a reflection, so that R is a proper rotation
if Sg(3, 3) == 0
    Sg(3, 3) = 1;
end
R = U * Sg * V.';                           % rotation Motive -> floor (3x3)
scale = trace(D * Sg) / varP;               % m per Motive unit
t = muF - scale * R * muP;                  % floor position of the Motive origin (m), 3x1
if ~(scale > 0) || abs(det(R) - 1) > 1e-9 || norm(R.' * R - eye(3), 'fro') > 1e-9
    error('fit_motive_cal:fit', 'fit failed sanity: scale %g, det(R) %g', scale, det(R));
end
cal = struct('R', R, 't', t, 'scale', scale, 'measured', false);
% --- residuals -----------------------------------------------------------------------------------
Fhat = scale * (R * P) + t;                 % transformed marks (m), 3xn
res = vecnorm(F - Fhat, 2, 1).';            % position residual per mark (m), n x 1
hres = zeros(n, 1);                         % heading residual per mark (rad)
for i = 1:n
    flI = lab_rec_to_floor(recC{i}, cal);   % floor [x; y; psi] of the accepted frames of recording i
    psiI = angle(mean(exp(1i * flI(3, :))));% circular mean heading of recording i (rad)
    hres(i) = angle(exp(1i * (psiI - mh(i))));   % wrapped to (-pi, pi]
end
cal.n = n;
sigMocap = scale * sqrt(mean(markStd.^2 ./ nFrames));   % noise of a mark's mean position, per coordinate (m)
rmsExp = sqrt(sigMocap^2 + opts.sigma_mark_m^2) * sqrt(max(3*n - 7, 1) / n);   % expected rms residual (m)
cal.sigma_mocap_m = sigMocap;
cal.sigma_mark_m = opts.sigma_mark_m;
cal.k = opts.k;
cal.rms_expected_m = rmsExp;
cal.fit_gate_m = opts.k * rmsExp;
cal.pos_resid_m = res;
cal.rms_pos_m = sqrt(mean(res.^2));
cal.max_pos_m = max(res);
cal.heading_resid_rad = hres;
cal.psi_offset_rad = angle(mean(exp(1i * hres)));
cal.mark_std_motive = markStd;
cal.fit_ok = cal.rms_pos_m <= cal.fit_gate_m && all(isfinite(res));
cal.n_frames = nFrames;
cal.method = 'Umeyama 1991, IEEE TPAMI 13(4):376-380, DOI 10.1109/34.88573 (citation checked at Crossref 19 Nov 2026; paper text not opened)';
end
