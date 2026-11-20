function test_lab_frame(noiseScale)
%TEST_LAB_FRAME  The lab-frame kit (Plan 3 Task 1): fit_motive_cal, frame_check, record_motive.
%   NO NETWORK, no robot, no model, no define_constants. All data are synthetic (fixed RandStream seed).
%   test_lab_frame(noiseScale) multiplies the synthetic noise of check i only (default 1); the
%   make-it-fail run is test_lab_frame(100), which must make check i FAIL. Source-level mutations (a copy of one
%   lab file with one count-asserted change, put first on the path) are listed in the Plan 3 Task 1 report.
%     0   helpers: the quaternion helpers agree with Rodrigues' formula (independent of the code under test)
%     i   fit_motive_cal on 6 marks x 20 frames with 0.3 mm noise per frame and coordinate, two truths
%         (A: Z-up, yaw 37 deg + 0.5 deg tilt, scale 1, no z column; B: Y-up, yaw 37 deg, mm units
%         scale 0.001, z column): R within tolR, t within tolT, scale within tolS, where each tolerance
%         is 3 x the noise effect for 6 marks computed here (sqrt(3) for the norm of a 3-vector), capped
%         at the brief's 0.1 deg and 1 mm. Heading residuals within tolR. Output fields present; measured is
%         false and fit_ok true. Truth C is chosen in the test so that the UNGUARDED Umeyama SVD gives
%         det(U V') = -1 (a reflection): this exercises the reflection guard. Bad fits (a mark 20 mm / 100 mm
%         wrong, marks in another order than the recordings) give fit_ok = false (detected, not refused).
%     ii  frame_check on a synthetic push recording (4 rests, +0.5 m x, +0.5 m y, +90 deg):
%         PASS; FAIL with y flipped; FAIL/PASS either side of the 10 mm and 2 deg bounds;
%         FAIL with the rotation sign flipped, y +15 mm, base moving 30 mm during the rotation, only 3 rests;
%         PASS with a cal obtained from fit_motive_cal (A); refuses a cal that is neither fit_ok nor measured;
%         FAIL with a cal whose R is transposed.
%     iii record_motive on a fake transport writes the file with the exact bytes and poses; a second
%         call with the same name errors and leaves the file and the transport untouched; a real socket is
%         opened only with opts.live = true (a shadow MotiveUdpTransport that errors on construction shows the
%         injected path never builds it and the live path does).
%     iv  fit_motive_cal refuses collinear marks and a count mismatch.
%   Errors if any check fails; every check is printed first. Flat, no local functions (CLAUDE.md 5).
if nargin < 1, noiseScale = 1; end
here = fileparts(mfilename('fullpath'));    % ROME_merged/tests
root = fileparts(here);                     % ROME_merged
addpath(here, fullfile(root, 'lab'), fullfile(root, 'Optitrack'), '-end');   % '-end': a mutated copy placed earlier on the path wins (make-it-fail runs)
fprintf('using: %s\n       %s\n       %s\n       %s\n', which('fit_motive_cal'), which('frame_check'), which('record_motive'), which('lab_rec_to_floor'));
fails = {};                                 % one message per failed check (cell of char)
rs = RandStream('mt19937ar', 'Seed', 20261119);   % private generator (no global rng change)

% quaternion helpers, [w x y z], Hamilton
qmul  = @(a, b) [a(1)*b(1)-a(2)*b(2)-a(3)*b(3)-a(4)*b(4); a(1)*b(2)+a(2)*b(1)+a(3)*b(4)-a(4)*b(3); ...
                 a(1)*b(3)-a(2)*b(4)+a(3)*b(1)+a(4)*b(2); a(1)*b(4)+a(2)*b(3)-a(3)*b(2)+a(4)*b(1)];
qconj = @(q) [q(1); -q(2:4)];
qax   = @(ax, ang) [cos(ang/2); sin(ang/2) * ax(:) / norm(ax)];   % rotation of ang (rad) about axis ax
q2R   = @(q) [1-2*(q(3)^2+q(4)^2), 2*(q(2)*q(3)-q(4)*q(1)), 2*(q(2)*q(4)+q(3)*q(1)); ...
              2*(q(2)*q(3)+q(4)*q(1)), 1-2*(q(2)^2+q(4)^2), 2*(q(3)*q(4)-q(2)*q(1)); ...
              2*(q(2)*q(4)-q(3)*q(1)), 2*(q(3)*q(4)+q(2)*q(1)), 1-2*(q(2)^2+q(3)^2)];
rodr  = @(ax, ang) eye(3) + sin(ang) * [0 -ax(3) ax(2); ax(3) 0 -ax(1); -ax(2) ax(1) 0] ...
                   + (1 - cos(ang)) * ([0 -ax(3) ax(2); ax(3) 0 -ax(1); -ax(2) ax(1) 0]^2);   % Rodrigues, unit axis

%% 0  helpers
axg = [1; 2; -0.5] / norm([1; 2; -0.5]);    % generic unit axis
c0 = norm(q2R(qax(axg, 0.7)) - rodr(axg, 0.7), 'fro') < 1e-14 ...
     && norm(q2R(qmul(qax([0 0 1], 0.3), qax([1 0 0], 0.4))) - rodr([0 0 1], 0.3) * rodr([1 0 0], 0.4), 'fro') < 1e-14;
fprintf('0    quaternion helpers vs Rodrigues -> %s\n', mat2str(c0));
if ~c0, fails{end+1} = '0: quaternion helpers disagree with Rodrigues'; end

%% i  fit_motive_cal
nm = 6; nf = 20;                            % marks, frames per mark (count)
mxy = [0 0; 2 0; 2 1.5; 0.5 2; -1 1.2; -0.5 -1];   % floor x y of the marks (m), 6x2
mh  = [0.0; 0.3; -1.0; 2.0; 3.0; -2.5];     % floor headings of the marks (rad)
sig = 0.3e-3;                               % noise, per frame and coordinate, floor metres (m)
cl = {[30 20 15], [30 60 15], [100 60 15], [160 20 75], [30 20 75], [100 120 75], [30 20 -40], [100 120 -40]};   % candidate yaw, roll, pitch (deg) for truth C
vq = {qmul(qax([0 0 1], 37*pi/180), qax([1 0 0], 0.5*pi/180)), ...   % A: rotation Motive -> floor (quaternion)
      qmul(qax([0 0 1], 37*pi/180), qax([1 0 0], pi/2)), ...         % B: Y-up Motive
      []};                                                            % C: chosen below (first candidate whose UNGUARDED SVD is a reflection)
vs = [1.0, 0.001, 0.001];                   % scales, m per Motive unit
vt = {[1.2; -0.4; 0], [1.2; -0.4; 0.3], [1.2; -0.4; 0.3]};    % floor position of the Motive origin (m)
vz = {0, 0.15, 0.15};                       % mark pivot height (m); A has no z column
vn = {'A (Z-up, 37 deg yaw, 0.5 deg tilt, scale 1)', 'B (Y-up, mm units, scale 0.001, z column)', ...
      'C (tilted, mm units, z column; unguarded SVD is a reflection)'};
optS = struct('sigma_mark_m', 0);           % the synthetic marks are exact: no tape uncertainty in the fit gate
calTrue = cell(1, 3); calFit = cell(1, 3); recsV = cell(1, 3); marksV = cell(1, 3);
for v = 1:3
    ncand = 1;                              % candidate rotations tried (count)
    if v == 3, ncand = numel(cl); end
    Ft = [mxy.'; vz{v} * ones(1, nm)];      % true floor marks (m), 3 x nm
    t = vt{v}; s = vs(v);                   % truth: translation (m), scale
    for cand = 1:ncand
        if v == 3
            vq{3} = qmul(qax([0 0 1], cl{cand}(1)*pi/180), qmul(qax([1 0 0], cl{cand}(2)*pi/180), qax([0 1 0], cl{cand}(3)*pi/180)));
        end
        qR = vq{v}; Rt = q2R(qR);           % truth: quaternion, R (3x3)
        recs = cell(1, nm);
        for i = 1:nm
            P = Rt.' * (Ft(:, i) - t) / s;  % true Motive position (Motive units), 3x1
            qb = qmul(qconj(qR), qax([0 0 1], mh(i)));   % body quaternion in the Motive frame (R_bm = R' Rz(psi))
            pose = [repmat(P, 1, nf) + (noiseScale * sig / s) * randn(rs, 3, nf); repmat(qb([2 3 4 1]), 1, nf)];
            ok = true(1, nf);
            pose(:, 3) = NaN; ok(3) = false;    % a rejected frame the fit must ignore
            recs{i} = struct('t', (0:nf-1)/100, 'pose7', pose, 'ok', ok);
        end
        if v == 3
            Pm = zeros(3, nm);              % mean Motive positions (Motive units)
            for i = 1:nm, Pm(:, i) = mean(recs{i}.pose7(1:3, recs{i}.ok), 2); end
            [U3, ~, V3] = svd(((Ft - mean(Ft, 2)) * (Pm - mean(Pm, 2)).') / nm);
            detUnguarded = det(U3 * V3.');  % det(U V') of the cross-covariance: -1 means an unguarded Umeyama returns a reflection
            if detUnguarded < 0, break; end
        end
    end
    if v == 3
        c3r = detUnguarded < 0;
        fprintf('i    truth C exercises the reflection guard: det(U V'') = %+.0f (candidate %d of %d) -> %s\n', detUnguarded, cand, ncand, mat2str(c3r));
        if ~c3r, fails{end+1} = 'i-C: no candidate truth made the unguarded SVD a reflection, the guard is not exercised'; end
    end
    if v == 1
        marks = table(mxy(:, 1), mxy(:, 2), mh, 'VariableNames', {'x', 'y', 'heading'});
    else
        marks = table(mxy(:, 1), mxy(:, 2), vz{v} * ones(nm, 1), mh, 'VariableNames', {'x', 'y', 'z', 'heading'});
    end
    recsV{v} = recs; marksV{v} = marks;
    cal = fit_motive_cal(recs, marks, optS);
    calFit{v} = cal;
    calTrue{v} = struct('R', Rt, 't', t, 'scale', s, 'measured', true);
    % tolerances: 3 x the effect of the noise on the mark mean (sigM) for 6 marks
    sigM = sig / sqrt(nf);                  % noise of a mark's mean position (m), unscaled
    Fc = mxy.' - mean(mxy.', 2);
    ev = eig(Fc * Fc.' / nm);               % in-plane principal variances of the marks (m^2)
    rhoMin = sqrt(min(ev)); rhoTot = sqrt(sum(ev));   % smallest principal rms radius, rms radius (m)
    sigTh = sigM / (rhoMin * sqrt(nm));     % rotation error per axis (rad)
    sigS  = sigM / (rhoTot * sqrt(nm));     % relative scale error
    dc = norm(mean(Ft, 2) - t);             % lever arm from the Motive origin to the marks' centroid (m)
    tolR = min(3*sqrt(3)*sigTh, 0.1*pi/180);                                   % rad (capped at the brief's 0.1 deg)
    tolT = min(3*sqrt(3*sigM^2/nm + 3*(sigTh*dc)^2 + (sigS*dc)^2), 1e-3);      % m (capped at 1 mm)
    tolS = 3*sqrt(3)*sigS;                  % relative
    dRm = cal.R.' * Rt;                     % residual rotation (3x3)
    eR = atan2(norm([dRm(3,2)-dRm(2,3), dRm(1,3)-dRm(3,1), dRm(2,1)-dRm(1,2)]) / 2, (trace(dRm) - 1) / 2);   % rotation error angle (rad)
    eT = norm(cal.t - t);                   % translation error (m)
    eS = abs(cal.scale / s - 1);            % relative scale error
    eH = max(abs(cal.heading_resid_rad));   % heading residual (rad)
    flds = all(isfield(cal, {'R', 't', 'scale', 'measured', 'fit_ok', 'pos_resid_m', 'heading_resid_rad'})) ...
           && ~isfield(cal, 'yaw_axis') ...   % yaw_axis removed everywhere (F5a, 19 Nov 2026): the yaw axis is floor z by convention
           && islogical(cal.measured) && ~cal.measured && cal.fit_ok && abs(det(cal.R) - 1) < 1e-12;
    ci = eR <= tolR && eT <= tolT && eS <= tolS && eH <= tolR + 1e-12 && flds;
    fprintf('i    %s: dR %.4f deg (tol %.4f), dt %.3f mm (tol %.3f), dscale %.2e (tol %.2e), head resid %.4f deg, rms resid %.3f mm (gate %.3f), fit_ok %d, measured %d, fields ok %d -> %s\n', ...
        vn{v}, eR*180/pi, tolR*180/pi, eT*1e3, tolT*1e3, eS, tolS, eH*180/pi, 1e3*cal.rms_pos_m, 1e3*cal.fit_gate_m, cal.fit_ok, cal.measured, flds, mat2str(ci));
    if ~ci, fails{end+1} = sprintf('i-%s: dR %.4f deg > %.4f or dt %.3f mm > %.3f or dscale %.2e > %.2e or head %.4f deg or fields/fit_ok/measured', ...
            vn{v}(1), eR*180/pi, tolR*180/pi, eT*1e3, tolT*1e3, eS, tolS, eH*180/pi); end %#ok<AGROW>
end

% fit_ok gate: a bad fit must be flagged; measured is never set by the fit
mb = marksV{2}; mb.x(2) = mb.x(2) + 0.020;  % one mark's floor x wrong by 20 mm
calB1 = fit_motive_cal(recsV{2}, mb, optS); % strict gate (no tape uncertainty)
mb2 = marksV{2}; mb2.x(2) = mb2.x(2) + 0.100;   % one mark wrong by 100 mm
calB2 = fit_motive_cal(recsV{2}, mb2);      % default gate (5 mm tape uncertainty, k = 3)
perm = [1 3 2 4 5 6];                       % marks 2 and 3 swapped relative to the recordings
calP = fit_motive_cal(recsV{2}, marksV{2}(perm, :));   % default gate
c5 = ~calB1.fit_ok && ~calB2.fit_ok && ~calP.fit_ok && ~calB1.measured && ~calB2.measured && ~calP.measured && calFit{2}.fit_ok;
fprintf('i    gate: 20 mm shifted mark strict -> fit_ok %d (rms %.1f mm, gate %.2f); 100 mm shifted default -> %d (rms %.1f, gate %.1f); marks in another order (2<->3) default -> %d (rms %.0f mm, gate %.1f); measured never true; good fit ok -> %s\n', ...
    calB1.fit_ok, 1e3*calB1.rms_pos_m, 1e3*calB1.fit_gate_m, calB2.fit_ok, 1e3*calB2.rms_pos_m, 1e3*calB2.fit_gate_m, calP.fit_ok, 1e3*calP.rms_pos_m, 1e3*calP.fit_gate_m, mat2str(c5));
if ~c5, fails{end+1} = 'i-gate: a bad fit (shifted mark or marks in a different order) was not flagged fit_ok = false, or measured was set'; end

%% ii  frame_check on a synthetic push recording
% name, dx (m), dy (m), dpsi (deg), ysign, truth variant, cal (1 = fit of A, 2 = true B), expected PASS, base shift during the rotation (m), recording end (s), expected rest count
cases = {'nominal',                    0.5,   0.5,   90,   1, 2, 2, true,  0,     49, 4; ...
         'y sign flipped',             0.5,   0.5,   90,  -1, 2, 2, false, 0,     49, 4; ...
         'x +8 mm (inside)',           0.508, 0.5,   90,   1, 2, 2, true,  0,     49, 4; ...
         'x +15 mm (outside)',         0.515, 0.5,   90,   1, 2, 2, false, 0,     49, 4; ...
         'y +8 mm (inside)',           0.5,   0.508, 90,   1, 2, 2, true,  0,     49, 4; ...
         'y +15 mm (outside)',         0.5,   0.515, 90,   1, 2, 2, false, 0,     49, 4; ...
         'yaw 91.5 deg (inside)',      0.5,   0.5,   91.5, 1, 2, 2, true,  0,     49, 4; ...
         'yaw 93 deg (outside)',       0.5,   0.5,   93,   1, 2, 2, false, 0,     49, 4; ...
         'rotation sign flipped',      0.5,   0.5,  -90,   1, 2, 2, false, 0,     49, 4; ...
         'rotation moves base 6 mm',   0.5,   0.5,   90,   1, 2, 2, true,  0.006, 49, 4; ...
         'rotation moves base 30 mm',  0.5,   0.5,   90,   1, 2, 2, false, 0.030, 49, 4; ...
         'recording cut: 3 rests only',0.5,   0.5,   90,   1, 2, 2, false, 0,     40, 3; ...
         'cal from fit (A)',           0.5,   0.5,   90,   1, 1, 1, true,  0,     49, 4};
s01 = @(tq, t0, dur) min(max((tq - t0) / dur, 0), 1);   % 0..1 ramp
sm  = @(u) u.^2 .* (3 - 2*u);               % smoothstep
x0 = 0.3; y0 = -0.2; p0 = 0.4;              % start pose of the push test on the floor (m, m, rad)
recNom = [];                                % the nominal recording made with truth A (for the refusal checks below)
for k = 1:size(cases, 1)
    cn = cases{k, 1}; vt_ = cases{k, 6}; expPass = cases{k, 8}; expN = cases{k, 11};   % name, truth variant, expected verdict, expected rests
    Rt = calTrue{vt_}.R; t = calTrue{vt_}.t; s = calTrue{vt_}.scale; qR = vq{vt_};
    ttc = 0:0.01:cases{k, 10};              % time (s), 100 Hz
    fx = x0 + cases{k, 2} * sm(s01(ttc, 10, 3)) + cases{k, 9} * sm(s01(ttc, 36, 3));   % floor x (m)
    fy = y0 + cases{k, 5} * cases{k, 3} * sm(s01(ttc, 23, 3));         % floor y (m)
    fp = p0 + cases{k, 4} * pi/180 * sm(s01(ttc, 36, 3));              % floor heading (rad)
    nt = numel(ttc);
    P = Rt.' * ([fx; fy; vz{vt_} * ones(1, nt)] - t) / s + (sig / s) * randn(rs, 3, nt);   % Motive positions (Motive units)
    pose = zeros(7, nt);
    for j = 1:nt
        qb = qmul(qconj(qR), qax([0 0 1], fp(j)));
        pose(:, j) = [P(:, j); qb([2 3 4 1])];
    end
    recP = struct('t', ttc, 'pose7', pose, 'ok', true(1, nt));
    if strcmp(cn, 'cal from fit (A)'), recNom = recP; end
    if cases{k, 7} == 1, calU = calFit{1}; else, calU = calTrue{2}; end
    rep = frame_check(recP, calU);
    cp = (rep.pass == expPass) && rep.nRest == expN;
    if strcmp(cn, 'nominal')
        cp = cp && abs(rep.steps(1).meas_dxy(1) - 0.5) < 2e-3 && abs(rep.steps(2).meas_dxy(2) - 0.5) < 2e-3 ...
                && abs(rep.steps(3).meas_dpsi - pi/2) < 0.2*pi/180;
    elseif strcmp(cn, 'y sign flipped')
        cp = cp && abs(rep.steps(2).meas_dxy(2) + 0.5) < 2e-3 && ~rep.steps(2).pass;
    elseif strcmp(cn, 'recording cut: 3 rests only')
        cp = cp && ~isempty(strfind(rep.reason, 'rest segments')); %#ok<STREMP>
    end
    fprintf('ii   %-28s verdict %s (expected %s), nRest %d (expected %d), step err mm [%s] deg [%s] -> %s\n', cn, ...
        char(string(rep.pass)), char(string(expPass)), rep.nRest, expN, num2str(1e3*[rep.steps.err_pos_m], '%.1f '), ...
        num2str(180/pi*[rep.steps.err_psi_rad], '%.2f '), mat2str(cp));
    if ~cp, fails{end+1} = sprintf('ii: %s gave pass=%d, expected %d (nRest %d, expected %d; %s)', cn, rep.pass, expPass, rep.nRest, expN, rep.reason); end %#ok<AGROW>
end
% frame_check refuses a cal that is neither fit_ok nor measured, and fails a wrongly fitted cal
calU1 = calFit{1}; calU1.fit_ok = false;    % fit gate failed, measured false
calU2 = calB2;                              % a real bad fit (100 mm wrong mark): fit_ok false, measured false
calU3 = struct('R', calFit{1}.R, 't', calFit{1}.t, 'scale', calFit{1}.scale);   % no flags at all
ids = {'', '', ''};                         % error identifiers of the three refusals
cs = {calU1, calU2, calU3};
for j = 1:3
    try, frame_check(recNom, cs{j}); catch e, ids{j} = e.identifier; end
end
cr = all(strcmp(ids, 'frame_check:cal'));
fprintf('ii   refusals (fit_ok false / bad fit / no flags): ids ''%s'' ''%s'' ''%s'' -> %s\n', ids{:}, mat2str(cr));
if ~cr, fails{end+1} = 'ii: frame_check did not refuse a cal that is neither fit_ok nor measured'; end
calT = calFit{1}; calT.R = calT.R.';        % a wrongly fitted cal: R transposed (fit_ok flag left true)
repT = frame_check(recNom, calT);
crt = ~repT.pass;
fprintf('ii   wrongly fitted cal (R transposed): verdict %s, reason ''%s'' -> %s\n', char(string(repT.pass)), repT.reason, mat2str(crt));
if ~crt, fails{end+1} = 'ii: frame_check passed a cal with R transposed'; end

%% iii  record_motive on a fake transport
[~, ~, endian] = computer;
assert(endian == 'L', 'test_lab_frame:endian', 'host is big-endian');
le16  = @(vv) reshape(typecast(uint16(vv), 'uint8'), [], 1);
le32i = @(vv) reshape(typecast(int32(vv), 'uint8'), [], 1);
f32   = @(vv) reshape(typecast(single(vv(:).'), 'uint8'), [], 1);
msName = uint8(['RigidBody 1' 0]).';
build = @(msg, id, pose) [le16(msg); le16(78); le32i(1234); le32i(1); msName; le32i(0); le32i(0); le32i(1); ...
                          le32i(id); f32(pose); f32(0.0005); le16(1); zeros(8, 1, 'uint8')];   % hand-built NatNet frame (layout doc sections 3-4)
rbID = 3;                                   % Streaming ID recorded (-)
pk = cell(1, 33); pz = zeros(7, 33);        % queue of datagrams and the poses inside them
for j = 1:33
    q = [0.1; -0.2; 0.3 + 0.001*j]; pz(:, j) = [0.1*j; -0.05*j; 0.3; q; sqrt(1 - sum(q.^2))];
    pk{j} = build(7, rbID, pz(:, j));
end
queue = [pk(1:10), {[]}, pk(11:20), {build(7, 4, pz(:, 21))}, {[]}, pk(21:30)];   % 30 valid, 1 wrong ID, 2 timeouts = 33 entries
% expected ok mask over the 31 non-empty datagrams in order: 10 valid, 10 valid, wrong ID, 10 valid
okExp = [true(1, 20), false, true(1, 10)];
fakeA = RecorderFakeTransport(queue);       % 0.0078125 s per receive: 64 receive calls in 0.5 s
outFile = [tempname '.mat'];                % file the recording is written to (temp folder)
cleanup = onCleanup(@() delete(outFile)); %#ok<NASGU>
rec = record_motive(0.5, outFile, struct('transport', fakeA, 'rbID', rbID));
ex1 = isfile(outFile);
L = load(outFile);
sv = isequaln(L.rec, rec);
nP = numel(rec.t);
bytesOk = nP == 31 && isequal(rec.bytes(1:10), pk(1:10)) && isequal(rec.bytes(11:20), pk(11:20)) && isequal(rec.bytes{21}, build(7, 4, pz(:, 21))) && isequal(rec.bytes(22:31), pk(21:30));
poseOk = isequal(rec.pose7(:, [1:20 22:31]), double(single(pz(:, 1:30)))) && all(isnan(rec.pose7(:, 21)));
flagOk = isequal(rec.ok, okExp) && ~isempty(rec.why{21});
cntOk = rec.nReceives == 64 && rec.nTimeouts == 33 && fakeA.nOpen == 1 && fakeA.nClose == 1 && abs(rec.t(1) - 0.0078125) < 1e-15;
c3 = ex1 && sv && bytesOk && poseOk && flagOk && cntOk;
fprintf('iii  record_motive: file %d, saved==returned %d, 31 datagrams byte-exact %d, poses %d, ok flags %d, counts (64 receives, 33 timeouts, open/close once) %d -> %s\n', ...
    ex1, sv, bytesOk, poseOk, flagOk, cntOk, mat2str(c3));
if ~c3, fails{end+1} = 'iii: record_motive output/file check failed'; end
fid = fopen(outFile, 'r'); before = fread(fid, Inf, 'uint8=>uint8'); fclose(fid);   % file bytes before the second call
fakeB = RecorderFakeTransport(queue(1:5));
errId = '';
try
    record_motive(0.5, outFile, struct('transport', fakeB, 'rbID', rbID));
catch e
    errId = e.identifier;
end
fid = fopen(outFile, 'r'); after = fread(fid, Inf, 'uint8=>uint8'); fclose(fid);
c3b = strcmp(errId, 'record_motive:exists') && isequal(before, after) && fakeB.nOpen == 0 && fakeB.nReceive == 0;
fprintf('iii  second call, same name: error id ''%s'', file unchanged %d, transport never opened %d -> %s\n', errId, isequal(before, after), fakeB.nOpen == 0, mat2str(c3b));
if ~c3b, fails{end+1} = sprintf('iii: second call did not refuse cleanly (id %s)', errId); end
errId2 = '';
try
    record_motive(0.5, [tempname '.txt'], struct('transport', RecorderFakeTransport({})));
catch e
    errId2 = e.identifier;
end
c3c = strcmp(errId2, 'record_motive:ext');
fprintf('iii  non-.mat name refused (id ''%s'') -> %s\n', errId2, mat2str(c3c));
if ~c3c, fails{end+1} = 'iii: non-.mat name not refused'; end

%% iii-b  record_motive opens a real socket only with opts.live = true (a shadow MotiveUdpTransport that errors when constructed)
shDir = tempname; mkdir(shDir);             % folder with the shadow class (temp)
fid = fopen(fullfile(shDir, 'MotiveUdpTransport.m'), 'w');
fprintf(fid, '%s\n', 'classdef MotiveUdpTransport < handle', '    methods', '        function obj = MotiveUdpTransport(varargin)', ...
        '            error(''SHADOW:constructed'', ''MotiveUdpTransport constructed'');', '        end', '    end', 'end');
fclose(fid);
addpath(shDir);
cleanSh1 = onCleanup(@() rmpath(shDir)); cleanSh2 = onCleanup(@() rmdir(shDir, 's')); %#ok<NASGU>
shOn = ~isempty(strfind(which('MotiveUdpTransport'), shDir)); %#ok<STREMP>
idS = ''; try, MotiveUdpTransport(); catch e, idS = e.identifier; end
fOk = [tempname '.mat']; fLive = [tempname '.mat']; fNo = [tempname '.mat']; fNo2 = [tempname '.mat'];   % output files (temp)
fakeC = RecorderFakeTransport(queue(1:5));
idInj = ''; try, record_motive(0.1, fOk, struct('transport', fakeC, 'rbID', rbID)); catch e, idInj = e.identifier; end
idLive = ''; try, record_motive(0.1, fLive, struct('live', true)); catch e, idLive = e.identifier; end
idNo = ''; try, record_motive(0.1, fNo); catch e, idNo = e.identifier; end
idNo2 = ''; try, record_motive(0.1, fNo2, struct('live', false)); catch e, idNo2 = e.identifier; end
c3d = shOn && strcmp(idS, 'SHADOW:constructed') && isempty(idInj) && isfile(fOk) ...
      && strcmp(idLive, 'SHADOW:constructed') && ~isfile(fLive) ...
      && strcmp(idNo, 'record_motive:noTransport') && ~isfile(fNo) && strcmp(idNo2, 'record_motive:noTransport') && ~isfile(fNo2);
fprintf('iii  shadow active %d; injected transport: error ''%s'', file %d (never constructs the class); live=true: ''%s'', no file; no transport: ''%s''; live=false: ''%s'' -> %s\n', ...
    shOn, idInj, isfile(fOk), idLive, idNo, idNo2, mat2str(c3d));
if ~c3d, fails{end+1} = 'iii: live / injected / no-transport behaviour of record_motive is wrong'; end
if isfile(fOk), delete(fOk); end

%% iv  fit_motive_cal refuses bad input
recsC = cell(1, 6);
for i = 1:6
    recsC{i} = struct('t', 0, 'pose7', [i; 2*i; 0; 0; 0; 0; 1], 'ok', true);
end
mcol = table((1:6).', 2*(1:6).', zeros(6, 1), 'VariableNames', {'x', 'y', 'heading'});   % 6 collinear marks
idc = '';
try, fit_motive_cal(recsC, mcol); catch e, idc = e.identifier; end
idn = '';
try, fit_motive_cal(recsC(1:5), table(mxy(:, 1), mxy(:, 2), mh, 'VariableNames', {'x', 'y', 'heading'})); catch e, idn = e.identifier; end
c4 = strcmp(idc, 'fit_motive_cal:collinear') && strcmp(idn, 'fit_motive_cal:count');
fprintf('iv   collinear marks -> ''%s'', count mismatch -> ''%s'' -> %s\n', idc, idn, mat2str(c4));
if ~c4, fails{end+1} = sprintf('iv: wrong refusal ids (%s, %s)', idc, idn); end

if isempty(fails)
    fprintf('test_lab_frame: ALL CHECKS PASSED (noiseScale %g)\n', noiseScale);
else
    fprintf('test_lab_frame: %d FAILED:\n', numel(fails));
    fprintf('  %s\n', fails{:});
    error('test_lab_frame:failed', '%d check(s) failed (noiseScale %g)', numel(fails), noiseScale);
end
end
