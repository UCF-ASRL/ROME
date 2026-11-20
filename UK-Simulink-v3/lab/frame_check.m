function rep = frame_check(recPush, cal, opts)
%FRAME_CHECK  Part C step 12 evaluator: does the stream, through cal, match the push test?
%   rep = FRAME_CHECK(recPush, cal)
%   rep = FRAME_CHECK(recPush, cal, opts)
%     recPush  one record_motive recording of the whole push test (docs/lab/PART_C_LAB_VISIT.md C12):
%              rest, push +0.5 m along floor x, rest, push +0.5 m along floor y, rest, rotate +90 deg
%              (counter-clockwise seen from above) about the pivot, rest. Four rests of >= min_rest_s.
%     cal      struct from fit_motive_cal (R, t, scale). Refused (frame_check:cal) unless cal.fit_ok is true
%              (a good fit, not yet signed off) or cal.measured is true (signed off).
%     opts     optional struct overriding the defaults below (all provisional, UNMEASURED):
%                pos_tol_m 0.010   PASS bound on the position error of each step (m)
%                ang_tol_rad 2 deg PASS bound on the heading error of each step (rad)
%                  The two bounds are [Claude's call, author to confirm]: tape placement about 5 mm and
%                  hand alignment of the heading about 1 deg, doubled (Plan 3 Task 1 brief).
%                push_m 0.5, rot_rad pi/2   expected step sizes
%                win_s 0.5, still_pos_m 0.003, still_ang_rad 0.5 deg   a frame is "still" when the pose
%                  changed less than this over the next win_s seconds
%                min_rest_s 3, trim_s 0.5   a rest is a still run of at least min_rest_s; trim_s is cut
%                  from both ends before the rest pose (median) is taken
%     rep      struct: pass (logical), reason (char, '' if pass), nRest (count), rest (struct array:
%              t0 t1 x y psi n), steps (struct array with name, exp_dxy, exp_dpsi, meas_dxy, meas_dpsi,
%              err_pos_m, err_psi_rad, pass), bounds, text (printable table). Prints the table if
%              nargout == 0.
%   Expected steps: [+0.5 0] m, [0 +0.5] m, and +90 deg with no position change (the rotation is
%   about the pivot). Heading drift during the two pushes is expected to be 0 and is held to the
%   same angle bound: a hand push that turns the robot more than 2 deg fails, by this author-less
%   judgment (NOT an author ruling).
%   Pose = floor [x; y; psi] through cal (lab_rec_to_floor). psi is unwrapped across the recording.
if nargin < 3 || isempty(opts)
    opts = struct();
end
dflt = struct('pos_tol_m', 0.010, 'ang_tol_rad', 2*pi/180, 'push_m', 0.5, 'rot_rad', pi/2, ...
              'win_s', 0.5, 'still_pos_m', 0.003, 'still_ang_rad', 0.5*pi/180, 'min_rest_s', 3, 'trim_s', 0.5);
fn = fieldnames(dflt);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k})
        opts.(fn{k}) = dflt.(fn{k});
    end
end
for k = {'R', 't', 'scale'}
    if ~isfield(cal, k{1})
        error('frame_check:cal', 'cal has no field %s', k{1});
    end
end
isFitOk = isfield(cal, 'fit_ok') && logical(cal.fit_ok);        % residual gate of fit_motive_cal passed
isMeasured = isfield(cal, 'measured') && logical(cal.measured); % signed off
if ~(isFitOk || isMeasured)
    error('frame_check:cal', 'cal is neither fit_ok nor measured: refusing to check against a bad or unmeasured transform');
end
rep = struct('pass', false, 'reason', '', 'nRest', 0, 'rest', [], 'steps', [], ...
             'bounds', struct('pos_tol_m', opts.pos_tol_m, 'ang_tol_rad', opts.ang_tol_rad), 'text', '');
[fl, tt] = lab_rec_to_floor(recPush, cal);  % floor [x;y;psi] (3xM) and times (1xM) of accepted frames
M = numel(tt);                              % accepted frames (count)
if M < 10
    rep.reason = sprintf('only %d accepted frames in the recording', M);
    rep.text = rep.reason;
    if nargout == 0, disp(rep.text); end
    return
end
if any(diff(tt) < 0)
    rep.reason = 'receive times are not non-decreasing';
    rep.text = rep.reason;
    if nargout == 0, disp(rep.text); end
    return
end
psiU = unwrap(fl(3, :));                    % heading, unwrapped (rad), 1xM
% --- still frames: pose change over the next win_s seconds (backward window at the very end) -------
partner = zeros(1, M);                      % index of the frame win_s later (or earlier at the end); 0 if none
j = 1;                                      % forward pointer
for i = 1:M
    while j <= M && tt(j) < tt(i) + opts.win_s
        j = j + 1;
    end
    if j <= M
        partner(i) = j;
    end
end
jb = 1;                                     % backward pointer: last index with tt <= tt(i) - win_s
for i = 1:M
    if partner(i) == 0
        while jb < M && tt(jb + 1) <= tt(i) - opts.win_s
            jb = jb + 1;
        end
        if tt(jb) <= tt(i) - opts.win_s
            partner(i) = jb;
        end
    end
end
still = false(1, M);                        % frame belongs to a rest (logical)
has = partner > 0;
pi_ = partner(has);                         % partner indices (1xK)
dpos = hypot(fl(1, has) - fl(1, pi_), fl(2, has) - fl(2, pi_));   % position change over the window (m)
dang = abs(psiU(has) - psiU(pi_));          % heading change over the window (rad)
still(has) = dpos < opts.still_pos_m & dang < opts.still_ang_rad;
% --- runs of still frames, merged over gaps under 0.3 s, kept if long enough -----------------------
d = diff([0, still, 0]);
s0 = find(d == 1);                          % first index of each run
s1 = find(d == -1) - 1;                     % last index of each run
keep0 = zeros(1, 0); keep1 = zeros(1, 0);   % merged runs
for k = 1:numel(s0)
    if ~isempty(keep1) && tt(s0(k)) - tt(keep1(end)) < 0.3
        keep1(end) = s1(k);
    else
        keep0(end + 1) = s0(k); %#ok<AGROW>
        keep1(end + 1) = s1(k); %#ok<AGROW>
    end
end
rest = struct('t0', {}, 't1', {}, 'x', {}, 'y', {}, 'psi', {}, 'n', {});
for k = 1:numel(keep0)
    if tt(keep1(k)) - tt(keep0(k)) >= opts.min_rest_s
        sel = keep0(k):keep1(k);            % frames of this rest
        inner = sel(tt(sel) >= tt(keep0(k)) + opts.trim_s & tt(sel) <= tt(keep1(k)) - opts.trim_s);
        if numel(inner) < 3
            inner = sel;
        end
        rest(end + 1) = struct('t0', tt(keep0(k)), 't1', tt(keep1(k)), ...   %#ok<AGROW>
                               'x', median(fl(1, inner)), 'y', median(fl(2, inner)), ...
                               'psi', median(psiU(inner)), 'n', numel(inner));
    end
end
rep.rest = rest;
rep.nRest = numel(rest);
names = {'push +x', 'push +y', 'rotate +90 deg'};
expD = [opts.push_m, 0; 0, opts.push_m; 0, 0];   % expected floor [dx dy] per step (m), 3x2
expP = [0; 0; opts.rot_rad];                % expected heading change per step (rad)
steps = struct('name', {}, 'exp_dxy', {}, 'exp_dpsi', {}, 'meas_dxy', {}, 'meas_dpsi', {}, ...
               'err_pos_m', {}, 'err_psi_rad', {}, 'pass', {});
for k = 1:min(3, rep.nRest - 1)
    md = [rest(k + 1).x - rest(k).x, rest(k + 1).y - rest(k).y];   % measured floor [dx dy] (m)
    mp = rest(k + 1).psi - rest(k).psi;     % measured heading change (rad)
    ep = norm(md - expD(k, :));             % position error (m)
    ea = mp - expP(k);                      % heading error (rad)
    steps(end + 1) = struct('name', names{k}, 'exp_dxy', expD(k, :), 'exp_dpsi', expP(k), ...   %#ok<AGROW>
                            'meas_dxy', md, 'meas_dpsi', mp, 'err_pos_m', ep, 'err_psi_rad', ea, ...
                            'pass', ep <= opts.pos_tol_m && abs(ea) <= opts.ang_tol_rad);
end
rep.steps = steps;
if rep.nRest ~= 4
    rep.reason = sprintf('found %d rest segments of >= %.1f s, expected 4 (start, after x, after y, after rotation)', ...
                         rep.nRest, opts.min_rest_s);
elseif ~all([steps.pass])
    bad = names(~[steps.pass]);
    rep.reason = ['step outside the bounds: ', strjoin(bad, ', ')];
end
rep.pass = rep.nRest == 4 && all([steps.pass]);
lines = cell(1, numel(steps) + 2);
lines{1} = sprintf('frame_check: %d rest segments; bounds %.1f mm, %.1f deg', rep.nRest, 1e3*opts.pos_tol_m, opts.ang_tol_rad*180/pi);
for k = 1:numel(steps)
    s = steps(k);
    lines{k + 1} = sprintf('  %-15s expected dxy [%+.3f %+.3f] m dpsi %+.2f deg | measured [%+.4f %+.4f] m %+.2f deg | err %.1f mm %+.2f deg | %s', ...
        s.name, s.exp_dxy(1), s.exp_dxy(2), s.exp_dpsi*180/pi, s.meas_dxy(1), s.meas_dxy(2), s.meas_dpsi*180/pi, ...
        1e3*s.err_pos_m, s.err_psi_rad*180/pi, ternary_(s.pass, 'PASS', 'FAIL'));
end
lines{end} = ['  OVERALL: ', ternary_(rep.pass, 'PASS', ['FAIL (', rep.reason, ')'])];
rep.text = strjoin(lines, newline);
if nargout == 0
    disp(rep.text);
end
end

function out = ternary_(c, a, b)
% ternary_  a if c else b (char); local helper
if c
    out = a;
else
    out = b;
end
end
