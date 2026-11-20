function [ok, reasons, info] = prerun_check_core(S)
%PRERUN_CHECK_CORE  Pre-run check of a hardware run, pure core (focus plan F2 + F11; Plan 3 Task 4).
%   [ok, reasons, info] = prerun_check_core(S)
%   Reads NOTHING global: no base workspace, no model, no file. Everything comes in through the state struct S, so
%   every refusal is testable offline (tests/test_prerun_core.m). The wrapper that fills S from the workspace and the
%   model (prerun_check.m, called at the end of define_constants when EnableHardware = 1) is the later wiring step.
%
%   ok       true when the run may start (logical)
%   reasons  1xN cell of char, EVERY failing item (not only the first). Each reason reads
%              '<item>: <value found>; allowed: <allowed>; source: <source>'
%            <item> is the token the tests match: a variable name of allowed_ranges.m, or one of
%              pacing, sensor_model_hw, arm_freeze_banner, dc_sha, release, signoff_partF, motive_source,
%              motive_cal.measured, signoff_partC, park
%   info     struct: skipped (true when EnableHardware = 0: nothing is checked), park_tol_deg, unchecked ('name (status)'
%            the UNSOURCED rows of allowed_ranges, not enforced), items (items that were active)
%
%   S fields (the wrapper's job to fill; a missing field is a refusal of its item, never a pass):
%     values         struct of the user-block values by allowed_ranges ws_name (EnableHardware, EnableMotive,
%                    scenario, speed_factor, ..., arm_freeze_user, clearance_margin, motive_source, motive_cal)
%     pacing         struct EnablePacing (char), PacingRate (double), FastRestart (char): get_param of the model
%     banner_error   '' when arm_freeze_banner(mdl) passed, else its error identifier and message (char). Covers the
%                    freeze WIRING refusal (arm_freeze_banner:notwired), which needs the model and is not re-done here
%     dc_sha_at_run  SHA-256 define_constants stored when it last ran to its end (hex char; dc_file_sha.m)
%     dc_sha_now     SHA-256 of define_constants.m now (hex char)
%     release        version('-release'), e.g. '2026a' (char)
%     signoff        struct partF, partC: sign-off dates 'yyyy-mm-dd' (char), '' when not signed (lab_signoff.m, later)
%     frozen_fw_deg  1x6 the joint command a frozen run sends (firmware deg): arm_sign .* rad2deg(q0_9dof(4:9)) +
%                    arm_offset_deg, then arm_clamp, the computation of arm_freeze_banner.m:88-96 @4134946 and
%                    ROMECommand. NOT arm_freeze_deg, which can be stale (TRACKER Q-F1-a)
%     park_record    [] or the struct arm_park_record('read', file) returns (fw_deg 1x6, dc_sha, time)
%
%   Items (when each applies), all for EnableHardware ~= 0:
%     ranges              always: every row of allowed_ranges with kind interval / set / equals
%     pacing              always: EnablePacing 'on', PacingRate 1, FastRestart 'off' (apply_pacing.m enforces it at
%                         every start; re-asserted here, plan 3 Task 4)
%     sensor_model_hw     always: sensor_model = 0 (apply_pacing.m:73-86 @4134946; TRACKER section 2 final review (4))
%     arm_freeze_banner   always: banner_error empty
%     dc_sha              always: dc_sha_at_run = dc_sha_now, both non-empty (arm_freeze_banner.m:38-51 @4134946)
%     release             always: release >= allowed_ranges C.min_release
%     signoff_partF       always: a valid date (PART_I_RUN_PLAN.md:47)
%     motive_source       EnableMotive ~= 0: 'emulator' or 'optitrack'
%     motive_cal.measured EnableMotive ~= 0 and motive_source 'optitrack' only (focus plan L17: the emulator needs no
%                         lab frame). Malformed R / t / scale stay with motive_cal_check.m (F5b), not repeated here
%     signoff_partC       same condition as motive_cal.measured (PART_I_RUN_PLAN.md:48, :85)
%     park (F11)          arm_freeze_user ~= 0: a parked record exists, was made with the current define_constants
%                         (record dc_sha = dc_sha_now), and |frozen_fw_deg - record fw_deg| <= C.park_tol_deg on every
%                         joint. Reason: a frozen run sends frozen_fw_deg for the whole run; if the arm is parked
%                         elsewhere the firmware moves it there at the first packet, which the freeze exists to prevent
%                         (define_constants arm_freeze comment, L123-127 @4134946; F1 review I3)
%                         LIMIT: compares commanded postures only; it does not detect an arm moved by hand, a jog or
%                         a power cycle after calibration; re-calibrate after any of these. A check of the actual joints
%                         (telemetry) at run start was offered and not chosen (author 19 Nov, "OK as is"); not planned.
%     (frozen_feedback, Q-F7-a, removed 19 Nov 2026 on the author's decision "Arm from model if frozen": with the
%                         arm frozen the controller reads the model's arm angles, JointSource on SimJoints)
%   Fail-safe readings: an EnableHardware that is not exactly 0 counts as a robot run; an invalid EnableMotive as
%   cameras on; an invalid motive_source as 'optitrack'; an arm_freeze_user that is not exactly 0 as frozen. The range
%   item reports the invalid value itself.
if nargin < 1 || ~isstruct(S), S = struct(); end
[T, C] = allowed_ranges();
reasons = {};                                       % failing items (cell of char)
info = struct('skipped', false, 'park_tol_deg', C.park_tol_deg, ...
              'unchecked', {{}}, 'items', {{}});
unen = ~ismember(T.status, {'SOURCED', 'CHECKED BY ITEM'});   % rows with a limit not enforced on some side (logical)
info.unchecked = strcat(T.name(unen), {' ('}, T.status(unen), {')'}).';   % every unenforced limit, 'name (status)' (cell of char)
V = fld(S, 'values', struct());                     % user-block values (struct)

eh = fld(V, 'EnableHardware', []);                  % 0 simulate only, 1 robot (flag)
if isnumeric(eh) && isscalar(eh) && eh == 0
    ok = true;  info.skipped = true;                % simulation only: the pre-run check is not called
    return
end

% ---- 1. every user-block value in its allowed range (allowed_ranges.m, the single source) --------------------
info.items{end+1} = 'ranges';
for i = 1:height(T)
    kind = T.kind{i};                               % 'interval' | 'set' | 'equals' | 'item' | 'none' (char)
    if ~any(strcmp(kind, {'interval', 'set', 'equals'})), continue; end
    ws = T.ws_name{i};                              % workspace variable read (char)
    item = T.name{i};                               % item token printed in the reason (char)
    if ~strcmp(ws, item), item = sprintf('%s (read as %s)', item, ws); end
    allow = range_text(T, i);                       % allowed range as printed (char)
    if ~isfield(V, ws)
        reasons{end+1} = sprintf('%s: undefined; allowed: %s; source: %s', item, allow, T.source{i}); %#ok<AGROW>
        continue
    end
    v = V.(ws);                                     % value found (unit of the row, T.unit)
    sz = T.sz(i);  if iscell(sz), sz = sz{1}; end   % required size ([] any)
    good = isnumeric(v) && isreal(v) && ~isempty(v) && (isempty(sz) || isequal(size(v), sz));
    if good
        switch kind
            case 'interval'
                lo = T.min(i);  hi = T.max(i);      % bounds (unit of the row)
                if T.min_incl(i), okLo = v >= lo; else, okLo = v > lo; end   % elementwise lower bound met (logical)
                if T.max_incl(i), okHi = v <= hi; else, okHi = v < hi; end   % elementwise upper bound met (logical)
                good = all(okLo(:) & okHi(:)) && ~any(isnan(v(:)));
            case 'set'
                good = all(ismember(v(:), T.allowed{i}{1}));
            case 'equals'
                ref = T.allowed{i}{1};              % firmware table the value must equal (unit of the row)
                good = isequal(size(v), size(ref)) && all(abs(v(:) - ref(:)) <= 1e-9 * max(1, abs(ref(:))));
        end
    end
    if ~good
        reasons{end+1} = sprintf('%s: value %s; allowed: %s; source: %s', item, val_text(v), allow, T.source{i}); %#ok<AGROW>
    end
end

% ---- 2. pacing -------------------------------------------------------------------------------------------------
info.items{end+1} = 'pacing';
P = fld(S, 'pacing', struct());                     % pacing state of the model (struct)
ep = fld(P, 'EnablePacing', '');  pr = fld(P, 'PacingRate', []);  fr = fld(P, 'FastRestart', '');   % 'on'/'off' (char), rate (x real time), 'on'/'off' (char)
if ~(ischar(ep) && strcmp(ep, 'on') && isnumeric(pr) && isscalar(pr) && pr == 1 && ischar(fr) && strcmp(fr, 'off'))
    reasons{end+1} = sprintf(['pacing: EnablePacing %s, PacingRate %s, FastRestart %s; allowed: EnablePacing ''on'', ' ...
        'PacingRate 1, FastRestart ''off''; source: apply_pacing.m:44-65 @4134946 (pacing on whenever EnableHardware = 1; ' ...
        'Fast Restart refused)'], val_text(ep), val_text(pr), val_text(fr));
end

% ---- 3. sensor noise off on the robot ---------------------------------------------------------------------------
info.items{end+1} = 'sensor_model_hw';
sm = fld(V, 'sensor_model', []);                    % 1 simulated noise, 0 exact state (flag)
if ~(isnumeric(sm) && isscalar(sm) && sm == 0)
    reasons{end+1} = sprintf(['sensor_model_hw: sensor_model %s on a robot run; allowed: 0 (the SensorModel noise is ' ...
        'simulation-only); source: apply_pacing.m:73-86 @4134946 (review decision 19 Nov: the robot gets no synthetic noise, as in lab main)'], val_text(sm));
end

% ---- 4. arm-freeze banner (wiring refusal, needs the model: passed in) -----------------------------------------
info.items{end+1} = 'arm_freeze_banner';
if ~isfield(S, 'banner_error')
    reasons{end+1} = ['arm_freeze_banner: not run (banner_error missing); allowed: arm_freeze_banner passes; ' ...
        'source: arm_freeze_banner.m:17-26 @4134946'];
elseif ~(ischar(S.banner_error) && isempty(S.banner_error))
    reasons{end+1} = sprintf(['arm_freeze_banner: refused (%s); allowed: arm_freeze_banner passes (freeze wiring as ' ...
        'tools/wire_arm_freeze.m builds it); source: arm_freeze_banner.m:17-26, :52-81 @4134946'], val_text(S.banner_error));
end

% ---- 5. define_constants not changed since it last ran ----------------------------------------------------------
info.items{end+1} = 'dc_sha';
shaRun = fld(S, 'dc_sha_at_run', '');  shaNow = fld(S, 'dc_sha_now', '');   % SHA-256 at the last run / now (hex char)
if ~(ischar(shaRun) && ischar(shaNow) && ~isempty(shaRun) && strcmp(shaRun, shaNow))
    reasons{end+1} = sprintf(['dc_sha: define_constants.m SHA-256 now %s, at its last complete run %s; allowed: equal ' ...
        '(re-run define_constants after every edit); source: arm_freeze_banner.m:38-51 @4134946, dc_file_sha.m'], ...
        val_text(shaNow), val_text(shaRun));
end

% ---- 6. MATLAB release -------------------------------------------------------------------------------------------
info.items{end+1} = 'release';
rel = fld(S, 'release', '');                        % MATLAB release, e.g. '2026a' (char)
if ~release_ok(rel, C.min_release)
    reasons{end+1} = sprintf('release: %s; allowed: R%s or newer; source: %s', val_text(rel), C.min_release, C.min_release_src);
end

% ---- 7. Part F sign-off (every robot run) -----------------------------------------------------------------------
info.items{end+1} = 'signoff_partF';
SO = fld(S, 'signoff', struct());                   % sign-off dates (struct of char)
dF = fld(SO, 'partF', '');                          % Part F sign-off date 'yyyy-mm-dd' (char)
if ~date_ok(dF)
    reasons{end+1} = sprintf('signoff_partF: %s; allowed: the Part F bench sign-off date, yyyy-mm-dd; source: %s', ...
        val_text(dF), C.signoff.partF_src);
end

% ---- 8. cameras: source, calibration and Part C (mode 3 only) ---------------------------------------------------
em = fld(V, 'EnableMotive', []);
cam = ~(isnumeric(em) && isscalar(em) && em == 0);  % cameras on (fail-safe: invalid counts as on) (logical)
if cam
    info.items{end+1} = 'motive_source';
    src = fld(V, 'motive_source', '');              % 'emulator' | 'optitrack' (char)
    if isstring(src) && isscalar(src), src = char(src); end
    srcOk = ischar(src) && any(strcmp(src, {'emulator', 'optitrack'}));   % valid source (logical)
    if ~srcOk
        reasons{end+1} = sprintf(['motive_source: %s with EnableMotive = 1; allowed: ''emulator'' (mode 2) or ''optitrack'' ' ...
            '(mode 3); source: author decision 19 Nov, run_mode_table.m'], val_text(src));
    end
    if ~srcOk || strcmp(src, 'optitrack')           % fail-safe: an invalid source is treated as the real cameras
        info.items{end+1} = 'motive_cal.measured';
        cal = fld(V, 'motive_cal', []);             % camera-to-floor calibration (struct)
        meas = isstruct(cal) && isscalar(cal) && isfield(cal, 'measured') && isscalar(cal.measured) && ...
               (islogical(cal.measured) || isnumeric(cal.measured)) && isequal(cal.measured, true);
        if ~meas
            if isstruct(cal) && isscalar(cal) && isfield(cal, 'measured'), mtxt = val_text(cal.measured); else, mtxt = 'undefined'; end   % measured as printed (char)
            reasons{end+1} = sprintf(['motive_cal.measured: %s with motive_source %s (needed for ''optitrack''; an invalid source counts as ''optitrack''); allowed: true (Part C ' ...
                'frame measured and entered); source: author decision 19 Nov "Yes, refuse until measured", optitrack only ' ...
                '(not the fake camera), motive_cal_check.m'], mtxt, val_text(src));
        end
        info.items{end+1} = 'signoff_partC';
        dC = fld(SO, 'partC', '');                  % Part C sign-off date 'yyyy-mm-dd' (char)
        if ~date_ok(dC)
            reasons{end+1} = sprintf(['signoff_partC: %s with motive_source %s (needed for ''optitrack''; an invalid source counts as ''optitrack''); allowed: the Part C frame ' ...
                'sign-off date, yyyy-mm-dd; source: %s'], val_text(dC), val_text(src), C.signoff.partC_src);
        end
    end
end

% ---- 9. F11: frozen pose = parked pose -------------------------------------------------------------------------
af = fld(V, 'arm_freeze_user', []);                 % 1 arm frozen, 0 arm follows the model (flag)
frozen = ~(isnumeric(af) && isscalar(af) && af == 0);   % fail-safe: anything but exactly 0 counts as frozen (logical)
if frozen
    info.items{end+1} = 'park';
    psrc = sprintf(['compares commanded postures only; it does not detect an arm moved by hand, a jog or a power cycle ' ...
        'after calibration; re-calibrate after any of these. Margin %g deg: %s'], ...
        C.park_tol_deg, C.park_tol_deg_src);   % source text of every park reason (char)
    fz = fld(S, 'frozen_fw_deg', []);               % 1x6 frozen joint command (firmware deg)
    rec = fld(S, 'park_record', []);                % newest parked record, or [] (struct)
    fzOk = isnumeric(fz) && numel(fz) == 6 && all(isfinite(fz(:)));   % frozen command usable (logical)
    if ~fzOk
        reasons{end+1} = sprintf('park: frozen command %s; allowed: 6 finite firmware degrees; source: %s', val_text(fz), psrc);
    end
    if isempty(rec)
        reasons{end+1} = sprintf(['park: no parked record; allowed: a record written by calibrateROMEArm ' ...
            '(lab/arm_park_record.m) after the last edit of define_constants; source: %s'], psrc);
    elseif ~(isstruct(rec) && isscalar(rec) && all(isfield(rec, {'fw_deg', 'dc_sha'})))
        reasons{end+1} = sprintf('park: parked record malformed (fields fw_deg, dc_sha needed); allowed: arm_park_record(''read'') output; source: %s', psrc);
    else
        if ~(ischar(rec.dc_sha) && ischar(shaNow) && ~isempty(shaNow) && strcmp(rec.dc_sha, shaNow))
            reasons{end+1} = sprintf(['park: parked record made with define_constants SHA-256 %s, now %s; allowed: equal ' ...
                '(re-run calibrateROMEArm after editing define_constants); source: %s'], val_text(rec.dc_sha), val_text(shaNow), psrc);
        end
        pf = rec.fw_deg;                             % 1x6 parked joint target (firmware deg)
        if fzOk && isnumeric(pf) && numel(pf) == 6
            d = abs(fz(:).' - pf(:).');              % 1x6 |frozen - parked| (firmware deg)
            badJ = find(~(d <= C.park_tol_deg));     % joints over the tolerance (NaN counts as over)
            if ~isempty(badJ)
                parts = arrayfun(@(j) sprintf('J%d frozen %.3f vs parked %.3f (|d| %.3f)', j, fz(j), pf(j), d(j)), badJ, 'UniformOutput', false);
                reasons{end+1} = sprintf(['park: %s firmware deg; allowed: |frozen - parked| <= %g deg on every joint; ' ...
                    'source: %s'], strjoin(parts, ', '), C.park_tol_deg, psrc);
            end
        elseif fzOk
            reasons{end+1} = sprintf('park: parked fw_deg %s; allowed: 6 firmware degrees; source: %s', val_text(pf), psrc);
        end
    end
end

ok = isempty(reasons);
end

% ---------------------------------------------------------------------------------------------------------------
function v = fld(s, name, dflt)
%FLD  s.(name) if s is a struct with that field, else dflt.
if isstruct(s) && isscalar(s) && isfield(s, name), v = s.(name); else, v = dflt; end
end

function t = range_text(T, i)
%RANGE_TEXT  Allowed range of row i of allowed_ranges, as printed in a reason (char).
u = T.unit{i};
switch T.kind{i}
    case 'interval'
        lb = '(';  if T.min_incl(i), lb = '['; end
        rb = ')';  if T.max_incl(i), rb = ']'; end
        t = sprintf('%s%g, %g%s %s', lb, T.min(i), T.max(i), rb, u);
    case 'set'
        t = sprintf('each element one of %s', mat2str(T.allowed{i}{1}));
    case 'equals'
        t = sprintf('equal to %s %s', mat2str(T.allowed{i}{1}, 10), u);
    otherwise
        t = '';
end
sz = T.sz(i);  if iscell(sz), sz = sz{1}; end
if ~isempty(sz), t = sprintf('%s, size %s', t, mat2str(sz)); end
end

function t = val_text(v)
%VAL_TEXT  A value as printed in a reason (char).
if ischar(v)
    if isempty(v), t = '''''(empty)'; else, t = ['''' v '''']; end
elseif isstring(v)
    t = ['"' char(strjoin(v, ' ')) '"'];
elseif (isnumeric(v) || islogical(v)) && ndims(v) <= 2
    if isempty(v), t = '[] (empty)'; else, t = mat2str(v, 6); end
else
    t = sprintf('<%s>', class(v));
end
end

function ok = release_ok(rel, minRel)
%RELEASE_OK  true when release string rel ('2026a' or 'R2026a') is minRel or newer.
ok = false;
if isstring(rel) && isscalar(rel), rel = char(rel); end
if ~ischar(rel), return; end
a = regexp(rel, '^R?(\d{4})([ab])$', 'tokens', 'once');
b = regexp(minRel, '^R?(\d{4})([ab])$', 'tokens', 'once');
if isempty(a) || isempty(b), return; end
ya = str2double(a{1});  yb = str2double(b{1});
ok = ya > yb || (ya == yb && a{2} >= b{2});
end

function ok = date_ok(d)
%DATE_OK  true when d is a valid calendar date 'yyyy-mm-dd' (char).
ok = false;
if isstring(d) && isscalar(d), d = char(d); end
if ~ischar(d), return; end
tk = regexp(d, '^(\d{4})-(\d{2})-(\d{2})$', 'tokens', 'once');
if isempty(tk), return; end
y = str2double(tk{1});  m = str2double(tk{2});  dd = str2double(tk{3});
ok = m >= 1 && m <= 12 && dd >= 1 && dd <= eomday(y, m);
end
