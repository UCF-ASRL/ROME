function test_motive_reader()
%TEST_MOTIVE_READER  The Motive (OptiTrack) reader fails loudly (Plan 2 Task 1).
%   NO NETWORK. Packets are synthetic. They are built here by hand from the NatNet 3.x-4.0
%   frame-of-data layout in docs/verification/NATNET_PACKET_LAYOUT.md sections 3-4, NOT from
%   the parser's layout struct, so a wrong parser offset cannot be hidden by a builder that
%   moves with it. The built packet has one marker set named 'RigidBody 1' (11 characters)
%   with 0 markers and 0 unlabeled markers (S + 12 nO = 16), so the rigid-body ID is at bytes
%   37-40 and x at byte 41 (asserted on the built bytes, check 0).
%     0    builder: ID at bytes 37-40, the 7 floats at bytes 41-68, msgId 7 at bytes 1-2
%     i    a valid packet returns exactly double(single(pose)) as a 7x1 double, ok, why ''
%     ii   message ID 5 (not 7): ok = false, pose7 all NaN, why not empty
%     iii  rigid-body ID 4 when 3 is asked for: ok = false
%     iv   quaternion norm 1.01 and 1 + 2e-3: ok = false; NaN in x, Inf in qw: ok = false;
%          norm 1 + 0.5e-3 is ACCEPTED (so a parser that rejects everything fails here);
%          a packet cut to 60 bytes: ok = false (short packet)
%     v    MotiveDataFake (tests/MotiveDataFake.m, a subclass that overrides the protected
%          openSocket / receiveBytes seam, no socket) fed valid, timeout, timeout, valid(pose 2):
%          pose7 (output 7) = pose 1, pose 1, pose 1, pose 2; valid (8) = [1 0 0 1];
%          nDropped (9) = [0 1 2 0]; pose7 never has a zero element; warning
%          MotiveData:dropout raised on the first timeout and NOT on the second (once per run
%          of rejected frames); the legacy outputs 1-6 (the ones the model is wired to today)
%          are held over the timeouts and follow pose 2 after; the fake never opened a socket
%          (MotiveData's socket property stays empty).
%     vi   a new MotiveDataFake fed timeout, valid, wrong-ID (full-length packet), valid(pose 2):
%          step 1 (no valid frame yet) outputs 1-7 all NaN (none 0), valid 0, nDropped 1, warned;
%          step 2 x = y = 0 exactly (origin = first valid frame), pose7 = pose 1, valid 1,
%          nDropped 0; step 3 the parser-rejected frame: valid 0, nDropped 1, pose7 held at
%          pose 1, warned with the parser's reason ('rigid-body ID'); step 4 pose 2, valid 1.
%     vii  tracked flag (params int16 at bytes 73-74, bit 0x01; PacketClient.cpp L715-719): with
%          trackedOffset 73 a tracked packet is accepted, a valid-pose packet with bit clear is
%          rejected with 'not tracked' (pose NaN); without trackedOffset the same packet is accepted
%          (old behaviour); through MotiveDataFake an untracked frame is held (valid 0, nDropped 1,
%          pose held, warning text 'not tracked'); params 0x0002 rejected, 0x0003 accepted; a 72-byte
%          packet with trackedOffset on is rejected ('needs 74').
%          vii make-it-fail 2: trackedOffset removed from MotiveData's default Layout must make md_held FAIL.
%   Make-it-fail (run on scratch copies, not this file; drivers in ROME_merged/tools/):
%     poseOffset 41 -> 45 in the parser layout below must make (i) FAIL (p2t1_make_mif.py);
%     MotiveData's held pose initialised to zeros instead of NaN must make (vi) FAIL
%     (p2t1_make_mif_heldzeros.py).
%   Errors if any check fails; every check is printed first. Flat, no local functions
%   (CLAUDE.md section 5, crash 2): the packet builder is an anonymous function.
here = fileparts(mfilename('fullpath'));    % ROME_merged/tests
root = fileparts(here);                     % ROME_merged
addpath(here, fullfile(root, 'Optitrack'));
assert(strcmp(fileparts(which('MotiveData')), fullfile(root, 'Optitrack')), 'test_motive_reader:path', ...
       'MotiveData resolves to %s, not <model folder>/Optitrack', which('MotiveData'));
[~, ~, endian] = computer;                  % host byte order ('L' little-endian, 'B' big-endian)
assert(endian == 'L', 'test_motive_reader:endian', 'host is big-endian: typecast-based packets would be byte-swapped');
fails = {};                                 % one message per failed check (cell of char)

% Parser layout under test: 1-based byte index of each field (the documented default, MotiveData.m Layout)
lay = struct('msgIdOffset', 1, 'rbIdOffset', 37, 'poseOffset', 41);

% --- packet builder, hard-coded from NATNET_PACKET_LAYOUT.md section 3 (little-endian) -----------
le16  = @(v) reshape(typecast(uint16(v), 'uint8'), [], 1);           % uint16 -> 2 bytes (uint8 column)
le32i = @(v) reshape(typecast(int32(v), 'uint8'), [], 1);            % int32  -> 4 bytes (uint8 column)
f32   = @(v) reshape(typecast(single(v(:).'), 'uint8'), [], 1);      % float32 vector -> 4 bytes each (uint8 column)
msName = uint8(['RigidBody 1' 0]).';        % marker-set name, 11 characters + NUL = 12 bytes
nData  = 78;                                % nDataBytes field: bytes after the 4-byte header (count of the fields below)
build = @(msg, id, pose) [le16(msg); le16(nData); le32i(1234); ...   % iMessage, nDataBytes, frame number
                          le32i(1); msName; le32i(0); ...             % nMarkerSets = 1, name, nMarkers = 0  (S = 16 bytes)
                          le32i(0); ...                               % nOtherMarkers = 0 (nO = 0)
                          le32i(1); ...                               % nRigidBodies = 1
                          le32i(id); f32(pose); ...                   % ID, x y z qx qy qz qw
                          f32(0.0005); le16(1); ...                   % mean marker error (m), params (bit 0 = tracked)
                          zeros(8, 1, 'uint8')];                      % two zero int32 counts after the bodies (not parsed)

rbID = 3;                                   % Streaming ID asked for in the tests (not the default 1, -)
q1 = [0.1; -0.2; 0.3];                      % vector part of quaternion 1 (-)
p1 = [0.123456789; -1.987654321; 0.314159265; q1; sqrt(1 - sum(q1.^2))];   % pose 1: x y z (Motive units, m) qx qy qz qw (-)
q2 = [-0.05; 0.15; 0.7];                    % vector part of quaternion 2 (-)
p2 = [1.111111111; 0.777777777; 0.271828183; q2; sqrt(1 - sum(q2.^2))];   % pose 2, every element differs from pose 1
e1 = double(single(p1));                    % expected parser output for pose 1 (single -> double, 7x1)
e2 = double(single(p2));                    % expected parser output for pose 2
assert(all(e1 ~= p1), 'test_motive_reader:repr', 'pose 1 must not be exactly representable in single');
pk1 = build(7, rbID, p1);                   % valid packet, pose 1 (uint8 column)
pk2 = build(7, rbID, p2);                   % valid packet, pose 2

%% 0  the builder puts the fields where the layout document says
c0 = isa(pk1, 'uint8') && iscolumn(pk1) && numel(pk1) == 4 + nData ...   % verdict of check 0 (logical)
     && isequal(pk1(1:2), le16(7)) && isequal(pk1(37:40), le32i(rbID)) && isequal(pk1(41:68), f32(p1));
fprintf('0    builder: %d bytes, msgId@1-2, ID@37-40, pose@41-68 -> %s\n', numel(pk1), mat2str(c0));
if ~c0, fails{end+1} = '0: the packet builder does not match the NatNet layout (msgId@1-2, ID@37-40, pose@41-68)'; end

%% i  valid packet
[p, ok, why] = natnet_parse_rigidbody(pk1, rbID, lay);   % p: 7x1 pose (Motive units / -) or NaN; ok: logical; why: char
ci = ok && isa(p, 'double') && isequal(size(p), [7 1]) && isequal(p, e1) && isempty(why);   % verdict of check i (logical)
fprintf('i    valid packet: ok=%d, max|pose7 - double(single(pose))| = %g, why=''%s'' -> %s\n', ok, max(abs(p - e1)), why, mat2str(ci));
if ~ci, fails{end+1} = sprintf('i: valid packet parsed to %s (ok %d, why ''%s''), expected %s', mat2str(p.', 10), ok, why, mat2str(e1.', 10)); end

%% ii  message ID not 7
[p, ok, why] = natnet_parse_rigidbody(build(5, rbID, p1), rbID, lay);   % parser outputs, as in check i
cii = ~ok && all(isnan(p)) && ~isempty(why);   % verdict of check ii (logical)
fprintf('ii   msgId 5: ok=%d, why=''%s'' -> %s\n', ok, why, mat2str(cii));
if ~cii, fails{end+1} = sprintf('ii: msgId 5 gave ok %d, why ''%s''', ok, why); end

%% iii  rigid-body ID not the one asked for
[p, ok, why] = natnet_parse_rigidbody(build(7, rbID + 1, p1), rbID, lay);   % parser outputs, as in check i
ciii = ~ok && all(isnan(p)) && ~isempty(why);   % verdict of check iii (logical)
fprintf('iii  rigid-body ID %d, asked %d: ok=%d, why=''%s'' -> %s\n', rbID + 1, rbID, ok, why, mat2str(ciii));
if ~ciii, fails{end+1} = sprintf('iii: wrong rigid-body ID gave ok %d, why ''%s''', ok, why); end

%% iv  quaternion norm, non-finite values, short packet
% case name, pose fed to the builder, ok expected
pq = @(s) [p1(1:3); s * p1(4:7)];           % pose 1 with its quaternion scaled by s (norm = s, -)
ivc = {'norm 1.01',        pq(1.01),                  false;
       'norm 1+2e-3',      pq(1 + 2e-3),              false;
       'norm 1+0.5e-3',    pq(1 + 0.5e-3),            true;
       'x = NaN',          [NaN; p1(2:7)],            false;
       'qw = Inf',         [p1(1:6); Inf],            false};
for r = 1:size(ivc, 1)
    [p, ok, why] = natnet_parse_rigidbody(build(7, rbID, ivc{r,2}), rbID, lay);   % parser outputs, as in check i
    civ = (ok == ivc{r,3}) && (ok || all(isnan(p)));   % verdict of this check iv case (logical)
    fprintf('iv   %-14s ok=%d (expected %d), why=''%s'' -> %s\n', ivc{r,1}, ok, ivc{r,3}, why, mat2str(civ));
    if ~civ, fails{end+1} = sprintf('iv %s: ok %d, expected %d (why ''%s'')', ivc{r,1}, ok, ivc{r,3}, why); end %#ok<AGROW>
end
[p, ok, why] = natnet_parse_rigidbody(pk1(1:60), rbID, lay);   % parser outputs, as in check i
civs = ~ok && all(isnan(p)) && ~isempty(why);   % verdict of the short-packet check (logical)
fprintf('iv   60-byte packet: ok=%d, why=''%s'' -> %s\n', ok, why, mat2str(civs));
if ~civs, fails{end+1} = sprintf('iv short: 60-byte packet gave ok %d, why ''%s''', ok, why); end

%% v  MotiveData through the test seam: valid, timeout, timeout, valid(pose 2)
seq = {pk1, [], [], pk2};                   % queued packets; [] = receive timeout
md = MotiveDataFake(seq, 'RigidBodyID', rbID);   % MotiveData under test, network seam replaced (System object)
N = numel(seq);                             % number of steps
P  = zeros(7, N);                           % pose7 per step (Motive units)
V  = zeros(1, N);                           % valid per step (0/1)
D  = zeros(1, N);                           % nDropped per step (consecutive invalid frames)
L6 = zeros(6, N);                           % legacy outputs per step: x y (m, minus origin), z (m), Euler XYZ (rad)
lastwarn('');
wid = cell(1, N);                           % warning identifier raised during each step (char)
ws = warning('on', 'MotiveData:dropout');   % must stay on: a disabled warning does not update lastwarn
for k = 1:N
    lastwarn('');
    [L6(1,k), L6(2,k), L6(3,k), L6(4,k), L6(5,k), L6(6,k), P(:,k), V(k), D(k)] = md(1);
    [~, wid{k}] = lastwarn;
end
warning(ws);
noSocket = md.socketIsEmpty();              % true if MotiveData never created a socket
release(md);
eul1 = quat2eul(e1([7 4 5 6]).', 'XYZ');    % Euler XYZ of pose 1 (rad)
eul2 = quat2eul(e2([7 4 5 6]).', 'XYZ');    % Euler XYZ of pose 2 (rad)
L1 = [0; 0; e1(3); eul1(:)];                % legacy outputs at step 1: origin = pose 1, so x = y = 0 by design
L2 = [e2(1) - e1(1); e2(2) - e1(2); e2(3); eul2(:)];   % legacy outputs at step 4
cv = struct( ...                            % verdict of each (v) sub-check (struct of logical)
    'pose7_held',   isequal(P(:,1), e1) && isequal(P(:,2), e1) && isequal(P(:,3), e1), ...
    'pose7_update', isequal(P(:,4), e2), ...
    'valid',        isequal(V, [1 0 0 1]), ...
    'nDropped',     isequal(D, [0 1 2 0]), ...
    'never_zero',   all(P(:) ~= 0), ...
    'warned',       strcmp(wid{2}, 'MotiveData:dropout'), ...
    'warned_once',  isempty(wid{1}) && isempty(wid{3}) && isempty(wid{4}), ...
    'legacy_held',  isequal(L6(:,2), L6(:,1)) && isequal(L6(:,3), L6(:,1)), ...
    'legacy_vals',  max(abs(L6(:,1) - L1)) < 1e-12 && max(abs(L6(:,4) - L2)) < 1e-12, ...
    'no_socket',    noSocket);
fn = fieldnames(cv);                        % names of the (v) sub-checks
fprintf('v    valid=%s nDropped=%s warning@2=''%s''\n', mat2str(V), mat2str(D), wid{2});
for f = 1:numel(fn)
    fprintf('v    %-13s -> %s\n', fn{f}, mat2str(cv.(fn{f})));
    if ~cv.(fn{f}), fails{end+1} = sprintf('v %s failed', fn{f}); end %#ok<AGROW>
end

%% vi  before the first valid frame, and a parser-rejected frame through MotiveData
pkBad = build(7, rbID + 1, p2);             % full-length frame of data with the wrong rigid-body ID (uint8 column)
seq6 = {[], pk1, pkBad, pk2};               % queued packets; [] = receive timeout
md6 = MotiveDataFake(seq6, 'RigidBodyID', rbID);   % fresh MotiveData under test, network seam replaced (System object)
N6  = numel(seq6);                          % number of steps
P6  = zeros(7, N6);                         % pose7 per step (Motive units / -)
V6  = zeros(1, N6);                         % valid per step (0/1)
D6  = zeros(1, N6);                         % nDropped per step (consecutive rejected frames)
L66 = zeros(6, N6);                         % legacy outputs 1-6 per step: x y (minus origin), z (Motive units), Euler XYZ (rad)
wid6 = cell(1, N6);                         % warning identifier raised during each step (char)
wmsg6 = cell(1, N6);                        % warning message raised during each step (char)
ws = warning('on', 'MotiveData:dropout');   % must stay on: a disabled warning does not update lastwarn
for k = 1:N6
    lastwarn('');
    [L66(1,k), L66(2,k), L66(3,k), L66(4,k), L66(5,k), L66(6,k), P6(:,k), V6(k), D6(k)] = md6(1);
    [wmsg6{k}, wid6{k}] = lastwarn;
end
warning(ws);
noSocket6 = md6.socketIsEmpty();            % true if MotiveData never created a socket
release(md6);
cvi = struct( ...                           % verdict of each (vi) sub-check (struct of logical)
    'step1_all_nan', all(isnan(L66(:,1))) && all(isnan(P6(:,1))) && V6(1) == 0 && D6(1) == 1 && strcmp(wid6{1}, 'MotiveData:dropout'), ...
    'step2_origin',  L66(1,2) == 0 && L66(2,2) == 0 && isequal(P6(:,2), e1) && V6(2) == 1 && D6(2) == 0 && isempty(wid6{2}), ...
    'step3_bad_id',  V6(3) == 0 && D6(3) == 1 && isequal(P6(:,3), e1) && isequal(L66(:,3), L66(:,2)) ...
                     && strcmp(wid6{3}, 'MotiveData:dropout') && contains(wmsg6{3}, 'rigid-body ID'), ...
    'step4_update',  isequal(P6(:,4), e2) && V6(4) == 1 && D6(4) == 0 && max(abs(L66(:,4) - L2)) < 1e-12, ...
    'no_socket',     noSocket6);
fn6 = fieldnames(cvi);                      % names of the (vi) sub-checks
fprintf('vi   step 1 outputs 1-7 = %s; valid=%s nDropped=%s; step 2 x=%g y=%g; warning@3=''%s''\n', ...
        mat2str([L66(:,1); P6(:,1)].', 4), mat2str(V6), mat2str(D6), L66(1,2), L66(2,2), wmsg6{3});
for f = 1:numel(fn6)
    fprintf('vi   %-14s -> %s\n', fn6{f}, mat2str(cvi.(fn6{f})));
    if ~cvi.(fn6{f}), fails{end+1} = sprintf('vi %s failed', fn6{f}); end %#ok<AGROW>
end

%% vii  tracked flag: packet params word at bytes 73-74 (NATNET_PACKET_LAYOUT.md section 3 rows 'mean marker error', 'params')
layT = struct('msgIdOffset', 1, 'rbIdOffset', 37, 'poseOffset', 41, 'trackedOffset', 73);   % layout with the tracked check on
pkU = pk1;                                  % valid pose, params = 0 (untracked): bytes 73-74 cleared (uint8 column)
pkU(73:74) = [0; 0];
assert(isequal(pk1(73:74), le16(1)) && isequal(pkU(1:72), pk1(1:72)), 'test_motive_reader:vii', 'tracked-flag bytes are not at 73-74');
[pT, okT, whyT] = natnet_parse_rigidbody(pk1, rbID, layT);   % tracked packet
[pU, okU, whyU] = natnet_parse_rigidbody(pkU, rbID, layT);   % untracked packet
[pO, okO, ~]    = natnet_parse_rigidbody(pkU, rbID, lay);    % untracked packet, layout without trackedOffset (check off)
pkU2 = pk1; pkU2(73:74) = typecast(uint16(2), 'uint8').';   % params 0x0002: bit 0 clear, other bit set (uint8 column)
pkT3 = pk1; pkT3(73:74) = typecast(uint16(3), 'uint8').';   % params 0x0003: bit 0 set, other bit set
[~, okU2, whyU2] = natnet_parse_rigidbody(pkU2, rbID, layT);   % must be rejected
[~, okT3, ~]     = natnet_parse_rigidbody(pkT3, rbID, layT);   % must be accepted
[~, okS, whyS]   = natnet_parse_rigidbody(pk1(1:72), rbID, layT);   % 72 bytes, tracked flag out of range
seq7 = {pk1, pkU, pk2};                     % tracked, untracked, tracked through MotiveData (default Layout has trackedOffset)
md7 = MotiveDataFake(seq7, 'RigidBodyID', rbID);   % MotiveData under test, network seam replaced
P7 = zeros(7, 3); V7 = zeros(1, 3); D7 = zeros(1, 3); wm7 = cell(1, 3);   % pose7, valid, nDropped, warning text per step
ws = warning('on', 'MotiveData:dropout');
for k = 1:3
    lastwarn('');
    [~, ~, ~, ~, ~, ~, P7(:,k), V7(k), D7(k)] = md7(1);
    wm7{k} = lastwarn;
end
warning(ws);
release(md7);
cvii = struct( ...                          % verdict of each (vii) sub-check (struct of logical)
    'tracked_ok',     okT && isempty(whyT) && isequal(pT, e1), ...
    'untracked_rej',  ~okU && all(isnan(pU)) && contains(whyU, 'not tracked'), ...
    'off_without_offset', okO, ...
    'params_0x0002_rej', ~okU2 && contains(whyU2, 'not tracked'), ...
    'params_0x0003_ok',  okT3, ...
    'short_needs_74',    ~okS && contains(whyS, 'needs 74'), ...
    'md_held',        isequal(V7, [1 0 1]) && isequal(D7, [0 1 0]) && isequal(P7(:,2), e1) && isequal(P7(:,3), e2) && contains(wm7{2}, 'not tracked'));
fn7 = fieldnames(cvii);                     % names of the (vii) sub-checks
for f = 1:numel(fn7)
    fprintf('vii  %-18s -> %s\n', fn7{f}, mat2str(cvii.(fn7{f})));
    if ~cvii.(fn7{f}), fails{end+1} = sprintf('vii %s failed (why ''%s'')', fn7{f}, whyU); end %#ok<AGROW>
end

if ~isempty(fails)
    error('test_motive_reader:fail', 'test_motive_reader FAILED (%d):\n  %s', numel(fails), strjoin(fails, '\n  '));
end
fprintf('test_motive_reader PASS: checks 0, i, ii, iii, iv (6 cases), v (%d sub-checks), vi (%d sub-checks), vii (%d sub-checks)\n', numel(fn), numel(fn6), numel(fn7));
end
