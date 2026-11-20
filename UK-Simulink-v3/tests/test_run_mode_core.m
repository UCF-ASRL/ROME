function test_run_mode_core()
%TEST_RUN_MODE_CORE  Offline test of run_mode_table.m and set_run_mode.m (focus plan F12, pure core).
%   Pure: no model, no base workspace, no network, and the real define_constants.m is never written (every edit is
%   on a SYNTHETIC define_constants-like text built here, in temp folders). Errors with id test_run_mode_core:fail
%   and a message 'FAIL: (<check>) ...' on the first failing check; prints 'test_run_mode_core PASS' at the end.
%   Checks:
%     (i)    run_mode_table(1..4) -> exactly the flags of the plan's FOUR MODES table and the banner strings, field
%            names and classes exact; single(3) and int8(2) accepted as 3 and 2 (M.n double)
%     (ii)   every bad value refused with run_mode_table:bad, the message listing all four banners: [], [1 2], NaN,
%            Inf, -Inf, 2.5, '2', "2", 0, 5, -1, true, 2+1i, {2}, struct, int8(7); and set_run_mode with a bad n
%            leaves the file byte-identical, no temp file left
%     (iii)  set_run_mode on four synthetic variants (CRLF; LF; LF with the run_mode line LAST and no final newline;
%            CRLF with the run_mode line FIRST), modes 2, 3, 4, 1 in turn: first the line-ending counts (CR and LF
%            bytes) are unchanged ('(iii) line endings'), then the whole file equals the original with only the
%            digit replaced (oracle built independently here, '(iii) bytes'), the returned old mode, the printed
%            banner, no temp file left, the file never executed (it ends in error('synthetic:ran')); a second
%            call with the same mode leaves the bytes identical and writes nothing ('(iii) idempotent')
%     (iv)   refused with set_run_mode:count, file byte-identical, no temp file left: 0 run_mode lines; 2 lines;
%            1 line + an indented assignment; 1 line + 'run_mode = 2.5;'; only a line split 'run_mode =<EOL>3;'.
%            A missing file -> set_run_mode:read
%     (v)    set_run_mode(n) with no file argument edits define_constants.m in ITS OWN folder (a temp folder holding
%            copies of set_run_mode.m and run_mode_table.m, resolved first on the path); the real
%            define_constants.m next to the code under test is byte-identical before and after
%     (vi)   set_run_mode.m holds no flag name or camera source in its code (the mapping lives in run_mode_table)
%     (vii)  with define_constants.PRISTINE next to the file (a harness holds it, CLAUDE.md 20.4), set_run_mode is
%            refused with set_run_mode:locked, the file is byte-identical and no temp file is left (fix round 1)
%   Make-it-fail: tests/manual_makefail_run_mode_core.m.
root = fileparts(fileparts(mfilename('fullpath')));       % ROME_merged (or its temp mirror in the make-it-fail harness)
p0 = path;  cln = onCleanup(@() path(p0));
addpath(root);
chk('(setup)', startsWith(which('set_run_mode'), root) && startsWith(which('run_mode_table'), root), ...
    'code under test not resolved in %s: %s | %s', root, which('set_run_mode'), which('run_mode_table'));
tmpd = tempname;  mkdir(tmpd);  cln2 = onCleanup(@() rmdir(tmpd, 's'));

% ---- (i) every mode -> the exact flags ----
W = {  % n | EnableHardware | EnableMotive | motive_source | arm_freeze | banner   (plan, THE FOUR MODES; author's words)
  1, 0, 1, 'emulator',  1, 'MODE 1: Simulation only (end-effector dynamics sim, pose data from sim)';
  2, 1, 1, 'emulator',  1, 'MODE 2: ground vehicle moving, frozen arm, camera off (fake camera)';
  3, 1, 1, 'emulator',  0, 'MODE 3: ground vehicle moving, moving arm, camera off (fake camera)';
  4, 1, 1, 'optitrack', 0, 'MODE 4: ground vehicle moving, moving arm, camera on (OptiTrack)'};
fn = {'n', 'EnableHardware', 'EnableMotive', 'motive_source', 'arm_freeze', 'banner'};
for r = 1:4
    M = run_mode_table(W{r,1});
    got = fieldnames(M).';                                % field names (cell row)
    chk('(i)', isequal(got, fn), 'mode %d: fields %s', W{r,1}, strjoin(got, ','));
    chk('(i)', isa(M.n, 'double') && M.n == W{r,1}, 'mode %d: n = %s', W{r,1}, mat2str(M.n));
    chk('(i)', isequal(M.EnableHardware, W{r,2}) && isa(M.EnableHardware, 'double'), 'mode %d: EnableHardware %s, want %d', W{r,1}, mat2str(M.EnableHardware), W{r,2});
    chk('(i)', isequal(M.EnableMotive, W{r,3}) && isa(M.EnableMotive, 'double'), 'mode %d: EnableMotive %s, want %d', W{r,1}, mat2str(M.EnableMotive), W{r,3});
    chk('(i)', ischar(M.motive_source) && strcmp(M.motive_source, W{r,4}), 'mode %d: motive_source %s, want %s', W{r,1}, M.motive_source, W{r,4});
    chk('(i)', isequal(M.arm_freeze, W{r,5}) && isa(M.arm_freeze, 'double'), 'mode %d: arm_freeze %s, want %d', W{r,1}, mat2str(M.arm_freeze), W{r,5});
    chk('(i)', ischar(M.banner) && strcmp(M.banner, W{r,6}), 'mode %d: banner "%s", want "%s"', W{r,1}, M.banner, W{r,6});
    chk('(i)', ~contains(M.banner, 'movinb'), 'mode %d: banner carries the typo "movinb"', W{r,1});
end
M3 = run_mode_table(single(3));  M2 = run_mode_table(int8(2));
chk('(i)', M3.n == 3 && isa(M3.n, 'double') && M3.arm_freeze == 0, 'single(3) not accepted as mode 3');
chk('(i)', M2.n == 2 && isa(M2.n, 'double') && M2.arm_freeze == 1, 'int8(2) not accepted as mode 2');
fprintf('(i)    PASS  modes 1-4 give the exact flags and banners of the plan table\n');

% ---- (ii) every bad value refused ----
bad = {[], [1 2], NaN, Inf, -Inf, 2.5, '2', "2", 0, 5, -1, true, 2+1i, {2}, struct('n', 2), int8(7)};
for i = 1:numel(bad)
    [id, msg] = err_of(@() run_mode_table(bad{i}));
    chk('(ii)', strcmp(id, 'run_mode_table:bad'), 'value #%d (%s) not refused with run_mode_table:bad (got ''%s'')', i, class(bad{i}), id);
    for r = 1:4
        chk('(ii)', contains(msg, W{r,6}), 'value #%d: the message does not list "%s": %s', i, W{r,6}, msg);
    end
end
[orig, ~] = synth(char([13 10]), true, 'middle', '1');
D = fresh_dir(tmpd, 'ii');  f = fullfile(D, 'define_constants.m');  put(f, orig);
for v = {0, 5, 2.5, '2', NaN}
    id = err_of(@() set_run_mode(v{1}, f));
    chk('(ii)', strcmp(id, 'run_mode_table:bad'), 'set_run_mode(%s) not refused with run_mode_table:bad (got ''%s'')', mat2str(v{1}), id);
    chk('(ii)', isequal(get_bytes(f), orig), 'set_run_mode(%s): the file changed', mat2str(v{1}));
    chk('(ii)', isequal(listing(D), {'define_constants.m'}), 'set_run_mode(%s): folder now holds %s', mat2str(v{1}), strjoin(listing(D), ', '));
end
fprintf('(ii)   PASS  %d bad values refused, message lists the four modes; a bad n leaves the file untouched\n', numel(bad));

% ---- (iii) set_run_mode on synthetic files ----
V = {  % label | EOL | final EOL | where the run_mode line is
  'CRLF',                char([13 10]), true,  'middle';
  'LF',                  char(10),      true,  'middle';
  'LF last-line no EOL', char(10),      false, 'last';
  'CRLF first-line',     char([13 10]), true,  'first'};
for vi = 1:size(V, 1)
    [orig, mk] = synth(V{vi,2}, V{vi,3}, V{vi,4}, '1');   % original bytes (run_mode = 1) and the oracle builder
    D = fresh_dir(tmpd, sprintf('iii_%d', vi));  f = fullfile(D, 'define_constants.m');  put(f, orig);
    prev = 1;
    for k = [2 3 4 1]
        [id, out, Mk, oldk] = call_srm(k, f);
        chk('(iii)', ~strcmp(id, 'synthetic:ran'), '%s mode %d: set_run_mode EXECUTED the file', V{vi,1}, k);
        chk('(iii)', isempty(id), '%s mode %d: set_run_mode errored: %s', V{vi,1}, k, id);
        got = get_bytes(f);
        chk('(iii) line endings', sum(got == 13) == sum(orig == 13) && sum(got == 10) == sum(orig == 10), ...
            '%s mode %d: CR/LF bytes %d/%d, original %d/%d', V{vi,1}, k, sum(got == 13), sum(got == 10), sum(orig == 13), sum(orig == 10));
        chk('(iii) bytes', isequal(got, mk(sprintf('%d', k))), '%s mode %d: bytes other than the run_mode digit changed (first difference at byte %d)', ...
            V{vi,1}, k, first_diff(got, mk(sprintf('%d', k))));
        chk('(iii)', Mk.n == k && oldk == prev, '%s mode %d: returned n %g, old %g (want %d, %d)', V{vi,1}, k, Mk.n, oldk, k, prev);
        chk('(iii)', contains(out, [W{k,6} newline]), '%s mode %d: banner "%s" not printed; output: %s', V{vi,1}, k, W{k,6}, out);
        chk('(iii)', isequal(listing(D), {'define_constants.m'}), '%s mode %d: folder holds %s', V{vi,1}, k, strjoin(listing(D), ', '));
        prev = k;
    end
    b1 = get_bytes(f);  d1 = dir(f);
    [id, out] = call_srm(1, f);                           % same mode again
    chk('(iii) idempotent', isempty(id) && isequal(get_bytes(f), b1) && isequal(get_bytes(f), orig), '%s: a second set_run_mode(1) changed the bytes or errored (%s)', V{vi,1}, id);
    d2 = dir(f);
    chk('(iii) idempotent', contains(out, 'unchanged') && isequal(d2.datenum, d1.datenum), '%s: a second set_run_mode(1) rewrote the file', V{vi,1});
end
fprintf('(iii)  PASS  4 variants x 4 modes: only the digit changes, CRLF/LF kept, banner printed, idempotent, never executed\n');

% ---- (iv) 0 or 2 run_mode lines refused, file untouched ----
EL = char(10);
[base1, ~] = synth(EL, true, 'middle', '1');
t1 = char(base1);
R = {  % label | file text | expected error id
  '0 lines',               strrep(t1, ['run_mode = 1;'], '% (no run_mode line)'),                    'set_run_mode:count';
  '2 lines',               [t1 'run_mode = 2;' EL],                                                  'set_run_mode:count';
  '1 + indented',          [t1 '  run_mode = 2;' EL],                                                'set_run_mode:count';
  '1 + non-integer',       [t1 'run_mode = 2.5;' EL],                                                'set_run_mode:count';
  'split line only',       strrep(t1, 'run_mode = 1;', ['run_mode =' EL '1;']),                      'set_run_mode:count'};
chk('(iv)', numel(strfind(t1, 'run_mode = 1;')) == 1, 'synthetic base does not hold exactly one ''run_mode = 1;''');
for i = 1:size(R, 1)
    D = fresh_dir(tmpd, sprintf('iv_%d', i));  f = fullfile(D, 'define_constants.m');  b0 = uint8(R{i,2});  put(f, b0);
    id = err_of(@() set_run_mode(3, f));
    chk('(iv)', strcmp(id, R{i,3}), '%s: not refused with %s (got ''%s'')', R{i,1}, R{i,3}, id);
    chk('(iv)', isequal(get_bytes(f), b0), '%s: the file changed', R{i,1});
    chk('(iv)', isequal(listing(D), {'define_constants.m'}), '%s: folder holds %s', R{i,1}, strjoin(listing(D), ', '));
end
id = err_of(@() set_run_mode(2, fullfile(tmpd, 'no_such_define_constants.m')));
chk('(iv)', strcmp(id, 'set_run_mode:read'), 'missing file: got ''%s'', want set_run_mode:read', id);
fprintf('(iv)   PASS  %d malformed files refused with set_run_mode:count and left untouched; missing file refused\n', size(R, 1));

% ---- (vii) custody lock ----
[orig7, ~] = synth(EL, true, 'middle', '1');
D = fresh_dir(tmpd, 'vii');  f = fullfile(D, 'define_constants.m');  put(f, orig7);  put(fullfile(D, 'define_constants.PRISTINE'), orig7);
id = err_of(@() set_run_mode(3, f));
chk('(vii)', strcmp(id, 'set_run_mode:locked'), 'define_constants.PRISTINE present: got ''%s'', want set_run_mode:locked', id);
chk('(vii)', isequal(get_bytes(f), orig7), 'define_constants.PRISTINE present: the file changed');
chk('(vii)', isequal(listing(D), {'define_constants.PRISTINE', 'define_constants.m'}), 'folder holds %s', strjoin(listing(D), ', '));
fprintf('(vii)  PASS  refused with set_run_mode:locked while define_constants.PRISTINE exists; file untouched\n');

% ---- (v) default file = define_constants.m next to set_run_mode.m ----
realDc = fullfile(root, 'define_constants.m');            % the real file next to the code under test (char)
hasReal = isfile(realDc);
if hasReal, real0 = get_bytes(realDc); end
D = fresh_dir(tmpd, 'v');
copyfile(fullfile(root, 'set_run_mode.m'), D);  copyfile(fullfile(root, 'run_mode_table.m'), D);
[orig, mk] = synth(EL, true, 'middle', '1');  put(fullfile(D, 'define_constants.m'), orig);
here0 = pwd;                                              % current folder (char): it outranks the path, so cd to D too
cln3 = onCleanup(@() cd(here0));                          % (test_suite runs every step from ROME_merged itself)
addpath(D);  cd(D);
chk('(setup)', startsWith(which('set_run_mode'), D), 'temp copy of set_run_mode not first on the path (%s); not calling it', which('set_run_mode'));
[id, ~] = call_srm(3, []);
cd(here0);  rmpath(D);
if hasReal       % not restored automatically: a concurrent edit by someone else must not be overwritten
    chk('(v)', isequal(get_bytes(realDc), real0), ['the REAL %s changed during set_run_mode(3) with no file argument ' ...
        '(or was edited concurrently); NOT restored, inspect it'], realDc);
end
chk('(v)', isempty(id), 'set_run_mode(3) with no file argument errored: %s', id);
chk('(v)', isequal(get_bytes(fullfile(D, 'define_constants.m')), mk('3')), 'set_run_mode(3) did not edit define_constants.m in its own folder');
fprintf('(v)    PASS  set_run_mode(n) edits define_constants.m in its own folder; real file untouched (%s)\n', ...
    ternary(hasReal, 'compared', 'absent in this tree'));

% ---- (vi) no mapping in set_run_mode.m ----
src = strsplit(fileread(fullfile(root, 'set_run_mode.m')), newline);
code = regexprep(src, '%.*$', '');                        % naive comment strip (cuts at the first %, also inside strings)
hit = ~cellfun(@isempty, regexp(code, 'emulator|optitrack|EnableHardware|EnableMotive|arm_freeze|motive_source', 'once'));
chk('(vi)', ~any(hit), 'set_run_mode.m code mentions a flag or a camera source (line %s)', mat2str(find(hit)));
fprintf('(vi)   PASS  set_run_mode.m carries no flag value; the mapping is in run_mode_table.m only\n');

fprintf('test_run_mode_core PASS\n');
end

% ---------------------------------------------------------------------------------------------------------------
function [b, mk] = synth(eol, finalEol, where, digit)
%SYNTH  Synthetic define_constants-like file: bytes b (uint8 row) with 'run_mode = <digit>;' at 'first' | 'middle'
%   | 'last', lines joined by eol, a final eol if finalEol. mk(d) builds the same file with digit d (oracle).
pre  = 'run_mode = ';
post = ';          % 1-4, the author''s four modes (run_mode_table.m)';
L = {'%% ========================================================================'
     '%  EDIT THIS BLOCK. NOTHING ELSE IN THIS FILE NEEDS TOUCHING. (SYNTHETIC copy, test_run_mode_core)'
     ''
     'scenario = 1;          % 1 ellipse | 2 V-bar | 3 R-bar | 4 NMC | 5 circle'
     ['% run_mode = 3;   <- a comment, must not count; J2 51.4' char([194 176]) ' (UTF-8 degree sign, two bytes)']
     'max_rpm = 80;          % trailing spaces follow   '
     [char(9) 'arm_fw_margin_deg = 2.0;   % tab-indented']
     ''
     '%% ========================================================================'
     '%  Below here is the formulation. Leave it alone.'
     'run_mode_label = ''x'';   % starts with run_mode but is another name'
     'M = run_mode_table(run_mode);'
     'fprintf(''run_mode = %d;\n'', run_mode);'
     'error(''synthetic:ran'', ''this synthetic define_constants must never be executed by set_run_mode'');'};
switch where
    case 'first',  L = [{'@RUN@'}; L];
    case 'middle', L = [L(1:4); {'@RUN@'}; L(5:end)];
    case 'last',   L = [L; {'@RUN@'}];
end
whole = strjoin(L.', eol);
if finalEol, whole = [whole eol]; end
i = strfind(whole, '@RUN@');
assert(numel(i) == 1, 'test_run_mode_core:synth', 'marker count %d', numel(i));
before = whole(1:i-1);  after = whole(i+5:end);
mk = @(d) uint8([before pre d post after]);
b = mk(digit);
end

function D = fresh_dir(tmpd, name)
D = fullfile(tmpd, name);  mkdir(D);
end

function put(f, b)
fid = fopen(f, 'w');  fwrite(fid, uint8(b), 'uint8');  fclose(fid);
end

function b = get_bytes(f)
fid = fopen(f, 'r');  b = fread(fid, Inf, '*uint8').';  fclose(fid);
end

function c = listing(D)
%LISTING  Sorted names in folder D, without . and .. (cell row).
d = dir(D);  c = sort({d(~ismember({d.name}, {'.', '..'})).name});
end

function [id, out, M, old] = call_srm(k, f)
%CALL_SRM  set_run_mode(k, f) (or set_run_mode(k) when f is empty) with its printed output captured.
id = '';  out = '';  M = struct('n', NaN);  old = NaN;
try
    if isempty(f), out = evalc('[M, old] = set_run_mode(k);');
    else,          out = evalc('[M, old] = set_run_mode(k, f);'); end
catch err
    id = err.identifier;  if isempty(id), id = ['(no id) ' err.message]; end
end
end

function [id, msg] = err_of(fun)
id = '';  msg = '';
try, fun(); catch err, id = err.identifier;  msg = err.message;  if isempty(id), id = ['(no id) ' err.message]; end, end
end

function i = first_diff(a, b)
n = min(numel(a), numel(b));  i = find(a(1:n) ~= b(1:n), 1);
if isempty(i), i = n + 1; end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end

function chk(check, cond, fmt, varargin)
if ~cond
    error('test_run_mode_core:fail', ['FAIL: %s ' fmt], check, varargin{:});
end
end
