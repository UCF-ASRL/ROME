function manual_makefail_f12(ids)
%MANUAL_MAKEFAIL_F12  Make-it-fail campaign of focus plan F12 wiring (and two F7 re-review items), one run each.
%   Part 1  every break of test_run_mode('list') (checks a-f): test_run_mode('<id>') must error test_run_mode:fail
%           naming its check (the test itself verifies the named check is among the failed ones).
%   Part 2  wrong anchor fails loudly: a temp mirror of ROME_merged whose define_constants.m has 'run_mode = 1.0;'
%           (not the count-asserted form); the mirror's tests/test_run_length must error dc_set_mode:count (before F12
%           its regexprep(...,'once') edit of 'EnableHardware = \d;' matched nothing and the test ran on silently).
%   Part 3  tools/wire_motive_emulator.m re-read assertions (F7 re-review): on a temp copy, W0 the unmodified tool must
%           pass (control); W1 the tool builds JointFromTelemetry with Value 'double(EnableHardware ~= 0)' and must
%           error wire_motive_emulator:verify naming 'F7 block parameters differ'; W2 the tool re-feeds StateSource
%           input 2 from JointFromTelemetry instead of UseMotive and must error wire_motive_emulator:verify naming
%           'StateSource in 2'.
%   manual_makefail_f12(ids)  runs only the listed test_run_mode breaks (cellstr) and skips parts 2 and 3 (fix rounds).
%   Errors at the end (manual_makefail_f12:result) unless every item behaved as required. Never writes ROME_merged.
%   Flat function without local functions (CLAUDE.md section 5, crash 2).
here = fileparts(mfilename('fullpath'));      % ROME_merged/tests (char)
root = fileparts(here);                       % ROME_merged (char)
mdl = 'ROME_9DOF';
res = {};                                     % {item, verdict, detail} rows
startdir = pwd;  path0 = path;

% ---- part 1 -----------------------------------------------------------------------------------------------------------
B = test_run_mode('list');
only = nargin >= 1 && ~isempty(ids);          % a subset of part 1 only (logical)
if only
    keep = ismember(B(:,1), ids);
    assert(sum(keep) == numel(ids), 'manual_makefail_f12:ids', 'unknown break id among %s', strjoin(ids, ', '));
    B = B(keep, :);
end
for k = 1:size(B, 1)
    fprintf('\n================ test_run_mode break %s (check %s must FAIL) ================\n', B{k,1}, B{k,2});
    v = 'DID NOT FAIL';  d = '';
    try
        test_run_mode(B{k,1});
    catch e
        d = [e.identifier ': ' e.message];
        if strcmp(e.identifier, 'test_run_mode:fail') && contains(e.message, ['FAILED AS REQUIRED at (' B{k,2} ')'])
            v = 'FAILS AS REQUIRED';
        else
            v = 'WRONG REASON';
        end
    end
    cd(startdir);  path(path0);
    fprintf('  %s: %s\n', v, d);
    res(end+1, :) = {['test_run_mode ' B{k,1}], v, d}; %#ok<AGROW>
end

if only, W = {}; else
% ---- part 2 -----------------------------------------------------------------------------------------------------------
fprintf('\n================ wrong anchor: test_run_length on a mirror with ''run_mode = 1.0;'' ================\n');
tmp = tempname;  mkdir(tmp);  copyfile(root, tmp);
f = fullfile(tmp, 'define_constants.m');
fid = fopen(f, 'r');  s = char(fread(fid, Inf, '*uint8').');  fclose(fid);
n = numel(regexp(s, '^run_mode = 1;', 'start', 'lineanchors'));
assert(n == 1, 'manual_makefail_f12:anchor', 'run_mode = 1; x%d in the mirror (expected 1)', n);
fid = fopen(f, 'w');  fwrite(fid, uint8(regexprep(s, '^run_mode = 1;', 'run_mode = 1.0;', 'once', 'lineanchors')));  fclose(fid);
cd(fullfile(tmp, 'tests'));  path(path0);  rehash;
v = 'DID NOT FAIL';  d = '';
try
    test_run_length();
catch e
    d = [e.identifier ': ' e.message];
    if strcmp(e.identifier, 'dc_set_mode:count'), v = 'FAILS AS REQUIRED'; else, v = 'WRONG REASON'; end
end
wTL = which('test_run_length');  wDC = which('dc_set_mode');
cd(startdir);  path(path0);
if ~(startsWith(wTL, tmp) && startsWith(wDC, tmp)), v = ['WRONG PATH (' wTL ' | ' wDC ')']; end
fprintf('  %s: %s\n', v, d);
res(end+1, :) = {'test_run_length wrong anchor (run_mode = 1.0;)', v, d};
if bdIsLoaded(mdl), close_system(mdl, 0); end
try, rmdir(tmp, 's'); catch, end

% ---- part 3 -----------------------------------------------------------------------------------------------------------
TOOL = 'wire_motive_emulator.m';
W = {  % id | old (once) | new | expected text in the error ('' = must pass)
  'W0', '', '', '';
  'W1', 'add_block(''simulink/Sources/Constant'', JT, ''Value'', jtVal,', 'add_block(''simulink/Sources/Constant'', JT, ''Value'', ''double(EnableHardware ~= 0)'',', 'F7 block parameters differ';
  'W2', '    set_param(TD, ''InitialCondition'', ''q0_9dof(4:9)'');', ...
        sprintf('    set_param(TD, ''InitialCondition'', ''q0_9dof(4:9)'');\n    delete_line(mdl, ''UseMotive/1'', ''StateSource/2'');  add_line(mdl, ''JointFromTelemetry/1'', ''StateSource/2'', ''autorouting'', ''smart'');   %% make-it-fail W2'), ...
        'StateSource in 2'};
for k = 1:size(W, 1)
    fprintf('\n================ wire_motive_emulator %s ================\n', W{k,1});
    tmp = tempname;  mkdir(tmp);  copyfile(root, tmp);
    tf = fullfile(tmp, 'tools', TOOL);
    if ~isempty(W{k,2})
        fid = fopen(tf, 'r');  s = char(fread(fid, Inf, '*uint8').');  fclose(fid);
        n = numel(strfind(s, W{k,2}));
        assert(n == 1, 'manual_makefail_f12:anchor', '%s: anchor x%d in %s (expected 1)', W{k,1}, n, TOOL);
        fid = fopen(tf, 'w');  fwrite(fid, uint8(strrep(s, W{k,2}, W{k,3})));  fclose(fid);
    end
    path(path0);  addpath(fullfile(tmp, 'tools'));  cd(tmp);  rehash;
    wT = which('wire_motive_emulator');
    d = '';
    try
        evalin('base', 'clear all');
        wire_motive_emulator(fullfile(tmp, [mdl '.slx']));
        ok = true;
    catch e
        ok = false;  d = [e.identifier ': ' e.message];
    end
    cd(startdir);  path(path0);
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    if ~startsWith(wT, tmp)
        v = ['WRONG PATH (' wT ')'];
    elseif isempty(W{k,4})
        if ok, v = 'PASSED as required (control)'; else, v = 'CONTROL FAILED'; end
    elseif ~ok && contains(d, 'wire_motive_emulator:verify') && contains(d, W{k,4})
        v = 'FAILS AS REQUIRED';
    elseif ok
        v = 'DID NOT FAIL';
    else
        v = 'WRONG REASON';
    end
    fprintf('  %s: %s\n', v, d);
    res(end+1, :) = {['wire_motive_emulator ' W{k,1}], v, d}; %#ok<AGROW>
    try, rmdir(tmp, 's'); catch, end
end

end   % parts 2 and 3

% ---- summary ---------------------------------------------------------------------------------------------------------
fprintf('\n================ manual_makefail_f12 summary ================\n');
for k = 1:size(res, 1), fprintf('  %-52s %s\n', res{k,1}, res{k,2}); end
good = startsWith(res(:,2), 'FAILS AS REQUIRED') | startsWith(res(:,2), 'PASSED as required');
if ~all(good)
    error('manual_makefail_f12:result', '%d of %d items not as required', sum(~good), numel(good));
end
fprintf('manual_makefail_f12: all %d items as required\n', numel(good));
end
