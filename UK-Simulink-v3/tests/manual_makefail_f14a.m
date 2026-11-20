function manual_makefail_f14a(which_)
%MANUAL_MAKEFAIL_F14A  Make-it-fail campaign of the F14a tests (focus plan F14a, 19 Nov 2026). Not in test_suite.
%   manual_makefail_f14a()          every break of test_lab_settings, test_run_verdict_core, test_heading_plot and
%                                   test_run_verdict
%   manual_makefail_f14a(tests)     only the tests named in the cellstr tests
%   Each break is applied by the test itself to its temporary copy (one match asserted; the real files are never
%   written). A break SCORES only when the test errors with <test>:fail and the message names EVERY check listed for
%   that break as "(x)". Prints one line per break and errors manual_makefail_f14a:result unless all scored.
%   Flat function (CLAUDE.md section 5, crash 2).
T = {'test_lab_settings', 'test_run_verdict_core', 'test_heading_plot', 'test_run_verdict'};
if nargin >= 1 && ~isempty(which_), T = intersect(T, which_, 'stable'); end
res = cell(0, 4);                                  % test | break | scored | what happened
for i = 1:numel(T)
    B = feval(T{i}, 'list');
    for k = 1:size(B, 1)
        id = B{k,1};  checks = strsplit(strtrim(B{k,2}));
        fprintf('\n######## %s(''%s''): must FAIL at %s ########\n', T{i}, id, strjoin(strcat('(', checks, ')'), ' '));
        msg = '';  eid = '';
        try
            feval(T{i}, id);
        catch e
            eid = e.identifier;  msg = e.message;
        end
        named = all(cellfun(@(c) contains(msg, ['(' c ')']), checks));
        ok = strcmp(eid, [T{i} ':fail']) && named;
        if isempty(eid), what = 'PASSED (break not caught)'; else, what = sprintf('%s: %s', eid, msg(1:min(end, 300))); end
        res(end+1, :) = {T{i}, id, ok, what}; %#ok<AGROW>
        fprintf('######## %s %s -> %s\n', id, mat2str(ok), what);
    end
end
fprintf('\n==== manual_makefail_f14a summary ====\n');
for k = 1:size(res, 1)
    lab = {'NOT CAUGHT', 'FAILS AS REQUIRED'};
    fprintf('  %-22s %-15s %s\n', res{k,1}, res{k,2}, lab{1 + res{k,3}});
end
if ~all([res{:,3}])
    error('manual_makefail_f14a:result', '%d of %d breaks not caught as required', sum(~[res{:,3}]), size(res, 1));
end
fprintf('manual_makefail_f14a: %d of %d breaks FAIL at their named checks\n', size(res, 1), size(res, 1));
end
