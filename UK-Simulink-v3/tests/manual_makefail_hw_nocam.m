function manual_makefail_hw_nocam(ids)
%MANUAL_MAKEFAIL_HW_NOCAM  Make-it-fail of tests/test_hw_nocam.m (Plan 2 Task 1c): one break per check.
%   Runs test_hw_nocam('<id>') for every break of its table (test_hw_nocam('list')), or for the ids given
%   (cell of char). Each break is applied to that run's own temporary copy of ROME_merged only; ROME_merged
%   itself is never modified. Verdict per break:
%     FAILS         test_hw_nocam errored with test_hw_nocam:fail, its list of failed checks contains the
%                   check the break targets (other checks may fail with it; they are printed), AND the ref, hw
%                   and hw_frz simulations returned (fix round 1, I3: a crashed run that fails every check is
%                   WRONG REASON); for the break with check 'stubdrift', the error test_hw_nocam:stubdrift
%     DID NOT FAIL  the test passed although the property was broken (a FINDING)
%     WRONG REASON  it errored some other way (setup, an edit that did not match once, a crash in MATLAB code)
%   Ends in error('manual_makefail_hw_nocam:notfailed') unless every break FAILS. A MATLAB crash (exit 139)
%   ends the launch: the breaks after the last "#### START" line did not run.
%   Flat function without local functions (CLAUDE.md section 5, crash 2).
here = fileparts(mfilename('fullpath'));      % ROME_merged/tests (char)
addpath(here);
B = test_hw_nocam('list');                    % break table: id | check | edits | why
if nargin < 1 || isempty(ids), ids = B(:,1).'; end
verdict = cell(1, numel(ids));                % verdict text per break (char)
good = false(1, numel(ids));                  % true when the break FAILS its check
for k = 1:numel(ids)
    row = find(strcmp(B(:,1), ids{k}));       % row of the break (index)
    assert(numel(row) == 1, 'manual_makefail_hw_nocam:id', 'unknown break %s', ids{k});
    expect = B{row,2};                        % check that must fail (char)
    fprintf('\n#### START %s: check (%s) must FAIL. %s\n', ids{k}, expect, B{row,4});
    t0 = tic;
    try
        test_hw_nocam(ids{k});
        verdict{k} = 'DID NOT FAIL';
    catch e
        tok = regexp(e.message, 'FAILED checks: ([^\[]+)', 'tokens', 'once');   % failed check ids (cell of char)
        failed = {};                          % failed check ids (cell of char)
        if ~isempty(tok), failed = strtrim(strsplit(strtrim(tok{1}), ',')); end
        rt = regexp(e.message, 'returned: ref (\d), hw (\d), hw_err (\d), hw_frz (\d)', 'tokens', 'once');   % sim returned per run (cell of char)
        ret = NaN(1, 4);                      % ref, hw, hw_err, hw_frz returned from sim() (flag), NaN if not reported
        if ~isempty(rt), ret = str2double(rt); end
        runsOk = isequal(ret([1 2 4]), [1 1 1]);   % ref, hw and hw_frz must have run to the end (hw_err ends in its injected error)
        if strcmp(expect, 'stubdrift') && strcmp(e.identifier, 'test_hw_nocam:stubdrift')
            verdict{k} = sprintf('FAILS (test_hw_nocam:stubdrift: %s)', e.message);
            good(k) = true;
        elseif strcmp(e.identifier, 'test_hw_nocam:fail') && any(strcmp(failed, expect)) && runsOk
            verdict{k} = sprintf('FAILS (failed checks: %s; runs returned ref/hw/hw_err/hw_frz %s)', strjoin(failed, ', '), mat2str(ret));
            good(k) = true;
        elseif strcmp(e.identifier, 'test_hw_nocam:fail') && any(strcmp(failed, expect))
            % fix round 1 (I3): a check that fails only because a simulation crashed is not shown to detect its property
            verdict{k} = sprintf('WRONG REASON (check failed but a run did not return from sim(): ref/hw/hw_err/hw_frz %s; %s)', mat2str(ret), e.message);
        else
            verdict{k} = sprintf('WRONG REASON (%s: %s)', e.identifier, e.message);
        end
    end
    fprintf('#### END %s (%.0f s): %s\n', ids{k}, toc(t0), verdict{k});
end
fprintf('\n==== manual_makefail_hw_nocam summary ====\n');
for k = 1:numel(ids)
    fprintf('  %-14s %s\n', ids{k}, verdict{k});
end
if ~all(good)
    error('manual_makefail_hw_nocam:notfailed', '%d of %d breaks did not FAIL their check: %s', ...
          sum(~good), numel(ids), strjoin(ids(~good), ', '));
end
fprintf('all %d breaks FAIL their check\n', numel(ids));
end
