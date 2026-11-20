function manual_makefail_motive_f7(testName, ids)
%MANUAL_MAKEFAIL_MOTIVE_F7  Runs breaks of an F7 test once each and scores them (focus plan F7).
%   manual_makefail_motive_f7('test_motive_source')            every break of test_motive_source('list')
%   manual_makefail_motive_f7('test_motive_emulator', {'mf_gate'})   only the listed breaks
%   A break PASSES (as a make-it-fail) only when <testName>('<id>') errors with <testName>:fail AND the message names
%   the break's check, e.g. '(iii)'. Any other error (setup, crash of the copy) is WRONG REASON.
%   Prints one line per break, then errors if any break did not fail for its own reason.
%   Flat function without local functions (CLAUDE.md section 5, crash 2).
B = feval(testName, 'list');                  % break table: id | check | edits | what it does
if nargin >= 2 && ~isempty(ids)
    keep = ismember(B(:,1), ids);
    assert(sum(keep) == numel(ids), 'manual_makefail_motive_f7:ids', 'unknown break id among %s', strjoin(ids, ', '));
    B = B(keep, :);
end
verdict = cell(size(B, 1), 1);                % FAILS AS REQUIRED / DID NOT FAIL / WRONG REASON per break
for k = 1:size(B, 1)
    fprintf('\n================ %s break %s (check %s must FAIL) ================\n', testName, B{k,1}, B{k,2});
    try
        feval(testName, B{k,1});
        verdict{k} = 'DID NOT FAIL';
    catch e
        if strcmp(e.identifier, [testName ':fail']) && contains(e.message, ['(' B{k,2} ')'])
            verdict{k} = 'FAILS AS REQUIRED';
        else
            verdict{k} = 'WRONG REASON';
        end
        fprintf('  error %s: %s\n', e.identifier, e.message);
    end
end
fprintf('\n================ make-it-fail summary (%s) ================\n', testName);
for k = 1:size(B, 1)
    fprintf('  %-14s check (%s): %s\n', B{k,1}, B{k,2}, verdict{k});
end
if ~all(strcmp(verdict, 'FAILS AS REQUIRED'))
    error('manual_makefail_motive_f7:fail', 'not every break failed for its own check');
end
fprintf('all %d breaks fail as required\n', size(B, 1));
end
