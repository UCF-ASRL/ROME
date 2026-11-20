function manual_makefail_motive_model()
%MANUAL_MAKEFAIL_MOTIVE_MODEL  Runs every break of test_motive_model('list') once and scores it (focus plan F5b).
%   A break PASSES (as a make-it-fail) only when test_motive_model('<id>') errors with test_motive_model:fail AND the
%   message names the break's check, e.g. '(iv)'. Any other error (setup, crash of the copy) is WRONG REASON.
%   Prints one line per break, then errors if any break did not fail for its own reason.
%   Flat function without local functions (CLAUDE.md section 5, crash 2).
B = test_motive_model('list');                % break table: id | check | edits | what it does
verdict = cell(size(B, 1), 1);                % FAILS AS REQUIRED / DID NOT FAIL / WRONG REASON per break
for k = 1:size(B, 1)
    fprintf('\n================ break %s (check %s must FAIL) ================\n', B{k,1}, B{k,2});
    try
        test_motive_model(B{k,1});
        verdict{k} = 'DID NOT FAIL';
    catch e
        if strcmp(e.identifier, 'test_motive_model:fail') && contains(e.message, ['(' B{k,2} ')'])
            verdict{k} = 'FAILS AS REQUIRED';
        else
            verdict{k} = 'WRONG REASON';
        end
        fprintf('  error %s: %s\n', e.identifier, e.message);
    end
end
fprintf('\n================ make-it-fail summary ================\n');
for k = 1:size(B, 1)
    fprintf('  %-14s check (%s): %s\n', B{k,1}, B{k,2}, verdict{k});
end
if ~all(strcmp(verdict, 'FAILS AS REQUIRED'))
    error('manual_makefail_motive_model:fail', 'not every break failed for its own check');
end
fprintf('all %d breaks fail as required\n', size(B, 1));
end
