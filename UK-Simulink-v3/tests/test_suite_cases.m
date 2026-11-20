function test_suite_cases()
%TEST_SUITE_CASES  Make-it-fail of test_suite's step registration (wire3 review C1, 20 Nov 2026). Not a suite step
%   (it calls test_suite). Runs test_suite on ONE quick step, so it takes seconds, not the full suite:
%     (a) control: test_suite({'test_run_verdict_core'}) runs that step (its PASS line is printed) and the suite FAILS
%         (test_suite:fail) because every other step is SKIPPED: a selected run can never print "test_suite PASS"
%     (b) break: test_suite({'test_nocase_probe'}, {'test_nocase_probe'}) lists a step that has no case: the step FAILS
%         with test_suite:nostep and the suite FAILS. Before the fix (switch on the step number, no otherwise) such a
%         step was counted as passed without running.
%   Errors test_suite_cases:fail listing the failed checks.
fails = {};
txtA = '';  idA = '';
try, txtA = evalc('test_suite({''test_run_verdict_core''})'); catch e, idA = e.identifier;  txtA = [txtA e.message]; end
okA = strcmp(idA, 'test_suite:fail') && ~contains(txtA, 'test_suite PASS:');
fprintf('(a) only test_run_verdict_core: error ''%s'', no "test_suite PASS:" line %d -> %s\n', idA, ~contains(txtA, 'test_suite PASS:'), mat2str(okA));
if ~okA, fails{end+1} = '(a)'; end
txtB = '';  idB = '';
try
    txtB = evalc('test_suite({''test_nocase_probe''}, {''test_nocase_probe''})');
catch e
    idB = e.identifier;  txtB = [txtB e.message];
end
% evalc loses the printed text when the call errors: re-run without capture to read the step message
if isempty(txtB) || ~contains(txtB, 'test_suite:nostep')
    try, test_suite({'test_nocase_probe'}, {'test_nocase_probe'}); catch e2, txtB = [txtB ' | ' e2.message]; end
end
okB = strcmp(idB, 'test_suite:fail') && contains(txtB, 'test_nocase_probe');
fprintf('(b) listed step without a case: error ''%s'', names the step %d -> %s\n', idB, contains(txtB, 'test_nocase_probe'), mat2str(okB));
if ~okB, fails{end+1} = '(b)'; end
if ~isempty(fails), error('test_suite_cases:fail', 'test_suite_cases FAILED: %s', strjoin(fails, ', ')); end
fprintf('test_suite_cases PASS\n');
end
