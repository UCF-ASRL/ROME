function manual_makefail_bench_joint()
%MANUAL_MAKEFAIL_BENCH_JOINT  Make-it-fail of the joint-side and parser checks of test_bench (Plan 3 Task 2, fix round 1).
%   Runs test_bench(k) for the ten mutations k = 1..10 (each defined at its "MUT k" comment in test_bench.m):
%     1 parser: 8-field GV line expected to be accepted     6 a1 dirpin log expected to give arcs_caldir
%     2 evaluator constants differ in J2, a1 expects no flag  7 m5 lean 90 at fw 132 expected arcs_caldir
%     3 J2 not inverted, a2 expects it flagged               8 i1 independent lean 9.4 at fw 51.4 expected arcs_caldir
%     4 J3 follows, a3 expects NO FOLLOW                     9 i2 both-map observation expected rome_dirpin
%     5 J1 offset not shifted, a4 expects it flagged        10 i3 NO FOLLOW row expected to give arcs_caldir
%   EACH mutation must make test_bench error with identifier test_bench:fail. A mutation that passes, or errors
%   with another identifier, means the check cannot fail (or the test is broken) and this function errors.
%   Prints MAKE-IT-FAIL OK when all ten failed as required. Pure functions, synthetic data, no model.
nMut = 10;
bad = [];
for k = 1:nMut
    try
        test_bench(k);
        fprintf('MUT %2d: PASSED (should have failed)\n', k);  bad(end+1) = k; %#ok<AGROW>
    catch err
        if strcmp(err.identifier, 'test_bench:fail')
            fprintf('MUT %2d: FAILED AS REQUIRED: %s\n', k, err.message);
        else
            fprintf('MUT %2d: WRONG ERROR %s: %s\n', k, err.identifier, err.message);  bad(end+1) = k; %#ok<AGROW>
        end
    end
end
if ~isempty(bad), error('manual_makefail_bench_joint:notfailed', 'mutations not failing as required: %s', mat2str(bad)); end
fprintf('manual_makefail_bench_joint: MAKE-IT-FAIL OK (%d of %d mutations failed as required)\n', nMut, nMut);
end
