function manual_makefail_bench()
%MANUAL_MAKEFAIL_BENCH  Make-it-fail of test_bench case w2 (Plan 3 Task 2, step 5).
%   Builds a wheel log with NO swap (channel k = wheel k) and asserts that a swap of wheels 1 and 3 is
%   detected. The assertion is WRONG on purpose, so this function must END IN AN ERROR
%   ('manual_makefail_bench:expected_failure'): it shows that bench_wheel_order does not flag a clean log, i.e.
%   that case w2 of test_bench can fail. If it returns without error, the evaluator flags clean logs: broken.
%   Pure function, synthetic data, no model, no define_constants.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(fileparts(here), 'lab'));
wl = struct('k_cmd', {}, 'rpm_cmd', {}, 'gv_lines', {}, 'eye_wheel', {}, 'eye_bottom_xy', {});
for k = 1:4
    L = strings(10, 1);
    for i = 1:10
        act = zeros(1, 4);  cmd = zeros(1, 4);  cmd(k) = 15;
        if i > 3, act(k) = 15; end                 % no swap: wheel k turns channel k
        L(i) = sprintf("GV,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", act, cmd);
    end
    wl(k).k_cmd = k;  wl(k).rpm_cmd = 15;  wl(k).gv_lines = L;  wl(k).eye_wheel = 0;  wl(k).eye_bottom_xy = [NaN NaN];
end
T = bench_wheel_order(wl);
swapDetected = isequal(T.k_cmd(T.flag).', [1 3]);   % what a swap of wheels 1 and 3 would give
if ~swapDetected
    error('manual_makefail_bench:expected_failure', ...
          'EXPECTED FAILURE: a log with no swap was asserted to show wheels 1 and 3 flagged; flagged rows = [%s]', ...
          num2str(T.k_cmd(T.flag).'));
end
fprintf('manual_makefail_bench: UNEXPECTED, a clean log was flagged as swapped\n');
end
