function test_v3_table3(runsDir, intDir)
%TEST_V3_TABLE3  Check frozen runs case_1..4.mat against Table 3 of the final paper.
%   test_v3_table3(runsDir)          runsDir holds case_1.mat .. case_4.mat (struct S, arrays t, q).
%   test_v3_table3(runsDir, intDir)  intDir holds define_constants.m (default: ../../ROME_9DOF_Integration).
%
%   Source of the expected values: Submission/initialSubmission_V3.tex, lines 467-470 (Table 3).
%   Reference copy of the runs: CMS/docs/verification/reference_runs_V3/ (byte copies of
%   ROME_9DOF_Integration/runs/, cmp-verified).
%
%   Mapping of the saved fields to the printed columns (confirmed from the numbers, case 1:
%   rmse_ee_lead = 6.0448e-04 m -> 604 um; rmse_ee = 0.0080 m -> 8.0 mm):
%     aligned position [um] = 1e6 * S.rmse_ee_lead   (m -> um)
%     latency position [mm] = 1e3 * S.rmse_ee        (m -> mm)
%     aligned attitude [deg] = S.rmse_att_lead       (already deg)
%     latency attitude [deg] = S.rmse_att            (already deg)
%     residual = S.max_res, peak [rpm] = S.peak_rpm, min s = S.min_s, run [s] = round(t(end))
%     margin [deg] = closest approach of any arm joint command to a firmware limit, with the
%     expression of ROME_9DOF_Integration/fig_9dof_cases.m (lines 27-40), firmware degrees.
%   Each value is rounded to the digits the paper prints and compared to the paper's number,
%   except the residual, which must be below 1e-11 (round-off; ruling R1, 18 Nov 2026).
%   Raw and rounded values are printed side by side. Any mismatch raises an error naming the
%   case and column, after all cells have been printed.
%   Never writes inside the integration folder.
here = fileparts(mfilename('fullpath'));
if nargin < 2, intDir = fullfile(here, '..', '..', 'ROME_9DOF_Integration'); end
assert(isfolder(runsDir), 'runs directory not found: %s', runsDir);
assert(isfile(fullfile(intDir, 'define_constants.m')), 'define_constants.m not found in %s', intDir);
assert(~isfile(fullfile(intDir, 'define_constants.PRISTINE')), 'a run holds define_constants.m');
K = evalc('run(fullfile(intDir, ''define_constants.m''))'); %#ok<NASGU>  % arm_fw_switch, arm_fw_other, arm_sign, arm_offset_deg, arm_fw_margin_deg
lo = min(arm_fw_switch, arm_fw_other); hi = max(arm_fw_switch, arm_fw_other); mg = arm_fw_margin_deg; %#ok<NODEF>  % firmware limits [deg], 1x6

% Table 3 rows: expected value and number of printed decimals (residual: decimals of the mantissa, %e).
% columns: T, pos aligned [um], pos latency [mm], att aligned [deg], att latency [deg], residual, peak [rpm], min s, margin [deg]
cols = {'T [s]','pos aligned [um]','pos latency [mm]','att aligned [deg]','att latency [deg]','residual','peak [rpm]','min s','margin [deg]'};
fmt  = {'%.0f','%.0f','%.1f','%.3f','%.2f','%.1e','%.1f','%.3f','%.0f'};   % default; per-case overrides below
exp_ = [31 604 8.0  0.064 0.79  4.7e-12 42.9 0.677 36;
        55 120 0.34 0.016 0.016 2.9e-13 1.8  0.675 35;
        55 126 0.33 0.018 0.018 2.6e-13 1.7  0.675 35;
        55 173 3.0  0.028 0.40  5.2e-12 15.7 0.678 37];
fm = repmat(fmt, 4, 1);
res_bound = 1e-11;   % residual column: max |Jc qdd - b_ee| must stay below this (mixed-unit norm, m/s^2 and rad/s^2; round-off level)
fm{1,5} = '%.2f'; fm{2,3} = '%.2f'; fm{2,5} = '%.3f'; fm{3,3} = '%.2f'; fm{3,5} = '%.3f'; fm{4,3} = '%.1f'; fm{4,5} = '%.2f';   % digits as printed in the table

bad = {};
for c = 1:4
    f = fullfile(runsDir, sprintf('case_%d.mat', c));
    assert(isfile(f), 'missing %s', f);
    D = load(f, 'S', 't', 'q'); S = D.S;
    fw = arm_sign .* rad2deg(D.q(:,4:9)) + arm_offset_deg;                    % N x 6 firmware degrees (same expression as fig_9dof_cases.m)
    room = min([min(fw,[],1) - (lo + mg), (hi - mg) - max(fw,[],1)]) + mg;   % closest approach to a limit [deg]
    raw = [D.t(end), 1e6*S.rmse_ee_lead, 1e3*S.rmse_ee, S.rmse_att_lead, S.rmse_att, S.max_res, S.peak_rpm, S.min_s, room];
    for k = 1:9
        if k == 6
            % Residual column: bound, not digits (controller ruling R1, 18 Nov 2026). V3 line 480:
            % "the residual stays at round-off". The printed mantissa is not reproducible across
            % machines/releases (this PC, R2026a: 4.77e-12/2.63e-13/2.29e-13/4.11e-12 against the
            % printed 4.7e-12/2.9e-13/2.6e-13/5.2e-12), so the raw value is printed and bounded.
            ok = raw(k) < res_bound;
            fprintf('case %d  %-18s raw %-14.8g bound  < %-8g paper %-10g %s\n', c, cols{k}, raw(k), res_bound, exp_(c,k), ternary(ok));
            if ~ok, bad{end+1} = sprintf('case %d %s: raw %.8g not below the round-off bound %g', c, cols{k}, raw(k), res_bound); end %#ok<AGROW>
            continue
        end
        rnd = str2double(sprintf(fm{c,k}, raw(k)));        % value rounded to the printed digits
        ok = abs(rnd - exp_(c,k)) <= 1e-9 * abs(exp_(c,k));
        fprintf('case %d  %-18s raw %-14.8g rounded %-10g paper %-10g %s\n', c, cols{k}, raw(k), rnd, exp_(c,k), ternary(ok));
        if ~ok, bad{end+1} = sprintf('case %d %s: rounded %g (raw %.8g) vs paper %g', c, cols{k}, rnd, raw(k), exp_(c,k)); end %#ok<AGROW>
    end
end
if ~isempty(bad)
    error('test_v3_table3:mismatch', 'Table 3 mismatch (%d cells):\n  %s', numel(bad), strjoin(bad, sprintf('\n  ')));
end
fprintf('test_v3_table3 PASS: 4 cases x 8 columns match Table 3, residual < %g in all 4\n', res_bound);
end

function s = ternary(ok)
if ok, s = 'ok'; else, s = 'MISMATCH'; end
end
