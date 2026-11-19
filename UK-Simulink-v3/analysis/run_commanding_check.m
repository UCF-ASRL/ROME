function run_commanding_check(matFile)
%RUN_COMMANDING_CHECK  Plan 1 Task 6: every candidate map on the 24 Sep 2026 run, printed as a table.
%   run_commanding_check()           uses ROME_merged/data/9-24-26-15-33_Case_1_data.mat
%   run_commanding_check(matFile)
%   Candidates (controller ruling R6.2), each under both firmware -> ARCS maps (rome_dirpin, arcs_caldir):
%     i     the map as used on 24 Sep (arm_sign, arm_offset_deg saved in the run file), clamp as sent
%     ii    the 7 non-empty sign flips of J1, J2, J5 applied to arm_sign (offset kept), clamp
%     ii-b  the same 7 flips applied to the MOTION: the sent command reversed about its t = 0 value,
%           held inside the firmware range (the documented bug ran joints the wrong way)
%     iii   the convention-consistent map (commanding_check header), clamp
%     end   fix round 1 (controller ruling on review F1): each of J1, J2, J5 (the joints of the documented
%           direction bug, which drove them TO a range end) held at each firmware clamp limit
%           (min/max of arm_fw_switch, arm_fw_other, pulled in by arm_fw_margin_deg) for the whole run,
%           the other joints as used. The 'ii' rows that flip J2's arm_sign are J2 at firmware 130 (the
%           switch end) on every sample (the flipped command 128.6 + 180 deg is clamped there)
%   plus the model's own arm (what the model believed). Columns, all over the whole run:
%     dEE    max |ee_phys - ee_model| (m)
%     zmin   min z of any physical link point, shoulder to flange (m), and its time (s)
%     zfl    min z of the flange (m)
%     sdmin  min signed horizontal distance of a distal link point (elbow to flange) to the base
%            rectangle (m), negative = over the base
%     sdfl   min and max signed horizontal distance of the flange to the base rectangle (m)
%     tz     min and max vertical component of the tool axis (-), -1 = straight down; tz@ = its value
%            at the sample of the lowest flange
%     Lz0    min over the run of flange z / (-tool z), over samples with the tool pointing down: the
%            length past the flange along the tool axis (-z6) at which a straight rigid tool would put its
%            tip at z = 0 (m). Needs no assumed tool length; Inf if the tool never points down
%     z02    min over the run of the tip z of a straight tool 0.02 m long past the flange along -z6 (m).
%            0.02 m is the AUTHOR'S ESTIMATE ('a couple cm', 19 Nov 2026, TRACKER Q-T6a), NOT MEASURED
%     inB    percent of samples with a distal link point over the base rectangle
%     clamp  samples where the clamp or range end acted, per joint J1..J6
%     outA   samples outside the ARCS joint ranges (ARbot.cal), any joint J1..J5
%   z = 0 is the plane the sources call the base plate (ik9_fk.m L17-18), the floor (define_constants.m
%   z_work) and the table (define_constants.m arm_home comment). Every z moves one-for-one with the mount
%   height 0.20 m (ik9_fk.m L16, NOT MEASURED); the distances depend on the mount x, y (NOT MEASURED).
%   Flat, no local functions (CLAUDE.md section 5, crash 2).
here = fileparts(mfilename('fullpath'));    % ROME_merged/analysis
root = fileparts(here);                     % ROME_merged
addpath(here, root);
if nargin < 1 || isempty(matFile)
    matFile = fullfile(root, 'data', '9-24-26-15-33_Case_1_data.mat');
end
S = load(matFile);
fprintf('run file: %s\n', matFile);
q = S.out.get('q_cmd');  gf = S.out.get('guard_flag');
qc = reshape(q.Data, 9, []).';              % Nx9 command (m, rad)
fprintf('N = %d samples, t = %.3f .. %.3f s, dt = %.4f s; q_cmd finite: %d; guard_flag values: %s\n', ...
        numel(q.Time), q.Time(1), q.Time(end), median(diff(q.Time)), all(isfinite(qc(:))), mat2str(unique(gf.Data(:)).'));
fprintf('q_cmd(t=0)  = %s\nq0_9dof     = %s\nmax|diff|   = %.3g (m, rad)\n', mat2str(qc(1,:), 6), mat2str(S.q0_9dof(:).', 6), max(abs(qc(1,:) - S.q0_9dof(:).')));
fprintf('EnableHardware %g, EnableMotive %g, arm_freeze %g, max_rpm %g, speed_factor %g, size_factor %g, scenario %g\n', ...
        S.EnableHardware, S.EnableMotive, S.arm_freeze, S.max_rpm, S.speed_factor, S.size_factor, S.scenario);
fprintf('arm_sign %s, arm_offset_deg %s, arm_freeze_deg %s\n', mat2str(S.arm_sign), mat2str(S.arm_offset_deg), mat2str(S.arm_freeze_deg, 6));
fprintf('model joint range over the run (deg): min %s, max %s\n', mat2str(min(rad2deg(qc(:,4:9))), 5), mat2str(max(rad2deg(qc(:,4:9))), 5));
fprintf('base over the run: x %.3f..%.3f m, y %.3f..%.3f m, yaw %.1f..%.1f deg\n', min(qc(:,1)), max(qc(:,1)), min(qc(:,2)), max(qc(:,2)), rad2deg(min(qc(:,3))), rad2deg(max(qc(:,3))));

q_lo = [-170 -129.6   1   -164.5 -104.15 -148.1];   % ARCS lower limits (deg), ARbot.cal (test_ar3_fk.m)
q_hi = [ 170    0   143.7  164.5  104.15  148.1];   % ARCS upper limits (deg)
fl = [1 0 0; 0 1 0; 0 0 1; 1 1 0; 1 0 1; 0 1 1; 1 1 1];   % 7x3 flipped J1, J2, J5
maps = {'rome_dirpin', 'arcs_caldir'};

% model row
R0 = commanding_check(matFile, ones(6, 1));
fprintf('\nMODEL (what the model believed): zmin %.3f m, flange zmin %.3f m, sdmin %+.3f m, flange sdmin %+.3f m, over-base samples %.1f %%\n', ...
        min(R0.model_min_link_z), min(R0.Pm_body(3, 7, :) + 0), min(R0.model_min_link_to_base_xy), min(R0.model_flange_xy), ...
        100 * mean(R0.model_min_link_to_base_xy < 0));
fprintf('  ee_model(t=0) = %s m, ee_model z range %.3f..%.3f m\n', mat2str(R0.ee_model(1,:), 4), min(R0.ee_model(:,3)), max(R0.ee_model(:,3)));

fw_lo = min(S.arm_fw_switch, S.arm_fw_other) + S.arm_fw_margin_deg;   % 1x6 lower clamp limit (deg)
fw_hi = max(S.arm_fw_switch, S.arm_fw_other) - S.arm_fw_margin_deg;   % 1x6 upper clamp limit (deg)
fprintf('clamp limits (firmware deg): lo %s, hi %s\n', mat2str(fw_lo), mat2str(fw_hi));
fprintf('MODEL tool axis z component: %.3f..%.3f\n', min(R0.model_tool_z), max(R0.model_tool_z));
hj = [1 1 2 2 5 5];                         % joint held, range-end rows
fprintf('\n%-6s %-12s %-7s %-18s | %7s %7s %6s %7s %7s %15s %13s %6s %6s %6s %6s | %-24s %5s\n', 'cand', 'map', 'flip', 'joints', 'dEE', 'zmin', 't', 'zfl', 'sdmin', 'sdfl min..max', 'tz min..max', 'tz@', 'Lz0', 'z02', 'inB%', 'clamp J1..J6', 'outA');
for m = 1:2
    for c = 1:22
        f = ones(6, 1);  fm = 'map';  fw = 'as_used';  lab = 'i';  js = '-';  hv = nan(6, 1);    % 6x1 firmware angle held per joint (deg), NaN = not held
        if c >= 2 && c <= 8,  f([1 2 5]) = 1 - 2 * fl(c-1, :);  lab = 'ii';   end
        if c >= 9 && c <= 15, f([1 2 5]) = 1 - 2 * fl(c-8, :);  lab = 'ii-b'; fm = 'motion'; end
        if c == 16, fw = 'consistent'; lab = 'iii'; end
        if c >= 2 && c <= 15, js = strjoin(arrayfun(@(j) sprintf('J%d', j), find(f < 0).', 'UniformOutput', false), ''); end
        if c >= 2 && c <= 8 && f(2) < 0, js = [js '(J2@fw130)']; end
        if c >= 17
            j = hj(c-16);  lab = 'end';
            if mod(c, 2) == 1, hv(j) = fw_lo(j); else, hv(j) = fw_hi(j); end
            js = sprintf('J%d@fw%g', j, hv(j));
        end
        R = commanding_check(matFile, f, struct('romefw', maps{m}, 'flip', fm, 'fwmap', fw, 'hold', hv));
        dEE = max(sqrt(sum((R.ee_phys - R.ee_model).^2, 2)));       % (m)
        [zmin, kz] = min(R.min_link_z);                             % (m), sample index
        [zfl, kf] = min(R.ee_phys(:, 3));                           % lowest flange (m), its sample
        dn = R.tool_z < 0;                                          % samples with the tool pointing down
        Lz0 = min([inf; R.ee_phys(dn, 3) ./ (-R.tool_z(dn))]);      % tool length reaching z = 0 (m)
        L_est = 0.02;                                               % tool length past the flange (m), author's estimate, NOT MEASURED
        z02 = min(R.ee_phys(:, 3) + L_est * R.tool_z);              % lowest tip z of that tool (m)
        outA = sum(any(R.q_arcs(:, 1:5) < q_lo(1:5) - 1e-9 | R.q_arcs(:, 1:5) > q_hi(1:5) + 1e-9, 2));
        fprintf('%-6s %-12s %-7s %-18s | %7.3f %7.3f %6.2f %7.3f %+7.3f %+7.3f..%+7.3f %+6.2f..%+6.2f %+6.2f %6.3f %6.3f %6.1f | %-24s %5d\n', lab, maps{m}, fm, js, ...
                dEE, zmin, R.t(kz), zfl, min(R.min_link_to_base_xy), min(R.flange_xy), max(R.flange_xy), ...
                min(R.tool_z), max(R.tool_z), R.tool_z(kf), Lz0, z02, 100 * mean(R.min_link_to_base_xy < 0), ...
                mat2str(sum(R.clamp_hit, 1)), outA);
        if c == 1 || c == 16
            fprintf('       fw map: arm_sign %s, arm_offset_deg %s; q_arcs(t=0) %s; flange body(t=0) %s m\n', ...
                    mat2str(R.arm_sign), mat2str(R.arm_offset_deg, 6), mat2str(R.q_arcs(1,:), 5), mat2str(R.P_body(:,7,1).', 4));
            ks = [1:40:numel(R.t), numel(R.t)];                     % every 2 s and the last sample
            for k = ks
                fprintf('       t %5.2f  ee_model %s  ee_phys %s  |d| %.3f  flange body %s  zmin %.3f  sdmin %+.3f  q_arcs %s\n', R.t(k), ...
                        mat2str(R.ee_model(k,:), 3), mat2str(R.ee_phys(k,:), 3), norm(R.ee_phys(k,:) - R.ee_model(k,:)), ...
                        mat2str(R.P_body(:,7,k).', 3), R.min_link_z(k), R.min_link_to_base_xy(k), mat2str(R.q_arcs(k,:), 4));
            end
        end
    end
end
end
