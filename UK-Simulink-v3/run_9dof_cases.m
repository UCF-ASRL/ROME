function run_9dof_cases(mode)
%   run_9dof_cases('post') skips the simulation and recomputes the derived
%   arrays and the summary from the saved q, qd, t of each runs/case_<n>.mat
%   (added 25 Sep 2026 with the attitude arrays Yaw, YawRef, Eatt, deg).
%RUN_9DOF_CASES  Run ROME_9DOF.slx on each of the five case studies and keep
%   every logged signal, so the results figures are drawn from the model
%   itself and not from its offline twin.
%
%   Same harness as gate_all (define_constants.m is rewritten per scenario
%   under the PRISTINE lock and restored on exit), same model, same solver
%   settings, hardware, Motive and arm links disabled. Two deliberate
%   differences from check_9dof, both stated in the manuscript:
%     - real-time pacing is off (apply_pacing, EnableHardware = 0), so
%       a 200 s run does not take 200 s of wall clock;
%     - the stop time is T_scaled + T_ON/2 (the reference clock of
%       EndEffectorTrajectory lags t by half its onset), so every case
%       completes its full path.
%   Writes runs/case_<n>.mat with:
%     t (N x 1, s), q (N x 9: x y theta [m m rad], q1..q6 [rad]),
%     qd (N x 9), w (N x 4, rpm), tau (N x 6, N m), manip, resid, guard (N x 1),
%     P (N x 3, end-effector position from the commanded q, m),
%     Ref (N x 3, reference at t), RefNext (N x 3, reference at t + dt),
%     E (lead-corrected error, m), ES (same-sample error, m), vref (N x 1, m/s),
%     name, sc, dt, max_rpm, and the summary struct S with the Table 6 metrics
%     recomputed exactly as check_9dof computes them.
%   Prints one summary line per case; that printout is what the manuscript
%   quotes.

if nargin < 1, mode = 'sim'; end
here = fileparts(mfilename('fullpath')); cd(here);
addpath(here, fullfile(here,'Ground Vehicle'), fullfile(here,'Optitrack'));
outdir = fullfile(here, 'runs'); if ~isfolder(outdir), mkdir(outdir); end
mdl = 'ROME_9DOF';
names = {'Elliptical orbit','V-bar','R-bar','NMC','Circle'};

assert_sim_mode('run_9dof_cases', 'file', fullfile(here,'define_constants.m'));   % F12: once, before custody
orig = claim_constants();
restore = onCleanup(@() release_constants(orig)); %#ok<NASGU>
fprintf('\n%-17s %7s %9s %9s %11s %11s %8s %8s %8s\n', 'case', 'T [s]', 'peak rpm', 'min s', ...
        'lead rmse', 'same rmse', 'max res', 'peak tau', 'base r');
for sc = 1:5
    txt = regexprep(orig, 'scenario = \d+;', sprintf('scenario = %d;', sc), 'once');
    assert(contains(txt, sprintf('scenario = %d;', sc)), 'run_9dof_cases:select', ...
           'could not set scenario = %d in define_constants.m', sc);
    wf('define_constants.m', txt);

    evalin('base', 'clear all');
    evalin('base', sprintf('run(''%s'');', fullfile(here,'define_constants.m')));
    assignin('base','EnableArm',0);
    % RUN MODE (focus plan F12). This simulates run mode 1 only. The flags come from run_mode in define_constants.m,
    % and load_system's PostLoadFcn re-runs define_constants, so an assignin of the flags would be overwritten: the mode is
    % ASSERTED instead (assert_sim_mode, shared by every runner), here and again after load_system.
    assert_sim_mode('run_9dof_cases', 'base');
    T_scaled = evalin('base','T_scaled');  dt = evalin('base','dt_9dof');
    max_rpm  = evalin('base','max_rpm');
    T_ON     = evalin('base','T_ON');     % onset length (s), define_constants; the reference clock lags t by T_ON/2 after it
    T_stop   = evalin('base','T_stop');   % run length (s) = T_scaled + T_ON/2, define_constants

    if strcmp(mode, 'post')
        D = load(fullfile(outdir, sprintf('case_%d.mat', sc)));
        q = D.q; qd = D.qd; w = D.w; tau = D.tau; manip = D.manip; resid = D.resid; guard = D.guard; t = D.t; QM = D.QM;
        assert(abs(t(end) - T_stop) < 1.5*dt, 'run_9dof_cases:post', 'saved run length does not match define_constants');
        N = numel(t);
    else
        if bdIsLoaded(mdl), close_system(mdl,0); end
        load_system(fullfile(here,[mdl '.slx']));
        try, assert_sim_mode('run_9dof_cases', 'base'); catch em_, close_system(mdl,0); rethrow(em_); end   % after the PostLoadFcn
        set_param(mdl,'StopFcn','');
        % pacing: EnableHardware = 0 comes from run_mode = 1 (asserted above), so apply_pacing (InitFcn)
        % turns pacing off at the start of this run (tests/test_pacing.m case b)
        clear ukd_current
        out = sim(mdl, 'SimulationMode','normal', 'ReturnWorkspaceOutputs','on', ...
                  'StopTime', num2str(T_stop, '%.10g'));   % = the model's saved 'T_stop'
        fprintf('  run flags (read back after the sim): EnableMotive %g, motive_source %s, motive_emulate_noise %g, sensor_model %g\n', ...
                evalin('base','EnableMotive'), evalin('base','motive_source'), evalin('base','motive_emulate_noise'), evalin('base','sensor_model'));
        close_system(mdl,0);

        g = @(n) as_rows(squeeze(out.(n).Data));
        q = g('q_cmd'); qd = g('q_dot_cmd'); w = g('wheel_speeds_rpm'); tau = g('tau_arm');
        QM = g('q_meas');                                          % N x 9 what the controller read (SensorModel output)
        manip = g('manip'); manip = manip(:); resid = g('resid'); resid = resid(:);
        guard = g('guard_flag'); guard = guard(:);
        t = out.q_cmd.Time;  N = numel(t);
    end

    els = evalin('base','elements'); mu_ = evalin('base','mu');
    ts  = evalin('base','time_scale'); ds = evalin('base','dist_scale');
    zw  = evalin('base','z_work'); pr = evalin('base','par');
tt  = evalin('base','tool_tilt_deg');   % tool tilt (deg): the reference is rebuilt with it
    P = zeros(N,3); Ref = zeros(N,3); RefNext = zeros(N,3); E = zeros(N,1); ES = zeros(N,1); EM = zeros(N,1); vref = zeros(N,1);
    s_run = zeros(N,1); Yaw = zeros(N,1); YawRef = zeros(N,1); Eatt = zeros(N,1); EattLead = zeros(N,1);
    Vee = zeros(N,6); Vref = zeros(N,6);            % end-effector twist J_c qd and the reference twist V_des (m/s, rad/s)
    Aref = zeros(N,6);                               % reference twist rate dV_des/dt (m/s^2, rad/s^2)
    for k = 1:N
        [p_ref,u_ref,Vk,Ak] = EndEffectorTrajectory(t(k),  sc, els, mu_, ts, ds, zw, pr, tt);
        [p_nxt,u_nxt] = EndEffectorTrajectory(t(k) + dt, sc, els, mu_, ts, ds, zw, pr, tt);
        [pe, Re] = ik9_fk(q(k,:).');
        [Yaw(k), YawRef(k), Eatt(k)] = ee_attitude(Re, u_ref);   % deg; same-sample attitude error
        [~, ~, EattLead(k)] = ee_attitude(Re, u_nxt);             % deg; lead-corrected, as E for the position
        P(k,:) = pe.'; Ref(k,:) = p_ref.'; RefNext(k,:) = p_nxt.';
        E(k)  = norm(pe - p_nxt);            % lead-corrected: command vs reference one sample ahead
        ES(k) = norm(pe - p_ref);            % same sample: the one-step sampling lead |v| dt plus the noise response
        EM(k) = norm(ik9_fk(QM(k,:).') - p_ref);   % measured pose (camera noise, joint quantization) against the reference
        vref(k) = norm(Vk(1:3));
        [Mk, Jk] = mass_at(q(k,:).');
        s_run(k) = sqrt(max(min(eig(Jk*(Mk\Jk.'))), 0));
        Vee(k,:) = (Jk*qd(k,:).').'; Vref(k,:) = Vk(:).'; Aref(k,:) = Ak(:).';
    end
    Aee = gradient(Vee.', dt).';             % end-effector twist rate by central differences of J_c qd (the block does not output qdd)
    ss = t >= t(end)/4;                      % past the capture transient, as in check_9dof
    S = struct();
    S.T = t(end); S.N = N; S.dt = dt;
    S.rmse_ee_lead = sqrt(mean(E(ss).^2));  S.max_ee_lead = max(E(ss));
    S.rmse_ee      = sqrt(mean(ES(ss).^2));  S.max_ee = max(ES(ss));
    S.rmse_meas    = sqrt(mean(EM(ss).^2));  S.max_meas = max(EM(ss));   % measured-pose error, m
    S.lag_bound    = max(vref) * dt;
    S.max_res      = max(resid);
    S.peak_rpm     = max(abs(w(:)));
    S.min_s        = min(s_run);  S.min_manip = min(manip);
    S.peak_tau     = max(abs(tau(:)));
    S.base_reach   = max(vecnorm(q(:,1:2),2,2));
    S.settle_1mm   = settle_time(t, E, 1e-3, 0);        % first time from which E stays under 1 mm for the rest of the run
    S.rmse_att     = sqrt(mean(Eatt(ss).^2));  S.max_att = max(Eatt(ss));   % deg, same sample
    S.rmse_att_lead = sqrt(mean(EattLead(ss).^2));  S.max_att_lead = max(EattLead(ss));   % deg, lead-corrected
    name = names{sc};
    save(fullfile(outdir, sprintf('case_%d.mat', sc)), 't','q','qd','w','tau','manip','resid','guard', ...
         'P','Ref','RefNext','E','ES','EM','QM','vref','s_run','Yaw','YawRef','Eatt','EattLead','Vee','Vref','Aee','Aref','name','sc','dt','max_rpm','S','T_ON');
    fprintf('%-17s %7.1f %9.2f %9.3f %11.3e %11.3e %8.1e %8.2f %8.2f\n', name, S.T, S.peak_rpm, S.min_s, ...
            S.rmse_ee_lead, S.rmse_ee, S.max_res, S.peak_tau, S.base_reach);
    fprintf('%-17s settles under 1 mm at t = %.1f s (onset ends at %.0f s); max lead-corrected error after t/4: %.2e m; sampling-lead bound %.2e m\n', ...
            '', S.settle_1mm, T_ON, S.max_ee_lead, S.lag_bound);
    fprintf('%-17s attitude error after t/4: same-sample rms %.3f deg, max %.3f deg; lead-corrected rms %.2e deg, max %.2e deg\n', '', S.rmse_att, S.max_att, S.rmse_att_lead, S.max_att_lead);
end
fprintf('\nruns written to %s\n', outdir);
end

function ts = settle_time(t, E, thr, t0)
% First time >= t0 after which E stays below thr for the rest of the run.
idx = find(t >= t0 & E >= thr, 1, 'last');
if isempty(idx), ts = t0; elseif idx == numel(t), ts = NaN; else, ts = t(idx+1); end
end

function wf(p,s), f=fopen(p,'w'); fwrite(f,s); fclose(f); end

function A = as_rows(A)
if size(A,1) < size(A,2), A = A.'; end
end

function orig = claim_constants()
% Same custody rule as gate_all: refuse to start over an interrupted run.
lock = 'define_constants.PRISTINE';
if isfile(lock)
    error('constants:interrupted', ...
        ['a previous run was interrupted and left define_constants.m modified.\n' ...
         '  The unmodified text is in %s.  Restore it with:  ' ...
         'copyfile(''%s'', ''define_constants.m''); delete(''%s'')'], lock, lock, lock);
end
orig = fileread('define_constants.m');
wf(lock, orig);
end

function release_constants(orig)
wf('define_constants.m', orig);
lock = 'define_constants.PRISTINE';
if isfile(lock), delete(lock); end
end
