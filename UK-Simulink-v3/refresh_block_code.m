function refresh_block_code(mdlFile, allowCodeChange)
%REFRESH_BLOCK_CODE  Copy the current rome_uk_block.m and
%EndEffectorTrajectory.m into the UKDynamics and EndEffectorTrajectory
%blocks of an existing ROME_9DOF.slx, without rebuilding the model.
%
%   Layout, colours and wiring are kept. The source transform is the one in
%   build_9dof.m (ukdynamics_source). The model is saved only if the new code
%   differs from the old in comment lines alone; any change to executable
%   code raises an error and nothing is saved.
%
%   Run with Simulink closed: bdclose('all').

if nargin < 1 || isempty(mdlFile)
    mdlFile = fullfile(fileparts(mfilename('fullpath')), 'ROME_9DOF.slx');
end
if nargin < 2
    allowCodeChange = false;   % pass true only when the solve itself changed
end
here = fileparts(mfilename('fullpath'));
[~, mdl] = fileparts(mdlFile);
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system(mdlFile);
cleanup = onCleanup(@() close_system(mdl, 0));

newCode = struct( ...
    'UKDynamics',            ukd_source(fullfile(here, 'rome_uk_block.m')), ...
    'EndEffectorTrajectory', regexprep(fileread(fullfile(here, 'EndEffectorTrajectory.m')), '\r\n?', newline));

rt = sfroot;
names = fieldnames(newCode);
for k = 1:numel(names)
    ch = rt.find('-isa', 'Stateflow.EMChart', 'Name', names{k});
    ch = ch(arrayfun(@(c) strcmp(bdroot(c.Path), mdl), ch));
    assert(numel(ch) == 1, 'refresh_block_code:chart', ...
           '%s: expected 1 chart, found %d', names{k}, numel(ch));
    oldS = code_lines(ch.Script);
    newS = code_lines(newCode.(names{k}));
    if ~isequal(oldS, newS)
        n = min(numel(oldS), numel(newS));
        i = find(~strcmp(oldS(1:n), newS(1:n)), 1);
        if isempty(i), i = n + 1; end
        if ~allowCodeChange
            error('refresh_block_code:code', ...
                  ['%s: executable code differs (first difference at code line %d). ' ...
                   'Nothing saved. Call refresh_block_code(mdl, true) if that is intended.'], ...
                  names{k}, i);
        end
        fprintf('  %-22s CODE CHANGED: %d -> %d code lines, first difference at %d\n', ...
                names{k}, numel(oldS), numel(newS), i);
    else
        fprintf('  %-22s comments updated, %d code lines unchanged\n', names{k}, numel(newS));
    end
    ch.Script = newCode.(names{k});
end
save_system(mdl);
fprintf('  saved %s\n', mdlFile);

% ukd_current.m is the offline twin of the UKDynamics block: sweep_formulation,
% singularity_mechanism and check_mass_floor drive the solve through it rather
% than through Simulink. Until 22 Sep 2026 it was produced by hand, so it could
% silently fall behind rome_uk_block.m -- and did. It is regenerated here, from
% the same transform that writes the chart, so the two cannot diverge.
gen = ukd_source(fullfile(here, 'rome_uk_block.m'));
gen = strrep(gen, ...
    'function [q_cmd, q_dot_cmd, wheel_speeds_rpm, tau_arm, manip, resid] = ', ...
    'function [q_cmd, q_dot_cmd, wheel_speeds_rpm, tau_arm, manip, resid] = ');
gen = strrep(gen, 'UKDynamics(q_meas', 'ukd_current(q_meas');
hdr = ['% UKD_CURRENT  Verbatim copy of the UKDynamics block in ROME_9DOF.slx,' newline ...
       '% renamed so it can be driven outside Simulink. Do not edit; regenerate' newline ...
       '% with refresh_block_code, which writes this file from rome_uk_block.m' newline ...
       '% using the same transform that writes the chart.' newline];
p = fullfile(here, 'ukd_current.m');
old = '';
if isfile(p), old = regexprep(fileread(p), '\r\n?', newline); end
new = [hdr gen];
if strcmp(old, new)
    fprintf('  ukd_current.m already current\n');
else
    f = fopen(p, 'w');  fwrite(f, new);  fclose(f);
    fprintf('  ukd_current.m regenerated from rome_uk_block.m\n');
end
end


function s = ukd_source(f)
% Same transform as build_9dof.m ukdynamics_source.
s = regexprep(fileread(f), '\r\n?', newline);
s = strrep(s, 'function [q_cmd, qd_cmd, wheel_rpm, tau_arm, manip, resid] = ', ...
              'function [q_cmd, q_dot_cmd, wheel_speeds_rpm, tau_arm, manip, resid] = ');
s = strrep(s, 'rome_uk_block(q_meas, qd_meas, p_des, u_des, V_des, A_des, dt, reset)', ...
              'UKDynamics(q_meas, qd_meas, p_des, u_des, V_des, A_des, dt, reset, r, l, alphas)');
s = strrep(s, 'qd_cmd = qd_int;',    'q_dot_cmd = qd_int;');
s = strrep(s, 'wheel_rpm = rome_wheels(', 'wheel_speeds_rpm = rome_wheels(');
s = strrep(s, 'r_w = 0.0762;', 'r_w = r;       ');
s = strrep(s, 'l_w = 0.35;',   'l_w = l;     ');
s = strrep(s, 'alph = deg2rad([315; 225; 135; 45]);', 'alph = alphas(:);');
assert(contains(s, 'alph = alphas(:);') && contains(s, 'UKDynamics(q_meas'), ...
       'refresh_block_code:transform', 'source transform failed');
end


function c = code_lines(s)
% Executable content only: comments and blank lines removed, whitespace
% collapsed. A '%' inside a quoted string is not treated as a comment.
L = splitlines(string(s));
c = strings(0, 1);
for k = 1:numel(L)
    t = strip(strip_comment(L(k)));
    if strlength(t) > 0
        c(end+1, 1) = regexprep(t, '\s+', ' '); %#ok<AGROW>
    end
end
end


function t = strip_comment(line)
ch = char(line);  inQ = false;  t = line;
for i = 1:numel(ch)
    if ch(i) == ''''
        prev = ' ';
        if i > 1, prev = ch(i-1); end
        if ~inQ && (isletter(prev) || isstrprop(prev, 'digit') || any(prev == ')]}.'''))
            continue                       % transpose operator
        end
        inQ = ~inQ;
    elseif ch(i) == '%' && ~inQ
        t = string(ch(1:i-1));
        return
    end
end
end
