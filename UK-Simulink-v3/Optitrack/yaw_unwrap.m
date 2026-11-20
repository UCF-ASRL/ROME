function psi = yaw_unwrap(psi_raw, psi_prev)
%YAW_UNWRAP  One step of a continuous heading: psi = psi_prev + wrapToPi(psi_raw - psi_prev).
%   psi = YAW_UNWRAP(psi_raw, psi_prev)
%     psi_raw   heading from motive_to_floor (rad), in [-pi, pi]
%     psi_prev  previous continuous heading (rad); the model block seeds it with q0_9dof(3) (F5b)
%     psi       continuous heading (rad): the value congruent to psi_raw modulo 2 pi that is nearest to
%               psi_prev. Element-wise for arrays of equal size.
%   Why: scenario 1 starts about 1 deg from +-pi (CLAUDE.md section 33: yaw -178.8 deg) and its heading
%   turns a full lap, so the raw [-pi, pi] angle jumps by 2 pi; q(3) in the model is continuous.
%   Wrap: d - 2 pi ceil((d - pi) / (2 pi)) maps d to (-pi, pi]; a difference already in (-pi, pi] is
%   returned unchanged, bit for bit (ceil gives 0). The toolbox wrapToPi (toolbox/shared/maputils,
%   R2026a; on the path on this machine, NOT used here) maps to [-pi, pi] instead; the two differ only at a
%   difference of exactly -pi (half a turn in one sample), which is ambiguous either way. Inline so the
%   function needs no toolbox and works in a MATLAB Function block. Tests: tests/test_motive_floor.m iii-b, iii-c.
d = psi_raw - psi_prev;                     % raw change of heading (rad)
d = d - 2*pi*ceil((d - pi) / (2*pi));       % wrapped to (-pi, pi] (rad)
psi = psi_prev + d;                         % continuous heading (rad)
end
