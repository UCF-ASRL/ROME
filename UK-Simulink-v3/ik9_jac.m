function J = ik9_jac(q)
%IK9_JAC  End-effector Jacobian for the 9-DOF configuration.
%   Same kinematics the solve uses, so a seed built from it is consistent.
%   Central differences of the forward kinematics.
%
%   Built from the test's own forward kinematics, so the warm start does not
%   borrow the Jacobian from the code under test. Central differences, step
%   1e-6, which is ample for seeding an iteration.
%   q  9x1  configuration, m and rad
%   J  6x9  [linear; angular] Jacobian
    h = 1e-6;
    J = zeros(6,9);
    for k = 1:9                     % k: coordinate being perturbed
        dq_k = zeros(9,1); dq_k(k) = h;
        [pp, Rp] = ik9_fk(q + dq_k);
        [pm, Rm] = ik9_fk(q - dq_k);
        W = (Rp - Rm)*((Rp + Rm)/2).';      % skew of the rotation change
        J(:,k) = [(pp - pm); W(3,2); W(1,3); W(2,1)]/(2*h);
    end
end
