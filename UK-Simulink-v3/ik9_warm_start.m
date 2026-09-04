function [q0, pe_achieved] = ik9_warm_start(q_seed, p_des, u_des)
%IK9_WARM_START  Damped least squares IK for one end-effector pose.
%   The 9-DOF model starts from a
%   configuration that ACTUALLY REACHES the reference at t = 0. Starting
%   away from it drove the solve into a singularity
%   on an earlier attempt.
%IK_WARM_START  Damped least squares inverse kinematics for one pose.
%
%   Iterates  dq = J'(JJ' + lambda^2 I)^-1 (-e)  on the 6-vector pose error,
%   using the same world-frame quaternion error the block itself uses so the
%   two are consistent. Damping keeps the step bounded near a singularity
%   (Wampler 1986; Nakamura & Hanafusa 1986). This mirrors ik_pose in
%   rome_9dof_uk.m; it is duplicated here so the test stays self-contained.
%
%   q_seed  9x1  starting guess, m and rad
%   p_des   3x1  desired end effector position, world (m)
%   u_des   4x1  desired attitude, unit quaternion, scalar first
%   q       9x1  configuration reaching that pose
    q   = q_seed;
    lam = 0.05;         % damping factor (dimensionless)
    e   = ones(6,1);    % 6x1 pose error, m and rad
    for it = 1:400                  % it: iteration counter
        [pe, Re] = blk_fk(q);       % achieved pose
        du = qm(r2q(Re), [u_des(1); -u_des(2:4)]);      % 4x1 error quaternion
        if du(1) < 0, du = -du; end                     % short way round
        e = [pe - p_des; du(2:4)];
        if norm(e) < 1e-10, break; end
        J = blk_jac(q);             % 6x9 end effector Jacobian
        dq = J.'*((J*J.' + lam^2*eye(6))\(-e));
        if norm(dq) > 0.25, dq = 0.25*dq/norm(dq); end  % cap the step
        q = q + dq;
    end
    if norm(e) > 1e-6
        warning('rome:ik','IK warm start reached only %.2e m/rad.', norm(e));
    end
    q0 = q;
    pe_achieved = blk_fk(q0);   % 3x1 achieved EE position (m)
end

function [pe,Re] = blk_fk(q)
% Forward kinematics written out again here, deliberately, so the achieved
% pose is measured by code INDEPENDENT of the block being tested. If both
% used the same routine, a kinematics error would cancel and the tracking
% numbers would look perfect while being wrong.
%   q   9x1  configuration, m and rad
%   pe  3x1  end effector position, world (m)
%   Re  3x3  end effector orientation, world (dimensionless)
    dh = [0,pi/2,0.169,0; 0.305,0,0,-pi/2; 0,pi/2,0,0; 0,-pi/2,0.222,0; 0,pi/2,0,0; 0,0,0.036,0];
                % DH table, columns [a (m), alpha (rad), d (m), offset (rad)]
                % PLACEHOLDER values, mirroring rome_uk_block's own table
    th=q(3);    % base yaw angle (rad)
    T=[cos(th) -sin(th) 0 q(1); sin(th) cos(th) 0 q(2); 0 0 1 0.20; 0 0 0 1];
                % 4x4 running transform, seeded at the base. 0.20 m is the
                % height of joint 1 above the base plate.
    for i=1:6                   % i: joint index, 1 to 6
        a_=dh(i,1);             % link length a (m)
        al=dh(i,2);             % link twist alpha (rad)
        d_=dh(i,3);             % link offset d (m)
        off=dh(i,4);            % constant joint angle offset (rad)
        ti=q(3+i)+off;          % actual joint angle (rad)
        ct=cos(ti); st=sin(ti); ca=cos(al); sa=sin(al);  % DH trig terms
        T=T*[ct -st*ca st*sa a_*ct; st ct*ca -ct*sa a_*st; 0 sa ca d_; 0 0 0 1];
    end
    Re=T(1:3,1:3); pe=T(1:3,4);
end

function J = blk_jac(q)
%BLK_JAC  End effector Jacobian by central differences of BLK_FK.
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
        [pp, Rp] = blk_fk(q + dq_k);
        [pm, Rm] = blk_fk(q - dq_k);
        W = (Rp - Rm)*((Rp + Rm)/2).';      % skew of the rotation change
        J(:,k) = [(pp - pm); W(3,2); W(1,3); W(2,1)]/(2*h);
    end
end

function u=r2q(R)
% Quaternion from a rotation matrix, Shepperd's branching method. Written
% out again here so the test does not depend on the implementation it is
% checking.
%   R  3x3  rotation matrix (dimensionless)
%   u  4x1  unit quaternion, scalar first (dimensionless)
    tr=trace(R);    % trace, = 1 + 2 cos(angle) (dimensionless)
    if tr>0, S=sqrt(tr+1)*2; u=[0.25*S;(R(3,2)-R(2,3))/S;(R(1,3)-R(3,1))/S;(R(2,1)-R(1,2))/S];
    elseif R(1,1)>R(2,2)&&R(1,1)>R(3,3), S=sqrt(1+R(1,1)-R(2,2)-R(3,3))*2; u=[(R(3,2)-R(2,3))/S;0.25*S;(R(1,2)+R(2,1))/S;(R(1,3)+R(3,1))/S];
    elseif R(2,2)>R(3,3), S=sqrt(1+R(2,2)-R(1,1)-R(3,3))*2; u=[(R(1,3)-R(3,1))/S;(R(1,2)+R(2,1))/S;0.25*S;(R(2,3)+R(3,2))/S];
    else, S=sqrt(1+R(3,3)-R(1,1)-R(2,2))*2; u=[(R(2,1)-R(1,2))/S;(R(1,3)+R(3,1))/S;(R(2,3)+R(3,2))/S;0.25*S]; end
    u=u/norm(u);
end

function c=qm(a,b)
% Hamilton quaternion product, written out again so the test is independent
% of util/quat_mul. The PLUS on the cross term is what makes it Hamilton.
%   a, b  4x1  unit quaternions, scalar first
%   c     4x1  the product a (x) b
  c=[a(1)*b(1)-a(2:4).'*b(2:4); a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4))];
end

