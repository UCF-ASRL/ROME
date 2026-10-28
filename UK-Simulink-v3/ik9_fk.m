function [pe,Re] = ik9_fk(q)
%IK9_FK  End-effector pose from the 9-DOF configuration.
%   The same kinematics the block uses, so the error metric and the solve
%   agree by construction.
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
