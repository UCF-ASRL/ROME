function [pe,Re,P] = ik9_fk(q)
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
%   P   3x7  (optional) origins of arm frames 0..6 in the BASE BODY frame (m): origin at the base centre,
%            x, y turned with the base yaw q(3), z = world height. Frame 0 is the arm base (0.20 m up),
%            frame 6 the end effector; clearance_9dof reads the arm's link points from here.
    dh = [0,pi/2,0.169,0; 0.305,0,0,-pi/2; 0,pi/2,0,0; 0,-pi/2,0.222,0; 0,pi/2,0,0; 0,0,0.036,0];
                % DH table, columns [a (m), alpha (rad), d (m), offset (rad)]
                % PLACEHOLDER values, mirroring rome_uk_block's own table
    th=q(3);    % base yaw angle (rad)
    T=[cos(th) -sin(th) 0 q(1); sin(th) cos(th) 0 q(2); 0 0 1 0.20; 0 0 0 1];
                % 4x4 running transform, seeded at the base. 0.20 m is the
                % height of joint 1 above the base plate.
    Pw=zeros(3,7);  Pw(:,1)=T(1:3,4);   % 3x7 origins of frames 0..6, world (m)
    for i=1:6                   % i: joint index, 1 to 6
        a_=dh(i,1);             % link length a (m)
        al=dh(i,2);             % link twist alpha (rad)
        d_=dh(i,3);             % link offset d (m)
        off=dh(i,4);            % constant joint angle offset (rad)
        ti=q(3+i)+off;          % actual joint angle (rad)
        ct=cos(ti); st=sin(ti); ca=cos(al); sa=sin(al);  % DH trig terms
        T=T*[ct -st*ca st*sa a_*ct; st ct*ca -ct*sa a_*st; 0 sa ca d_; 0 0 0 1];
        Pw(:,i+1)=T(1:3,4);     % origin of frame i, world (m)
    end
    Re=T(1:3,1:3); pe=T(1:3,4);
    P=[cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1]*(Pw-[q(1);q(2);0]);
                % 3x7 frame origins in the base body frame (m): world minus base position, turned by -yaw
end
