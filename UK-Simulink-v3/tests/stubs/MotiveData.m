classdef MotiveData < matlab.System
    % STUB of MotiveData for tests/test_pacing.m. Same ports as the real block (input Enable; outputs 1-6 scalar
    % x y z rx ry rz, 7 pose7 7x1, 8 valid, 9 nDropped; Plan 2 Task 1) but it NEVER opens a multicast or any other
    % socket. Outputs 7-9 added by F5b (19 Nov 2026): the model now wires output 7 (pose7) into MotiveFloor, so a
    % 6-output stub no longer fits the block. The stub reports "no frame": NaN pose, valid 0, nDropped 0.
    % test_pacing.m copies it over Optitrack/MotiveData.m in a temporary copy of ROME_merged only.
    methods (Access = protected)
        function [x_pos, y_pos, z_pos, x_rot, y_rot, z_rot, pose7, valid, nDropped] = stepImpl(~, ~)
            [x_pos, y_pos, z_pos, x_rot, y_rot, z_rot] = deal(0);   % m, rad; nothing is connected
            pose7 = NaN(7, 1);                  % no frame (Motive units / -)
            valid = 0;                          % no frame accepted (flag)
            nDropped = 0;                       % consecutive rejected frames (count)
        end
        function [sz1,sz2,sz3,sz4,sz5,sz6,sz7,sz8,sz9] = getOutputSizeImpl(~)
            sz1=[1 1]; sz2=[1 1]; sz3=[1 1]; sz4=[1 1]; sz5=[1 1]; sz6=[1 1];
            sz7=[7 1]; sz8=[1 1]; sz9=[1 1];
        end
        function [dt1,dt2,dt3,dt4,dt5,dt6,dt7,dt8,dt9] = getOutputDataTypeImpl(~)
            dt1='double'; dt2='double'; dt3='double'; dt4='double'; dt5='double'; dt6='double';
            dt7='double'; dt8='double'; dt9='double';
        end
        function [cp1,cp2,cp3,cp4,cp5,cp6,cp7,cp8,cp9] = isOutputComplexImpl(~)
            cp1=false; cp2=false; cp3=false; cp4=false; cp5=false; cp6=false;
            cp7=false; cp8=false; cp9=false;
        end
        function [fx1,fx2,fx3,fx4,fx5,fx6,fx7,fx8,fx9] = isOutputFixedSizeImpl(~)
            fx1=true; fx2=true; fx3=true; fx4=true; fx5=true; fx6=true;
            fx7=true; fx8=true; fx9=true;
        end
    end
end
