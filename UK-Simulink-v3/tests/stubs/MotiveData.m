classdef MotiveData < matlab.System
    % STUB of MotiveData for tests/test_pacing.m. Same ports as the real block (input Enable;
    % six scalar outputs x y z rx ry rz) but it NEVER opens a multicast or any other socket.
    % test_pacing.m copies it over Optitrack/MotiveData.m in a temporary copy of ROME_merged only.
    methods (Access = protected)
        function [x_pos, y_pos, z_pos, x_rot, y_rot, z_rot] = stepImpl(~, ~)
            [x_pos, y_pos, z_pos, x_rot, y_rot, z_rot] = deal(0);   % m, rad; nothing is connected
        end
        function [sz1,sz2,sz3,sz4,sz5,sz6] = getOutputSizeImpl(~)
            sz1=[1 1]; sz2=[1 1]; sz3=[1 1]; sz4=[1 1]; sz5=[1 1]; sz6=[1 1];
        end
        function [dt1,dt2,dt3,dt4,dt5,dt6] = getOutputDataTypeImpl(~)
            dt1='double'; dt2='double'; dt3='double'; dt4='double'; dt5='double'; dt6='double';
        end
        function [cp1,cp2,cp3,cp4,cp5,cp6] = isOutputComplexImpl(~)
            cp1=false; cp2=false; cp3=false; cp4=false; cp5=false; cp6=false;
        end
        function [fx1,fx2,fx3,fx4,fx5,fx6] = isOutputFixedSizeImpl(~)
            fx1=true; fx2=true; fx3=true; fx4=true; fx5=true; fx6=true;
        end
    end
end
