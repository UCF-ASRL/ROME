classdef ROMECommand < matlab.System
    % STUB of ROMECommand for tests/test_pacing.m. Same ports and sizes as the real block
    % (inputs wheelRPM, jointRad, Enable; outputs actualWheelRPM 4x1, actualJointDeg 6x1,
    % armReady logical), but it NEVER opens a socket: no tcpclient, no serial, no network.
    % test_pacing.m copies it over ROMECommand.m in a temporary copy of ROME_merged only.
    methods (Access = protected)
        function [actualWheelRPM, actualJointDeg, armReady] = stepImpl(~, ~, ~, ~)
            actualWheelRPM = zeros(4, 1);   % rpm, nothing is connected
            actualJointDeg = zeros(6, 1);   % deg
            armReady       = false;
        end
        function num = getNumOutputsImpl(~)
            num = 3;
        end
        function [o1,o2,o3] = getOutputSizeImpl(~)
            o1 = [4 1]; o2 = [6 1]; o3 = [1 1];
        end
        function [o1,o2,o3] = getOutputDataTypeImpl(~)
            o1 = "double"; o2 = "double"; o3 = "logical";
        end
        function [o1,o2,o3] = isOutputComplexImpl(~)
            o1 = false; o2 = false; o3 = false;
        end
        function [o1,o2,o3] = isOutputFixedSizeImpl(~)
            o1 = true; o2 = true; o3 = true;
        end
    end
end
