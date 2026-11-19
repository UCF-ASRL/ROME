function [z_min, xy_min] = clearance_9dof(q9, geom, opt)
%CLEARANCE_9DOF  Arm link points against the floor and the base footprint, for one 9-DOF state.
%   [z_min, xy_min] = clearance_9dof(q9, geom, opt)
%     q9    9x1 model state [x y theta q1..q6] (m, rad)
%     geom  'paper' | 'ar3'
%             'paper': the model's own kinematics (ik9_fk frame origins; the DH table is a PLACEHOLDER)
%             'ar3'  : the AR3 vendor DH (analysis/ar3_fk) with the joint map the model sends:
%                      firmware deg = arm_sign .* rad2deg(q) + arm_offset_deg, then romefw_to_arcs_deg
%                      (map opt.romefw, default 'rome_dirpin'), no clamp. Mounted where the model puts
%                      the arm base: x, y = 0, yaw 0, height = ik9_fk frame 0 (0.20 m; Task 6 ruling
%                      R6.1). 'ar3' values are logged, not gated, until Plan 2.
%     opt   struct. d, w: base length and width (m, define_constants.m, 0.45 x 0.45). For 'ar3' also
%           arm_sign (1x6, -) and arm_offset_deg (1x6, deg). Optional romefw ('rome_dirpin').
%     z_min   lowest link point, world z (m): the segments shoulder (frame 1) to flange (frame 6).
%             The tool BEYOND the flange (Q-T6a, author's estimate ~0.02 m, not measured) is not included.
%     xy_min  smallest signed horizontal distance of the distal link points (elbow to flange) to the base
%             rectangle d x w centred on the base origin, body axes (m): positive outside, negative inside.
%             No deck height is modelled: negative means the point is over (or under) the footprint.
%   z reference: z = 0 is the plane the sources call the base plate (ik9_fk.m: the arm base is 0.20 m above
%   it), the floor (define_constants.m z_work) and the table (arm_home comment): three names for one plane
%   in the model, not shown to be one plane on the robot. Every z_min shifts one-for-one with the mount
%   height (0.20 m, NOT MEASURED); xy_min depends on the J1 axis x, y on the plate (NOT MEASURED).
%   The metrics are link_clearance, shared with analysis/commanding_check. Flat, no local functions.
if ~isstruct(opt) || ~isfield(opt, 'd') || ~isfield(opt, 'w')
    error('clearance_9dof:opt', 'opt.d and opt.w (base size, m) are required');
end
persistent ready                            % true once analysis/ is known to be on the path (logical)
q9 = q9(:);
if numel(q9) ~= 9, error('clearance_9dof:q9', 'q9 must have 9 elements, got %d', numel(q9)); end
switch geom
    case 'paper'
        [~, ~, Q] = ik9_fk(q9);             % 3x7 model frame origins, base body frame (m)
    case 'ar3'
        if ~isfield(opt, 'arm_sign') || ~isfield(opt, 'arm_offset_deg')
            error('clearance_9dof:opt', 'geom ''ar3'' needs opt.arm_sign and opt.arm_offset_deg');
        end
        mp = 'rome_dirpin';                 % romefw map (-)
        if isfield(opt, 'romefw') && ~isempty(opt.romefw), mp = opt.romefw; end
        if isempty(ready)               % path set up once per session, not per call (the gate calls this per sample)
            if isempty(which('ar3_fk')), addpath(fullfile(fileparts(mfilename('fullpath')), 'analysis')); end
            ready = true;
        end
        [~, ~, Qm] = ik9_fk(q9);            % model frames; only frame 0 (the mount) is used
        fw = opt.arm_sign(:) .* rad2deg(q9(4:9)) + opt.arm_offset_deg(:);   % 6x1 firmware command (deg)
        [~, Pa] = ar3_fk(romefw_to_arcs_deg(fw, mp));   % 3x7 AR3 frame origins, arm base frame (m)
        Q = Pa + Qm(:, 1);                  % mount: translation to frame 0 of the model, yaw 0 (m)
    otherwise
        error('clearance_9dof:geom', 'geom must be ''paper'' or ''ar3'', got ''%s''', string(geom));
end
[z_min, xy_min] = link_clearance(Q, opt.d / 2, opt.w / 2);
end
