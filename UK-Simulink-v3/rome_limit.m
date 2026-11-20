function v = rome_limit(name)
%ROME_LIMIT  One number of allowed_ranges.m for a model block (focus plan F6). The model's Constant blocks evaluate it at
%   compile, so the camera guard, the arm wait and the run verdict read the SAME constants (one source).
%     rome_limit('camera_guard')    3x1 [dt_9dof; C.cam_wait_s; C.cam_dropout_max_s] (s)
%     rome_limit('arm_wait')        3x1 [dt_9dof; C.arm_wait_s; C.arm_lost_max_s] (s)
%     rome_limit('emulate_dropout') k x 2 dropout windows [t_start t_end) (s) for the emulator (EmuDropout): the base
%                                   variable motive_emulate_dropout when it exists (tests only), else [-1 -1] (none)
%   dt_9dof is read from the base workspace (define_constants.m); errors when it is missing.
[~, C] = allowed_ranges();
switch name
    case 'camera_guard'
        v = [evalin('base', 'dt_9dof'); C.cam_wait_s; C.cam_dropout_max_s];
    case 'arm_wait'
        v = [evalin('base', 'dt_9dof'); C.arm_wait_s; C.arm_lost_max_s];
    case 'emulate_dropout'
        v = [-1 -1];
        if evalin('base', 'exist(''motive_emulate_dropout'', ''var'')') == 1
            v = double(evalin('base', 'motive_emulate_dropout'));
        end
    otherwise
        error('rome_limit:name', 'unknown limit %s', name);
end
end
