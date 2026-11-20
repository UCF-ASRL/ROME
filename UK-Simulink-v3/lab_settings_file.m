function f = lab_settings_file(dcFile)
%LAB_SETTINGS_FILE  Full path of the TA file lab_settings.m that belongs to a define_constants.m (focus plan F14a).
%   f = lab_settings_file(dcFile)
%   dcFile  path of the define_constants.m being run (char), as define_constants passes it:
%           [mfilename('fullpath') '.m'].
%   f       <folder of dcFile>/lab_settings.m when dcFile is a file named define_constants.m: the TA file is
%           ALWAYS the sibling of the define_constants that runs, never whatever lab_settings.m the MATLAB path
%           finds first (temporary copies in tests each carry their own pair).
%           Otherwise (define_constants' TEXT evaluated with eval/evalc, as tests/test_pacing d8 and
%           tests/test_motive_model v7 do, where mfilename names the caller), the sibling of
%           which('define_constants'), i.e. of the define_constants.m that the current folder or path gives.
%   Errors lab_settings_file:missing when that lab_settings.m does not exist: define_constants then stops with
%   arm_freeze_user still NaN, so the model refuses to run (arm_freeze_banner).
%   Also used by settings_sha.m, so the stale-file hash and the file that is run are the same file.
[d, n, e] = fileparts(char(dcFile));
if ~(strcmp([n e], 'define_constants.m') && isfile(dcFile))
    d = fileparts(which('define_constants'));       % define_constants run as text: the one the path gives (char)
end
f = fullfile(d, 'lab_settings.m');
if ~isfile(f)
    error('lab_settings_file:missing', ['%s does not exist. lab_settings.m (the TA file with the arm map, wheel ' ...
          'ceiling, camera calibration and sensor model) must sit next to define_constants.m. Run refused.'], f);
end
end
