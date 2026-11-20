function h = settings_sha(dcFile)
%SETTINGS_SHA  ONE SHA-256 over define_constants.m AND its lab_settings.m, as lower-case hex (char 1x64). Focus plan F14a.
%   h = settings_sha(dcFile)
%   dcFile  path of define_constants.m (char). Its TA file is lab_settings_file(dcFile) (the sibling).
%   h = SHA-256 of the ASCII text  'define_constants.m <SHA-256 of its bytes>' LF 'lab_settings.m <SHA-256 of its bytes>' LF
%   (each file hashed by dc_file_sha.m). A change of either file changes h.
%   Used everywhere a "settings as run" hash is made or compared, so they cannot disagree:
%     define_constants.m stores it as dc_sha_at_run (the name is kept from F1: one value, fails closed: a check
%       site not updated to this function compares a define_constants-only hash and refuses the Run);
%     arm_freeze_banner.m recomputes it at every Run (arm_freeze_banner:stale);
%     collect_run_snapshot.m records it with every saved run (dc_sha_file_now);
%     the park record (lab/arm_park_record.m, via prerun_check_core.m) must carry it, so an edit of EITHER file
%       after the calibration refuses a frozen robot run (author: "We need to recalibrate no matter what").
%   Errors (dc_file_sha:open, lab_settings_file:missing) when a file cannot be read.
labFile = lab_settings_file(dcFile);              % the TA file of this define_constants (char)
s = sprintf('define_constants.m %s\nlab_settings.m %s\n', dc_file_sha(dcFile), dc_file_sha(labFile));   % the two hashes (char)
md = java.security.MessageDigest.getInstance('SHA-256');
h = lower(reshape(dec2hex(typecast(md.digest(uint8(s)), 'uint8'), 2).', 1, []));   % hex digest (char)
end
