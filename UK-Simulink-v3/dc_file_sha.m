function h = dc_file_sha(f)
%DC_FILE_SHA  SHA-256 of a file's bytes, as lower-case hex (char 1x64). F1 fix round 2 (controller ruling R1).
%   Used twice, so the two cannot disagree on how the hash is made:
%     define_constants.m stores dc_file_sha(<its own path>) in the base workspace as dc_sha_at_run;
%     arm_freeze_banner.m recomputes it for which('define_constants') at every Run and refuses the run
%     (arm_freeze_banner:stale) when the two differ: define_constants.m changed since it was last run, so the
%     workspace (arm_freeze_user included) may belong to another version of the file.
%   f  file path (char). Errors if the file cannot be opened.
fid = fopen(f, 'r');
assert(fid > 0, 'dc_file_sha:open', 'cannot open %s', f);
b = fread(fid, Inf, '*uint8');                 % file bytes (uint8 column)
fclose(fid);
md = java.security.MessageDigest.getInstance('SHA-256');
h = lower(reshape(dec2hex(typecast(md.digest(b), 'uint8'), 2).', 1, []));   % hex digest (char)
end
