function [fl, tt, idx] = lab_rec_to_floor(rec, cal)
%LAB_REC_TO_FLOOR  Floor-frame base pose [x; y; psi] of every accepted frame of a recording.
%   [fl, tt, idx] = LAB_REC_TO_FLOOR(rec, cal)
%     rec  struct from record_motive (fields t 1xN s, pose7 7xN, ok 1xN logical); only these 3 are used.
%     cal  struct from fit_motive_cal (fields R 3x3, t 3x1 m, scale m per Motive unit), the cal contract
%          of motive_to_floor: p_floor = scale * R * p_motive + t (R Motive -> floor, t floor position of
%          the Motive origin).
%     fl   3xM  [x; y; psi] in the floor frame, M = number of accepted frames: x, y in m, psi in rad
%          in [-pi, pi] (NOT unwrapped; unwrap in the caller if a continuous angle is needed).
%     tt   1xM  receive time of each of those frames (s).
%     idx  1xM  their column indices in rec.
%   The conversion is Optitrack/motive_to_floor.m, called once on the 7xM accepted poses (it is
%   column-wise). There is ONE implementation: the lab gate frame_check, fit_motive_cal's heading
%   residuals and the model block (F5b) all run motive_to_floor (F5a consolidation, 19 Nov 2026;
%   tests/test_motive_floor.m check c). pose7 order: [x y z qx qy qz qw] (natnet_parse_rigidbody).
if ~exist('motive_to_floor', 'file')
    addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'Optitrack'));   % ROME_merged/Optitrack
end
ok = logical(rec.ok(:).');                  % accepted-frame mask (1xN logical)
idx = find(ok);                             % indices of accepted frames (1xM)
if isempty(idx)
    fl = zeros(3, 0);
    tt = zeros(1, 0);
    return
end
fl = motive_to_floor(rec.pose7(:, idx), cal);   % [x; y; psi] (m, m, rad), 3xM
tt = rec.t(idx);
tt = tt(:).';                               % receive times (s), 1xM
end
