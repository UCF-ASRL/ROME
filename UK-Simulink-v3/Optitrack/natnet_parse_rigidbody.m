function [pose7, ok, why] = natnet_parse_rigidbody(bytes, rbID, layout)
%NATNET_PARSE_RIGIDBODY  Pose of one rigid body from a NatNet frame-of-data datagram, checked.
%   [pose7, ok, why] = NATNET_PARSE_RIGIDBODY(bytes, rbID, layout)
%     bytes   uint8 column, the datagram from its first byte (4-byte NatNet header included),
%             only the bytes actually received (MotiveData.receiveBytes truncates to getLength).
%     rbID    Motive Streaming ID of the rigid body expected at layout.rbIdOffset (integer, -).
%     layout  struct of 1-based byte indices: msgIdOffset (uint16 message ID), rbIdOffset (int32
%             rigid-body ID), poseOffset (7 float32: x y z qx qy qz qw). Omitted or empty: the
%             offsets MotiveData.m has always used, msgId at 1, ID at 37-40, pose at 41-68.
%     pose7   7x1 double [x y z qx qy qz qw], RAW: Motive units (metres unless Motive's length
%             unit was changed; the lab's setting is NOT VERIFIED), no origin subtraction, no Euler
%             conversion. NaN(7,1) whenever ok is false, so a rejected frame cannot be used by mistake.
%     ok      true only if every check below passed.
%     why     '' when ok, else the first check that failed (char).
%   Checks, in order: host little-endian; bytes is uint8; packet long enough for the layout;
%   message ID == 7 (NAT_FRAMEOFDATA); rigid-body ID == rbID; all 7 values finite;
%   |norm([qx qy qz qw]) - 1| <= 1e-3.
%   Layout source: docs/verification/NATNET_PACKET_LAYOUT.md (OptiTrack NatNet SDK 3.1.0
%   PacketClient.cpp Unpack, via a third-party mirror). With a NatNet 3.x-4.0 stream, x of the first
%   rigid body is at byte 25 + S + 12 nO (S = marker-set record bytes, nO = unlabeled markers), so the
%   default poseOffset 41 holds only when S + 12 nO = 16; the ID is the 4 bytes before x. For NatNet
%   >= 4.1 the layout differs (community source only). Which one the lab streams is NOT VERIFIED
%   (TRACKER Q-P2a); the checks here make a wrong offset show up as ok = false, not as a pose.
%   Byte order: typecast uses the host order; NatNet is little-endian, so a big-endian host is refused.
persistent hostLE                           % true if this MATLAB host is little-endian (logical)
if isempty(hostLE)
    [~, ~, endian] = computer;              % host byte order ('L' or 'B')
    hostLE = (endian == 'L');
end
pose7 = NaN(7, 1);                          % output until every check passes (Motive units / -)
ok    = false;                              % verdict so far (logical)
why   = '';                                 % reason for rejection (char)
qTol  = 1e-3;                               % allowed |quaternion norm - 1| (-), Plan 2 Task 1 brief
msgFrame = 7;                               % NatNet message ID of a frame of data (NAT_FRAMEOFDATA, PacketClient.cpp L50)
if nargin < 3 || isempty(layout)
    layout = struct('msgIdOffset', 1, 'rbIdOffset', 37, 'poseOffset', 41);   % 1-based byte indices, MotiveData.m defaults
end
if ~hostLE
    why = 'host is big-endian; NatNet packets are little-endian';
    return
end
if ~isa(bytes, 'uint8')
    why = sprintf('bytes is %s, expected uint8', class(bytes));
    return
end
bytes = bytes(:);                           % datagram as a column (uint8)
need = max([layout.msgIdOffset + 1, layout.rbIdOffset + 3, layout.poseOffset + 27]);   % last byte the layout reads (1-based index)
if numel(bytes) < need
    why = sprintf('short packet: %d bytes, layout needs %d', numel(bytes), need);
    return
end
msgId = double(typecast(bytes(layout.msgIdOffset + (0:1)), 'uint16'));   % NatNet message ID (-)
if msgId ~= msgFrame
    why = sprintf('message ID %d, expected %d (frame of data)', msgId, msgFrame);
    return
end
id = double(typecast(bytes(layout.rbIdOffset + (0:3)), 'int32'));         % rigid-body Streaming ID in the packet (-)
if id ~= double(rbID)
    why = sprintf('rigid-body ID %d at bytes %d-%d, expected %d', id, layout.rbIdOffset, layout.rbIdOffset + 3, double(rbID));
    return
end
p = double(typecast(bytes(layout.poseOffset + (0:27)), 'single'));       % x y z (Motive units) qx qy qz qw (-), float32 -> double
p = p(:);
if ~all(isfinite(p))
    why = sprintf('non-finite value in pose %s', mat2str(p.', 6));
    return
end
qn = norm(p(4:7));                          % quaternion norm (-)
if abs(qn - 1) > qTol
    why = sprintf('quaternion norm %.6g, outside 1 +/- %g', qn, qTol);
    return
end
pose7 = p;
ok = true;
end
