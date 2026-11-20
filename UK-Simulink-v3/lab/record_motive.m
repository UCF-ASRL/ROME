function rec = record_motive(seconds, outFile, opts)
%RECORD_MOTIVE  Record the Motive stream (listen only) for some seconds and save it, never overwriting.
%   rec = RECORD_MOTIVE(seconds, outFile)
%   rec = RECORD_MOTIVE(seconds, outFile, opts)
%     seconds  recording length (s), positive finite scalar.
%     outFile  .mat file to create. If it already exists the call errors (record_motive:exists) BEFORE
%              the transport is opened, and again just before saving. An existing file is never touched.
%     opts     optional struct:
%                transport  object with open(), [bytes, why] = receive(), t = time() (s, monotonic),
%                           close(). Tests pass tests/RecorderFakeTransport (no network).
%                live       true to use the real network: MotiveUdpTransport(opts.ip, opts.port), listen only.
%                           Without opts.transport AND without opts.live = true the call errors
%                           (record_motive:noTransport); a socket is never opened by default.
%                rbID       rigid-body Streaming ID to parse (default 1, as MotiveData)
%                layout     byte layout struct for natnet_parse_rigidbody (default [] = the PARSER default: msgId 1, ID 37, pose 41, NO trackedOffset, unlike MotiveData's Layout)
%                ip, port   default '239.255.42.99', 1511 (MotiveData defaults; the lab's are NOT VERIFIED)
%                maxPackets safety cap on receive calls (default 1e6)
%     rec      struct, also saved as variable rec in outFile:
%                t        1xN s, receive time of each datagram, from the transport clock, relative to the start
%                bytes    1xN cell of uint8 columns, the raw datagrams
%                pose7    7xN double [x y z qx qy qz qw] RAW (natnet_parse_rigidbody; NaN where not ok)
%                ok       1xN logical, frame accepted by the parser
%                why      1xN cell of char, the parser's reason ('' if ok)
%                nTimeouts  count of receive calls that returned nothing
%                nReceives  count of receive calls
%                seconds, rbID, layout, ip, port, started_utc (char), file
%   Nothing is sent to Motive or the robot. Zero datagrams still saves the file and warns.
if nargin < 3 || isempty(opts)
    opts = struct();
end
if ~(isnumeric(seconds) && isscalar(seconds) && isfinite(seconds) && seconds > 0)
    error('record_motive:seconds', 'seconds must be a positive finite scalar');
end
outFile = char(outFile);
[outDir, ~, ext] = fileparts(outFile);
if ~strcmpi(ext, '.mat')
    error('record_motive:ext', 'outFile must end in .mat (save would otherwise add the extension and defeat the overwrite check): %s', outFile);
end
if ~isempty(outDir) && ~isfolder(outDir)
    error('record_motive:folder', 'folder does not exist: %s', outDir);
end
if isfile(outFile)
    error('record_motive:exists', 'refusing to overwrite existing file %s', outFile);
end
if ~isfield(opts, 'rbID'),       opts.rbID = 1; end
if ~isfield(opts, 'layout'),     opts.layout = []; end
if ~isfield(opts, 'ip'),         opts.ip = '239.255.42.99'; end
if ~isfield(opts, 'port'),       opts.port = 1511; end
if ~isfield(opts, 'maxPackets'), opts.maxPackets = 1e6; end
if ~exist('natnet_parse_rigidbody', 'file')
    addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'Optitrack'));
end
if isfield(opts, 'transport') && ~isempty(opts.transport)
    tr = opts.transport;                    % injected transport (tests)
elseif isfield(opts, 'live') && isequal(opts.live, true)
    tr = MotiveUdpTransport(opts.ip, opts.port);   % real network, listen only
else
    error('record_motive:noTransport', 'no transport given and opts.live is not true: refusing to open a socket');
end
tr.open();
closer = onCleanup(@() tr.close());         % release the socket on any exit
tStart = tr.time();                         % clock at start (s)
tList = zeros(1, 0);                        % receive times (s)
bList = cell(1, 0);                         % raw datagrams
pList = zeros(7, 0);                        % parsed poses
okList = false(1, 0);                       % accepted flags
wList = cell(1, 0);                         % reasons
nTimeouts = 0;                              % receives with no datagram (count)
nReceives = 0;                              % receive calls (count)
startedUtc = char(datetime('now', 'TimeZone', 'UTC', 'Format', 'yyyy-MM-dd''T''HH:mm:ss''Z'''));   % wall clock at start
while (tr.time() - tStart) < seconds && nReceives < opts.maxPackets
    [b, ~] = tr.receive();
    tNow = tr.time() - tStart;              % receive time relative to start (s)
    nReceives = nReceives + 1;
    if isempty(b)
        nTimeouts = nTimeouts + 1;
        continue
    end
    [p, ok, why] = natnet_parse_rigidbody(b, opts.rbID, opts.layout);
    tList(1, end + 1) = tNow;               %#ok<AGROW>
    bList{1, end + 1} = b;                  %#ok<AGROW>
    pList(:, end + 1) = p;                  %#ok<AGROW>
    okList(1, end + 1) = ok;                %#ok<AGROW>
    wList{1, end + 1} = why;                %#ok<AGROW>
end
rec = struct('t', tList, 'bytes', {bList}, 'pose7', pList, 'ok', okList, 'why', {wList}, ...
             'nTimeouts', nTimeouts, 'nReceives', nReceives, 'seconds', seconds, 'rbID', opts.rbID, ...
             'layout', opts.layout, 'ip', opts.ip, 'port', opts.port, 'started_utc', startedUtc, 'file', outFile);
if isempty(tList)
    warning('record_motive:noPackets', 'no datagram received in %g s (%d receive calls)', seconds, nReceives);
end
if isfile(outFile)
    error('record_motive:exists', 'file appeared during the recording, not overwriting: %s', outFile);
end
save(outFile, 'rec', '-v7');
end
