function packet = readPacketFast(a,bytes)
%READPACKETFAST Summary of this function goes here
%   Detailed explanation goes here

raw = fread(a,bytes+2);
data = raw(1:end-1);
packet = cobsi(data);

end


