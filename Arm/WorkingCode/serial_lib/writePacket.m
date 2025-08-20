function [] = writePacket(serial,data)
%WRITEPACKET Summary of this function goes here
%   Detailed explanation goes here

encoded = cobs(data);
packet = [encoded 0];
fwrite(serial,packet);

end

