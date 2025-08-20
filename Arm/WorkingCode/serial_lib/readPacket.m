function [packet] = readPacket(a)
%READPACKET Summary of this function goes here
%   Detailed explanation goes here
packet = [];

zero = uint8(0);
nextByte = fread(a,1);
while (nextByte ~= zero)
    packet = [packet nextByte];
    nextByte = fread(a,1);
end
packet = cobsi(packet);
end

