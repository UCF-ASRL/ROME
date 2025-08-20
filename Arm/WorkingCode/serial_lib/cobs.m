function [output] = cobs(input)
%COBS Encodes the array of values in input using COBS.
%   Assumes that input has less than 254 elements.
%   Otherwise, changes will be needed to this function.

input(end+1) = 0;
len = length(input);
output = zeros(0,len + 2,'uint8');

output(1)= uint8(0);
lastIndex = 1;

for i = 1:len
    if input(i) == 0
        output(lastIndex) = (i + 1) - lastIndex;
        lastIndex = i + 1;
    else
        output(i+1) = input(i);
    end
end        

end
 