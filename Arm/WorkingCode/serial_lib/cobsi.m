function output = cobsi(input)
	%COBSI Decode a message from Consistent Overhead Byte Stuffing - see
    % http://www.stuartcheshire.org/papers/cobsforton.pdf for algorithm
    % details
    output = zeros(0, length(input), 'uint8'); % preallocate

    while ~isempty(input)

        code = double(input(1)); % code=1+n bytes in block
        
        if code > 1 % copy code-1 bytes 
            output(end+1:end+code-1) = input(2:code); 
        end
        if code ~= 255 % except when code block is max length
            output(end+1) = 0; % re-insert '0'
        end
        input(1:code) = []; % done with this input block
    end 
    
    output(end) = []; % remove trailing zero
end