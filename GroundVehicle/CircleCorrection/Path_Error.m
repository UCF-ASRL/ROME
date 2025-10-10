% to compute error, we calculate the actual x,y coordinates of a circle, normalize
% the caputure postition and calcutlate the error: actual - caputured. to do so,
% some period for the circle must be given, some time period of the capture must be given.

function ERR = Path_Error(time,x_cap,y_cap,x_start,y_start, R, omega, n)
x_calc = zeros(size(time));
y_calc = zeros(size(time));
x_err  = zeros(size(time));
y_err  = zeros(size(time));

    for i = 1:n
    x_calc(i) = xpath(time(i),x_start, R, omega);
    y_calc(i) = ypath(time(i),y_start, R, omega);
    end
    for i = 1:n
    x_err(i) = (x_calc(i)- x_cap(i));
    y_err(i) = (y_calc(i)- y_cap(i));    
    end
    x_err = x_err';
    y_err = y_err';
    
    ERR(:,1) = x_err;
    ERR(:,2) = y_err;
    end