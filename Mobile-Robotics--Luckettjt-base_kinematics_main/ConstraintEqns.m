function [A, b] = ConstraintEqns(t)
    A = [1 0 0;
         0 1 0;
         0 0 1];

    b = [-sin(t);
          cos(t);
         0];
    %%((2*tan(t)*sec(t)^2)*(tan(t)^2-sec(t)^2+1))/((tan(t)^2+1)^2)
end