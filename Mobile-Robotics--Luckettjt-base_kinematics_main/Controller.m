function u = Controller(x, Ydesired, A, B, C, P, Q, R)
    g = ((P*B*(inv(R))*B'-A')^-1)*C*Q*C*Ydesired;
    u = -(inv(R))*B'*P*x+(inv(R))*B'*g;
end 
