function V_b = twistspace2body(V_s,T)
    % your code here
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    skew = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    expV = @(R, p) [R zeros(3,3); skew(p)*R R];
    
    V_b = expV(R', -R'*p)*V_s;
    
    
end