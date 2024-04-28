function AdT = adjoint(T)
    skew = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    
    p = T(1:3,4);
    R = T(1:3,1:3);
    
    AdT = [R zeros(3,3); skew(p)*R R];
end