function adV = ad(V)
    omega = V(1:3);
    v = V(4:6);
    skew = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    
    adV = [skew(omega) zeros(3,3); skew(v) skew(omega)];
end