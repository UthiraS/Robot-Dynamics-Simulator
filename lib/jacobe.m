function J_b = jacobe(S,M,q)    
    % your code here
    J_s = jacob0(S,q);
    skew = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    expV = @(R, p) [R zeros(3,3); skew(p)*R R];
    T = fkine(S,M,q, 'space');
    AdTinv = expV(T(1:3,1:3), T(1:3,4));
    J_b = AdTinv \ J_s;
end