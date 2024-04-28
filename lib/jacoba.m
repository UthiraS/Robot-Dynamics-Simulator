function J_a = jacoba(S,M,q)    
    % your code here
    skew = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    J = jacob0(S,q);
    Jw = J(1:3,:);
    Jv = J(4:6,:);
    T = fkine(S,M,q, 'space');
    J_a = Jv - skew(T(1:3,4))*Jw;
end
