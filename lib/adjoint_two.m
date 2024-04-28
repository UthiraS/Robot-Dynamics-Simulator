function Vtrans = adjoint_two(V,T)
    % Adjoint transformation from one twist in a frame to another
    %improper function to transform vector to skew matrix
    skew = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    p = T(1:3, 4);
    
    Vtrans = [T(1:3, 1:3), zeros(3); skew(p)*T(1:3, 1:3), T(1:3, 1:3)]*V;
    
end


