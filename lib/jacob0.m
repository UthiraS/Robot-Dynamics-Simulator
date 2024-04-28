function J = jacob0(S,q) 
% Function to find jacobian given the screw axes and joint values
    T = eye(4);
    J = zeros(6, length(q));
    for i = 1:length(q)
        %finding respective transformation matrices
        t = twist2ht(S(:, i), q(i));
        T = T*t;
        J(:, i) = adjoint_two(S(:, i), T);
    end
end