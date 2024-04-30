function J_a = jacoba(S,M,q)    
        
    
    % Determine the number of joints n by the number of columns in S
    n = size(S, 2);
    
    % Validate that n is either 3 or 6
    %if n ~= 3 && n ~= 6
    %    error('The number of joints (n) must be either 3 or 6.');
    %end
    
    
    T = fkine(S, M, q,'sapce');   % Compute forward kinematics to get current end-effector transformation matrix
    J_s = jacob0(S, q);   % Compute the space Jacobian

    % Extract position vector p from the transformation matrix T
    p = T(1:3, 4);
    
    % Construct the skew-symmetric matrix of p
    p_hat = [0, -p(3), p(2); 
             p(3), 0, -p(1); 
             -p(2), p(1), 0];
    
    % Split J_s into linear (J_v) and angular (J_omega) components
    J_v = J_s(4:6, :);
    J_omega = J_s(1:3, :);

    % Compute the analytical Jacobian using the derived formula
    % Adjust the calculation for the skew-symmetric matrix depending on n
    %if n == 3
    %    display(n);
    %    J_a = [J_v; J_omega] - [p_hat*J_omega(1:3, 1:3); zeros(3, 3)];
    %elseif n == 6
    %    display(n);
    %    J_a = [J_v; J_omega] - [p_hat*J_omega; zeros(3, 6)];
    %end
    J_a = (J_v - p_hat * J_omega);
   


end


function J = jacob0(S,q)
   dim = size(S);
    n = dim(2);
    T_matrix_list = cell(1, n);     
    J = S(:,1);
    T1 =  twist2ht(S(:,1),q(1));
    product = T1;
    for i = 2:n
        T_mat = twist2ht(S(:,i),q(i));        
        J_i = adjoint(S(:,i), product);
        product = product * T_mat;
        J = [J J_i];
    end 
    J_s = J;
end

function T = fkine(S,M,q,frame)
    % your code here
    dim = size(q);
    T = eye(4);
    %PoE formulae for Transofrmation calculations
    for i=1:dim(2)
        T = T*twist2ht(S(:,i),q(i));
    end
    if (frame == "body")
        T = M * T;
    else
        T = T * M;
    end
    

end


function Vtrans = adjoint(V,T)
    % your code here
    % Extract the rotation matrix R and the translation vector p from T
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    
    % Construct the skew-symmetric matrix from p
    p_hat = [0, -p(3), p(2); p(3), 0, -p(1); -p(2), p(1), 0];
    
    % Construct the adjoint representation Ad_T
    Ad_T = [R, zeros(3,3); p_hat*R, R];
    
    % Transform the twist vector
    Vtrans = Ad_T * V;
    
end



function T = twist2ht(S,theta)
    % your code here
    
   % Extract angular velocity (omega) and linear velocity (v) from the twist
    omega = S(1:3);
    v = S(4:6);
    
    % Create the skew-symmetric matrix of omega
    omega_skew = [0, -omega(3), omega(2); omega(3), 0, -omega(1); -omega(2), omega(1), 0];
    
    % Construct the 4x4 matrix omega_hat
    omega_hat = zeros(4, 4);
    omega_hat(1:3, 1:3) = omega_skew;
    omega_hat(1:3, 4) = v;
    
    % Calculate the homogeneous transformation matrix using matrix exponential
    T = expm(omega_hat * theta);
    
end