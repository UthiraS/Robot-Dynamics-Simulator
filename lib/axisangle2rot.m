function R = axisangle2rot(omega,theta)
% ...
    skew = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    
    % omega_skew = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
    R = eye(3) + sin(theta)*skew(omega) + (1 - cos(theta))*skew(omega)^2;
end