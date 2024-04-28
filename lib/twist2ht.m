function T = twist2ht(S,theta)
% T = ...
    skew = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    v = S(4:6);
    omega = S(1:3);
    T(1:3, 1:3) = axisangle2rot(omega,theta);
    T(1:3, 4) = (eye(3)*theta + (1-cos(theta))*skew(omega) + (theta - sin(theta))*skew(omega)^2)*v;
    T(4,1:3) = 0;
    T(4,4) = 1;
end