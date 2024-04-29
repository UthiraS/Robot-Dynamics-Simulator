function q = p2pControl(vector)
% Vector = x y z p r yaw
    x = vector(1);
    y = vector(1);
    z = vector(1);
    r = vector(1);
    p = vector(1);
    yaw = vector(1);
    R = eul2rotm([r p yaw]);
    p = [x y z]';
    T = [R p; 0 0 0 1];
    q = robot.ikine(T);
end