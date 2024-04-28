function [M01, M12, M23, M34, M45, M56, M67] = calculatelinkframes(robot)
  disp(robot)
  Mj1 = tdh(robot.theta(1), robot.d(1), robot.a(1), robot.alpha(1));
  Mj2 = Mj1 * tdh(robot.theta(2), robot.d(2), robot.a(2), robot.alpha(2));
  Mj3 = Mj2 * tdh(robot.theta(3), robot.d(3), robot.a(3), robot.alpha(3));
  Mj4 = Mj3 * tdh(robot.theta(4), robot.d(4), robot.a(4), robot.alpha(4));
  Mj5 = Mj4 * tdh(robot.theta(5), robot.d(5), robot.a(5), robot.alpha(5));
  Mj6 = Mj5 * tdh(robot.theta(6), robot.d(6), robot.a(6), robot.alpha(6));
  

M = @(r) [eye(3) r; 0 0 0 1];
M1 = Mj1*M(robot.links(1).A(0)*robot.links(1).r');
M2 = Mj2*M(robot.links(2).A(0)*robot.links(2).r');
M3 = Mj3*M(robot.links(3).A(0)*robot.links(3).r');
M4 = Mj4*M(robot.links(4).A(0)*robot.links(4).r');
M5 = Mj5*M(robot.links(5).A(0)*robot.links(5).r');
M6 = Mj6*M(robot.links(6).A(0)*robot.links(6).r');


   
  M01 = M1;
  M12 = pinv(pinv(M2)*M1);
  M23 = pinv(pinv(M3)*M2);
  M34 = pinv(pinv(M4)*M3);
  M45 = pinv(pinv(M5)*M4);
  M56 = pinv(pinv(M6)*M5);
  M67 = eye(4);
end
function T = tdh(theta, d, a, alpha)
    %% your code here
    
    rot_z = [cos(theta), - sin(theta), 0, 0;
             sin(theta), cos(theta), 0, 0;
             0, 0, 1, 0;
             0, 0, 0, 1;];
    t_z   = [ 1, 0, 0, 0;
              0, 1, 0, 0;
              0, 0, 1, d;
              0, 0, 0, 1];
    t_x   = [ 1, 0, 0, a;
              0, 1, 0, 0;
              0, 0, 1, 0;
              0, 0, 0, 1;];
    rot_x = [1, 0, 0, 0;
             0, cos(alpha), -sin(alpha), 0;
             0, sin(alpha), cos(alpha), 0;
             0, 0, 0, 1;];
    
    T = rot_z*t_z*t_x*rot_x;
              
    
    
end
