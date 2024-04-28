function [S,M] = make_kinematics_model(robot)
% MAKE_KINEMATICS_MODEL Calculates the Screw Axes and Home Configuration of
% the UR5 robot.
%
% Inputs: robot - the robot object created by the robotics toolbox
%
% Output: S - 6xn matrix whose columns are the screw axes of the robot
%         M - homogeneous transformation representing the home configuration

% Screw Axes
S = [0 0 1 0 0 0;
     0 1 0 0 0 0;
     0 1 0 0 0 0.4318;
     0 0 1 0.1501 -0.4115 0;
     0 1 0 -0.4331 0 0.4115;
     0 0 1 0.1501 -0.4115 0]';

% Home configuration
M = double(robot.fkine(zeros(1,6)));

end