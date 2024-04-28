function [robot,tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] = robot_stack(Ftip)

    % Create the environment
    g = [0 0 -9.81]; % Gravity Vector [m/s^2]
    
    % Create the robot and display it in the home configuration
    robot = make_robot();
    % robot.plot(zeros(1,6));
    
    % Create a kinematic model of the robot
    [S,M] = make_kinematics_model(robot);
    n = size(S,2); % read the number of joints
    nPts = 100;
    % Create a dynamical model of the robot
    [Mlist,Glist] = make_dynamics_model(robot);
    
    
    %% POSITION CONTROL  %%
    %% SPIRAL PATH
    fprintf('----------------------Dynamic Control of PUMA560 Arm--------------------\n');
    path = sprial(nPts);
    
    
    
    %% INVERSE KINEMATICS
      
    currentQ = zeros(1,n);
    waypoints =ik(S,M,n,nPts,currentQ,path);
    fprintf('Done.\n');
    
    %% Trajectory and Torque Profile
    % Now, for each pair of consecutive waypoints, we will first calculate a
    % trajectory between these two points, and then calculate the torque
    % profile necessary to move from one point to the next.
    [tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] =gen_traj_torq(S,M, Mlist, Glist,g,n,nPts,waypoints,Ftip);

end
