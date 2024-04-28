clear, clc, close all
addpath('../lib');

plotOn = false;

% Create the environment
g = [0 0 -9.81]; % Gravity Vector [m/s^2]

% Create the robot and display it in the home configuration
robot = make_robot();
robot.plot(zeros(1,6));

% Create a kinematic model of the robot
[S,M] = make_kinematics_model(robot);
n = size(S,2); % read the number of joints
nPts = 100;
% Create a dynamical model of the robot
[Mlist,Glist] = make_dynamics_model(robot);


%% Control the motion of the robot between 2 set points %%
%% SPIRAL PATH
fprintf('----------------------Dynamic Control of PUMA560 Arm--------------------\n');
path = sprial(nPts);



%% INVERSE KINEMATICS
   
currentQ = zeros(1,n);
waypoints = ik(S,M,currentQ,path);
fprintf('Done.\n');

%% Trajectory and Torque Profile
% Now, for each pair of consecutive waypoints, we will first calculate a
% trajectory between these two points, and then calculate the torque
% profile necessary to move from one point to the next.
[tau_acc,jointPos_acc, t_acc] =gen_traj_torq(S,M, Mlist, Glist,g,n,nPts,waypoints)

fprintf('\nDone. Simulating the robot...');

%% Animate the robot
title('Inverse Dynamics Control');
% ANimation command did not work on my Ubuntu machine
%robot.plot(jointPos_acc(:,1:100:end)','trail',{'r', 'LineWidth', 2}, 'movie', 'RBE-501-2024-HW4-spiral.mp4');
robot.plot(jointPos_acc(:,1:100:end)','trail',{'r', 'LineWidth', 2});
fprintf('Done.\n');


%% Display the Joint Torques
figure;
hold on;
grid on;

% Plot torque profiles for each joint
plot(t_acc, tau_acc(1,:), 'Linewidth', 2);
plot(t_acc, tau_acc(2,:), 'Linewidth', 2);
plot(t_acc, tau_acc(3,:), 'Linewidth', 2);
plot(t_acc, tau_acc(4,:), 'Linewidth', 2);
plot(t_acc, tau_acc(5,:), 'Linewidth', 2);
plot(t_acc, tau_acc(6,:), 'Linewidth', 2);

title('Torque Profiles');
xlabel('Time [s]'), ylabel('Torque [Nm]');
legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
set(gca, 'FontSize', 14);
