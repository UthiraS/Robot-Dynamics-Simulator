# Robot-Dynamics-Simulator

##steps to run the code
Run GUI.m file to visualize the GUI

in GUI.m
use robot_stack() function to enter ftip or path and get torque profile and joint acc, vel, positions 
use animate_robot() to get "one time simulation" of spiral path and visualize in gui and plot corresponding torque
use externalForceCallback(src, event)-to detect user inputed force and update Ftip value
use externalPathCallback(src, event)-to detect user inputed path points and update path value


funtions in othe files
1. ik.m - function waypoints =ik(S,M,n,nPts,currentQ,path)
takes given path, S, M as input 
gives waypoints as output

2. robot_stack() - function [robot,tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] = robot_stack(Ftip);
takes Ftip as input from user and 
gives output
robot - for plotting
tau_acc - torque 
jointPos_acc- joint accelerations
jointVel_acc - joint velocities
jointAcl_acc - joint positions
t_acc - time

3. gen_traj_torq()- function [tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] =gen_traj_torq(S,M, Mlist, Glist,g,n,nPts,waypoints,Ftip)

input 0 Mlist, Glist, S,M, waypoints, Ftip
gives output

tau_acc - torque 
jointPos_acc- joint accelerations
jointVel_acc - joint velocities
jointAcl_acc - joint positions
t_acc - time
