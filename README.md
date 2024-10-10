# Robot-Dynamics-Simulator README
go to `gui` branch and clone the repository.
## Overview
This project outlines the steps and functions used in the Robot-Dynamics-Simulator, a tool for simulating robot dynamics through a graphical user interface (GUI).

## Getting Started
### Steps to Run the Simulator
1. **Start the GUI**: Execute the `GUI.m` file to launch the GUI, which is essential for visualizing robot dynamics.
![Robot Dynamics Simulation](Capture.JPG)
2. Enter a value in any of the field and press `enter` in keyboard and click `animate Robot` button to visulize the movement and profile graphs.


### Functions within `GUI.m`
- **`robot_stack()`**: This function allows users to input `Ftip` (external force at the tip) or a `path point ` and returns the torque profile along with joint accelerations, velocities, and positions.
- **`animate_robot()`**: Executes a one-time simulation of a spiral path. It visualizes the robot's movement in the GUI and plots the corresponding torque.
- **`externalForceCallback(src, event)`**: Detects user-inputted force and updates the `Ftip` value accordingly.
- **`externalPathCallback(src, event)`**: Captures user-inputted path points and updates the path value.

## Functions in Other Files
### `ik.m`
- **Function**: `waypoints = ik(S, M, n, nPts, currentQ, path)`
- **Description**: Calculates waypoints based on the given path, screw axes `S`, home configuration `M`, and the current joint configuration `currentQ`.
- **Output**: `waypoints` - calculated waypoints for the robot.

### `robot_stack()`
- **Function**: `[robot, tau_acc, jointPos_acc, jointVel_acc, jointAcl_acc, t_acc] = robot_stack(Ftip,path)`
- **Description**: Processes the user-inputted `Ftip` or `path` to simulate robot dynamics and calculates various dynamics parameters.
- **Outputs**:
  - `robot` - Data for plotting robot dynamics.
  - `tau_acc` - Accumulated torque.
  - `jointPos_acc` - Joint positions.
  - `jointVel_acc` - Joint velocities.
  - `jointAcl_acc` - Joint accelerations.
  - `t_acc` - Time intervals.

### `gen_traj_torq()`
- **Function**: `[tau_acc, jointPos_acc, jointVel_acc, jointAcl_acc, t_acc] = gen_traj_torq(S, M, Mlist, Glist, g, n, nPts, waypoints, Ftip)`
- **Description**: Generates a trajectory and torque profile based on the input parameters.
- **Inputs**:
  - `Mlist`, `Glist`, `S`, `M`, `waypoints`, `Ftip` - Various robot parameters and dynamics inputs.
  - `g` - Gravitational force vector.
- **Outputs**:
  - `tau_acc` - Torque.
  - `jointPos_acc` - Joint positions.
  - `jointVel_acc` - Joint velocities.
  - `jointAcl_acc` - Joint accelerations.
  - `t_acc` - Time intervals.
