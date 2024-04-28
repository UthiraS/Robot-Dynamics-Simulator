function createRobotSimulatorApp()


    global robotAxes robotPanel plotPanel posXCheckbox posYCheckbox posZCheckbox externalForceField Ftip;
    % Main figure
    f = figure('Name', 'Awesome PUMA 560 Simulator', 'NumberTitle', 'off', ...
               'MenuBar', 'none', 'ToolBar', 'none', 'Position', [100, 110, 1024, 768]);

    % UI panels
    robotPanel = uipanel('Parent', f, 'Title', 'Robot- PUMA 560', 'Position', [0.05, 0.2, 0.6, 0.75]);
    plotPanel = uipanel('Parent', f, 'Title', 'Robot Profile', 'Position', [0.7, 0.05, 0.25, 0.9]);
    controlPanel = uipanel('Parent', f, 'Title', 'Control Panel', 'Position', [0.05, 0.05, 0.6, 0.15]);
    % dataPanel = uipanel('Parent', f, 'Title', 'Simulation Data', 'Position', [0.7, 0.05, 0.25, 0.9]);

    % Axes for robot visualization
   
    robotAxes = axes('Parent', robotPanel, 'Position', [0.05, 0.1, 0.9, 0.85]);
    
    % Create and plot the robot
    robot = make_robot();
    robot.plot(zeros(1,6));
    % % Button to animate robot
    % uicontrol('Parent', robotPanel, 'Style', 'pushbutton', 'String', ' Robot', ...
    %           'Position', [410, 10, 120, 25], 'Callback', @make_robot_callback);

    % Control checkboxes
    posXCheckbox = uicontrol('Parent', controlPanel, 'Style', 'checkbox', 'String', 'X', 'Position', [10, 10, 50, 25]);
    posYCheckbox = uicontrol('Parent', controlPanel, 'Style', 'checkbox', 'String', 'Y', 'Position', [70, 10, 50, 25]);
    posZCheckbox = uicontrol('Parent', controlPanel, 'Style', 'checkbox', 'String', 'Z', 'Position', [130, 10, 50, 25]);
   
    % External force label and field
    uicontrol('Parent', controlPanel, 'Style', 'text', 'String', 'External Force:', 'Position', [190, 10, 100, 25]);
    externalForceField = uicontrol('Parent', controlPanel, 'Style', 'edit', 'String', '0', ...
                                   'Position', [300, 10, 100, 25], 'Callback', @externalForceCallback);

    % Button to animate robot
    uicontrol('Parent', controlPanel, 'Style', 'pushbutton', 'String', 'Animate Robot', ...
              'Position', [410, 10, 120, 25], 'Callback', @animateRobot);

    % Button to start/stop the simulation
    % uicontrol('Parent', controlPanel, 'Style', 'pushbutton', 'String', 'Start Simulation', ...
    %           'Position', [530, 10, 120, 25], 'Callback', @startSimulationCallback);
    % Load and plot torque data
    % Axes for plotting data
    

    % Fetch torque data and joint positions from runme
    % [robot,tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] = robot_stack(); % Make sure runme.m outputs these data
    
    
    
   
   
    
end

function externalForceCallback(src, event)
    global posXCheckbox posYCheckbox posZCheckbox robotAxes robotPanel Ftip;
    forceValue = str2double(src.String);
    if isnan(forceValue) || forceValue < 0
        src.String = '0'; % Reset to default
        disp(['External force set to Zero']);
        Ftip = zeros(6,1);
    else
        disp(['External force set to ', num2str(forceValue)]);
        Ftip = zeros(6,1);

        % Check state of position checkboxes and respond accordingly
        if get(posXCheckbox, 'Value')
            disp('X position is checked.');
            Ftip(4) = forceValue * 9.81;
            % Additional logic for X
        end
        if get(posYCheckbox, 'Value')
            disp('Y position is checked.');
            Ftip(5) = forceValue * 9.81;
            % Additional logic for Y
        end
        if get(posZCheckbox, 'Value')
            disp('Z position is checked.');
            Ftip(6) = forceValue * 9.81;
            % Additional logic for Z
        end
         robotAxes = axes('Parent', robotPanel, 'Position', [0.05, 0.1, 0.9, 0.85]);
         % cla(robotAxes);
        % [robot,tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] = robot_stack(Ftip);
        % plot_function(tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc)
        % Assume applyForceAtTip handles the physical force application logic
         % Just an example, adjust as needed
        
    end
end


% function make_robot_callback(src,event)    
%     robot = make_robot();
%     robot.plot(zeros(1,6));
% end

function animateRobot(src, event)
   global robotAxes robot robotPanel Ftip;
   
   [robot,tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] = robot_stack(Ftip);
   nPts = 100;
  
   path = sprial(nPts);
   robotAxes = axes('Parent', robotPanel, 'Position', [0.05, 0.1, 0.9, 0.85]);
   scatter3(path(1,:), path(2,:), path(3,:), 'filled');
   robot.plot(jointPos_acc(:,1:100:end)','trail',{'r', 'LineWidth', 2});
   plot_function(tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc);
end

function plot_function(tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc)

    global plotPanel;
    % Plotting the joint positions on the  axesPositions
    axesPositions = axes('Parent', plotPanel, 'Position', [0.1, 0.75, 0.8, 0.2]);
    axesVelocities = axes('Parent', plotPanel, 'Position', [0.1, 0.5, 0.8, 0.2]);
    axesAccelerations = axes('Parent', plotPanel, 'Position', [0.1, 0.25, 0.8, 0.2]);
    axesTorques = axes('Parent', plotPanel, 'Position', [0.1, 0.05, 0.8, 0.2]);

    % cla(robotAxes);
    plot(axesPositions, t_acc, jointPos_acc(1,:), 'LineWidth', 2);
    hold(axesPositions, 'on');
    for j = 2:size(jointPos_acc, 1)
        plot(axesPositions, t_acc, jointPos_acc(j,:), 'LineWidth', 2);
    end
    hold( axesPositions, 'off');
    title( axesPositions, 'Joint Positions');
    legend( axesPositions, {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
    xlabel( axesPositions, 'Time [s]'), ylabel( axesPositions, 'Joint Position');
    set( axesPositions, 'FontSize', 14);

    % Plotting the joint velocities on the  axesVelocities
    plot(axesVelocities, t_acc, jointVel_acc(1,:), 'LineWidth', 2);
    hold(axesVelocities, 'on');
    for j = 2:size(jointVel_acc, 1)
        plot(axesVelocities, t_acc, jointVel_acc(j,:), 'LineWidth', 2);
    end
    hold( axesVelocities, 'off');
    title( axesVelocities, 'Joint Velocities');
    legend( axesVelocities, {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
    xlabel(axesVelocities, 'Time [s]'), ylabel( axesVelocities, 'Joint Velocities ');
    set(axesVelocities, 'FontSize', 14);


    % Plotting the joint accelerations on the  axesAccelerations
    plot(axesAccelerations, t_acc, jointAcl_acc(1,:), 'LineWidth', 2);
    hold(axesAccelerations, 'on');
    for j = 2:size(jointAcl_acc, 1)
        plot(axesAccelerations, t_acc, jointAcl_acc(j,:), 'LineWidth', 2);
    end
    hold(axesAccelerations, 'off');
    title(axesAccelerations, 'Joint Accelerations');
    legend(axesAccelerations, {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
    xlabel( axesAccelerations, 'Time [s]'), ylabel(axesAccelerations, 'Joint Accelerations');
    set(axesAccelerations, 'FontSize', 14);

    % Plotting the torque profiles on the axesTorques
    plot(axesTorques, t_acc, tau_acc(1,:), 'LineWidth', 2);
    hold(axesTorques, 'on');
    for j = 2:size(tau_acc, 1)
        plot(axesTorques, t_acc, tau_acc(j,:), 'LineWidth', 2);
    end
    hold(axesTorques, 'off');
    title(axesTorques, 'Joint Torques');
    legend(axesTorques, {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
    xlabel(axesTorques, 'Time [s]'), ylabel(axesTorques, 'Torque [Nm]');
    set(axesTorques, 'FontSize', 14);

end

function startSimulationCallback(src, event)
    disp('Starting simulation...');
    createRobotSimulatorApp();
    % Here you would typically call a function to start the simulation
end
