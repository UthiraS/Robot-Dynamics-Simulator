function path = sprial(nPts)

    fprintf('Generating task space path... ');
    phi = linspace(0, 4*pi, nPts);
    r = linspace(0, 0.3, nPts) ;
    x = r .* cos(phi) + 0.4;
    y = r  .* sin(phi);
    z = 0.2 * ones(1,nPts);
    path = [x; y; z];
    fprintf('Done.\n');
    display(path);
    % fprintf('Calculating the Inverse Kinematics... ');
    % robot.plot(zeros(1,6)); hold on;
    % scatter3(path(1,:), path(2,:), path(3,:), 'filled');
    title('Inverse Dynamics Control');

end