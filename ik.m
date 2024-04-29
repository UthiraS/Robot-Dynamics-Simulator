function waypoints =ik(S,M,n,nPts,currentQ,V)
    addpath('lib');
    waypoints = zeros(n,nPts); 
    qlim = [-180  180;  % q(1)
            -180  180;  % q(2)
            -180  180;  % q(3)
            -180  180;  % q(4)
            -180  180;  % q(5)
            -180  180]; % q(6)
    qlim = deg2rad(qlim);
    % Compute Analytic Jacobian at current pose
    Ja = jacoba(S, M, currentQ);
    ii = 1;
    
    % IK loop
    while ii <= nPts
       
        
        T = fkine(S, M, currentQ, 'space');
        currentPose = MatrixLog6(T_current);
        currentPose = [currentPose(3,2); currentPose(1,3); currentPose(2,1); T_current(1:3,4)];
        targetPose = V; 
        
        if norm(targetPose - currentPose) > 1e-3
            Ja = jacoba(S, M, currentQ);
            
            % Damped Least Squares to solve IK
            lambda = 0.75;
            deltaQ = Ja' * pinv(Ja * Ja' + lambda^2 * eye(6)) * (targetPose - currentPose);
            currentQ = currentQ + deltaQ';
            currentQ = min(max(qlim(:,1)', currentQ), qlim(:,2)');
            T = fkine(S, M, currentQ, 'space');
            currentPose = MatrixLog6(T);
            currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';
        else
            waypoints(:, ii) = currentQ;
            ii = ii + 1;
        end
    end



end