function waypoints =ik(S,M,n,nPts,currentQ,path)
    addpath('lib');
    waypoints = zeros(n,nPts); 
    
    % Compute Analytic Jacobian at current pose
    Ja = jacoba(S, M, currentQ);
    ii = 1;
    
    % IK loop
    while ii <= nPts
       
        targetPose = path(:, ii);
        T = fkine(S, M, currentQ, 'space');
        currentPose = T(1:3, 4);
        
        if norm(targetPose - currentPose) > 1e-3
            Ja = jacoba(S, M, currentQ);
            
            % Damped Least Squares to solve IK
            lambda = 0.75;
            deltaQ = Ja' * pinv(Ja * Ja' + lambda^2 * eye(3)) * (targetPose - currentPose);
            currentQ = currentQ + deltaQ';
            T = fkine(S, M, currentQ, 'space');
            currentPose = T(1:3, 4);
        else
            waypoints(:, ii) = currentQ;
            ii = ii + 1;
        end
    end



end