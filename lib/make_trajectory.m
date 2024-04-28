function traj = make_trajectory(type, params)

    traj.t = params.t(1):params.time_step:params.t(2); 
    if strcmp(type, 'cubic')
        A1 = @(t) [1 t t^2 t^3];
        A2 = @(t) [0 1 2*t 3*t^2];
        
        A = [A1(params.t(1));A2(params.t(1));A1(params.t(2));A2(params.t(2))];
        B = [params.q(1);params.v(1);params.q(2);params.v(2)];
        x = A\B;
        
        q = @(x,t) x(1) + x(2)*t + x(3)*t^2 + x(4)*t^3;
        v = @(x,t) x(2) + 2*x(3)*t + 3*x(4)*t^2;
        a = @(x,t) 2*x(3) + 6*x(4)*t;
        
        for i=1:length(traj.t)
            traj.q(i) = q(x, traj.t(i));
            traj.v(i) = v(x, traj.t(i));
            traj.a(i) = a(x, traj.t(i));
        end
    
    elseif strcmp(type, 'quintic')
        A1 = @(t) [1 t t^2 t^3 t^4 t^5];
        A2 = @(t) [0 1 2*t 3*t^2 4*t^3 5*t^4];
        A3 = @(t) [0 0 2 6*t 12*t^2 20*t^3];
        
        A = [A1(params.t(1));A2(params.t(1));A3(params.t(1));A1(params.t(2));A2(params.t(2));A3(params.t(2))];
        B = [params.q(1);params.v(1);params.a(1);params.q(2);params.v(2);params.a(2)];
        x = A\B;

        q = @(x,t) x(1) + x(2)*t + x(3)*t^2 + x(4)*t^3 + x(5)*t^4 + x(6)*t^5;
        v = @(x,t) x(2) + 2*x(3)*t + 3*x(4)*t^2 + 4*x(5)*t^3 + 5*x(6)*t^4;
        a = @(x,t) 2*x(3) + 6*x(4)*t + 12*x(5)*t^2 + 20*x(6)*t^3;
        
        for i=1:length(traj.t)
            traj.q(i) = q(x, traj.t(i));
            traj.v(i) = v(x, traj.t(i));
            traj.a(i) = a(x, traj.t(i));
        end
    else
        error("Invalid input")
    end
end
