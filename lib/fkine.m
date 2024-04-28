function T = fkine(S,M,q,frame)
    % your code here
    
    % If needed, you can convert twists to homogeneous transformation matrices with:
    % twist2ht(S(i),q(i));
    if strcmp(frame, 'body')
        T = M;
        for i = 1:length(q)
            T = T * twist2ht(S(:, i),q(i));
        end
    elseif strcmp(frame, 'space')
        T = M;
        for i = length(q):-1:1
            T = twist2ht(S(:, i), q(i)) * T;
        end
    end
end