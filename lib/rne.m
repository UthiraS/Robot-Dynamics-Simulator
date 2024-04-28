function [tau,V,Vdot] = rne(params)
% Inputs: params - a structure containing the following fields:
%           params.g - 3-dimensional column vector describing the acceleration of gravity
%           params.S - 6xn matrix of screw axes (each column is an axis)
%           params.M - 4x4xn home configuration matrix for each link
%           params.G - 6x6xn spatial inertia matrix for each link
%           params.jointPos - n-dimensional column vector of joint coordinates
%           params.jointVel - n-dimensional column vector of joint velocities
%           params.jointAcc - n-dimensional column vector of joint accelerations
%           params.Ftip - 6-dimensional column vector representing the
%           wrench applied at the tip
%
% Output: tau  - n-dimensional column vector of generalized joint forces
%         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
%         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
%
% Forward iterations
V = zeros(6, length(params.jointPos)+1);
Vdot = zeros(6, length(params.jointPos)+1);
if size(params.g,1) == 1
    Vdot(:,1) = cat(1, zeros(3,1), -params.g');
else
    Vdot(:,1) = cat(1, zeros(3,1), -params.g);
end

M_i = params.M;

for i=1:length(params.jointPos)
    M_i(:,:,i+1) = M_i(:,:,i)*params.M(:,:,i+1);
    A(:,i) = adjoint(inv(M_i(:,:,i)))*params.S(:,i);
    T(:,:,i) = params.M(:,:,i)*twist2ht(A(:,i),params.jointPos(i));
    V(:,i+1) = A(:,i)*params.jointVel(i) + adjoint(inv(T(:,:,i)))*V(:,i);
    Vdot(:,i+1) = A(:,i)*params.jointAcc(i) + adjoint(inv(T(:,:,i)))*Vdot(:,i) + ad(V(:,i+1))*A(:,i)*params.jointVel(i);
end
% Backward iterations
% YOUR CODE HERE
F_adv = params.Ftip;
tau = zeros(length(params.jointPos),1);
T(:,:,length(params.jointPos)+1) =  params.M(:,:,length(params.jointPos)+1);

for i=length(params.jointPos):-1:1
    F = params.G(:,:,i)*Vdot(:,i+1) - transpose(ad(V(:,i+1)))*params.G(:,:,i)*V(:,i+1) + transpose(adjoint(inv(T(:,:,i+1))))*F_adv;
    tau(i) = F'*A(:,i); 
    F_adv = F;
end
end