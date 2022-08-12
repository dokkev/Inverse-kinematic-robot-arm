function [tip,elbow]=forwardKinematics(q)
% [tip,elbow]=forwardKinematics(q)
% FORWARD KINEMATICS. 
% q:      relative joint angles (N X 2)
%	tip: endpoint position vectors where col1 is x and col 2 is y 
%	
%	elbow:  elbow position vectors where col1 is x and col 2 is y; 

global L; %L(1) and L(2) are the link lengths
    L1 = L(1);
    L2 = L(2);
N = size(q);
    for i=1:N(1)
        
    q1 = q(i,1);
    q2 = q(i,2);
        
    % Elbow Position Vectors (N X 2) Matrix
    elbow(i,1) = L1* cos(q1);
    elbow(i,2) = L1* sin(q1);

    
    % Endpoint Poistion (N X 2) Matrix
    tip(i,1) = L1* cos(q1) + L2 * cos((q1)+(q2));
    tip(i,2) = L1* sin(q1) + L2 * sin((q1)+(q2));
    end
    
end