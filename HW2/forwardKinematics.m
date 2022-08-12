%  2D Planar Forward Kinematic Function with a two-joint (revolute) planar robot
% Written by Dongho Kang

function [tip,elbow] = forwardKinematics(q)
    % This function calculated planar forward kinematics for 2-DOF robot arm
    % INPUT: matrix q contains angles of two joints and lengths of two arms
    % OUTPUT: x and y positions of the end-effector(tip) and the elbow

    L1 = 0.3;
    L2 = 0.32;
   
    % Elbow Position Vectors
    x1 = L1* cos(q(1));
    y1 = L1* sin(q(1));
    elbow = [x1,y1];
    
    % Endpoint Poistion
    x2 = L1* cos(q(1)) + L2 * cos(q(1)+q(2));
    y2 = L1* sin(q(1)) + L2 * sin(q(1)+q(2));
    tip = [x2,y2];
    
  
end

