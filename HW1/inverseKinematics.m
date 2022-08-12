% 2D Planar Inverse Kinematic Function with a two-joint (revolute) planar robot
% Written by Dongho Kang


function q=inverseKinematics(X)
% This function calculates the joint angles based on initial and final
% position of the end effector
% INPUT: matrix containing the configurations of the end effector and lengths of two arms
% OUTPUT: q matrix consists of angles of the two joints
L1 = 0.3;
L2 = 0.32;

x = X(1); 
y = X(2);


q2 = pi - acos( (L1^2+L2^2-x^2-y^2) / (2*L1*L2) );
q1 = atan2(y,x) - atan( (L2*sin(q2)) / (L1+L2*cos(q2)) );
display(q2)

q = [q1,q2];
end
