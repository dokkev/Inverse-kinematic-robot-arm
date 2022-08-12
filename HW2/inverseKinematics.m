% 2D Planar Inverse Kinematic Function with a two-joint (revolute) planar robot
% Written by Dongho Kang

function q = inverseKinematics(X)
% This function calculates the joint angles based on initial and final
% position of the end effector
% INPUT: X matrix containing the configurations of the end effector and lengths of two arms
% OUTPUT: q matrix consists of angles of the two joints

q = [0, 0]; %reset
x = X(1);
y = X(2);

% Arm Length
L1 = 0.3;
L2 = 0.32;

B = acos(((L1^2)+(L2^2)-(x^2)-(y^2))/(2*L1*L2));
A = acos(((x^2)+(y^2)+(L1^2)-(L2^2))/(2*L1*sqrt((x^2)+(y^2))));
G = atan2(y,x);

q(1) = G - A;
q(2) = pi - B;

% Angles
q = [q(1),q(2)];

% Endpoint Poistion
x2 = x - (cos(q(2) + q(1))*L2);
y2 = y - (sin(q(2) + q(1))*L2);

% Elbow Position Vectors
x1 = x2 - (cos(q(1))*L1);
y1 = y2 - (sin(q(1))*L1);


end