function q=inverseKinematics(X)
% q=inverseKinematics(x)
% INVERSE KINEMATICS. 
% q:  relative joint angles;
%	x:  endpoint position vectors where col1 is x and col 2 is y; 


global L; %L(1) and L(2) are the link lengths

%
% Your code goes here
L1 = L(1);
L2 = L(2);
loop = size(X);

for i=1:loop(1)
x = X(i,1); 
y = X(i,2);
q(i,2) = pi - acos( (L1^2+L2^2-x^2-y^2) / (2*L1*L2) );
q(i,1) = atan2(y,x) - atan( (L2*sin(q(i,2))) / (L1+L2*cos(q(i,2))) );


end