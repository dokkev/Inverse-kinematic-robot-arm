clc,clear
global qd TAUd;
global Kp Kd;

initParams;
MT = 2;
td=((0:size(xd,1)-1)/(size(xd,1)-1))*MT';

% Controller gain
Kp = 50*[-10  0;  0   -10];
Kd = 50*[-1    0;  0   -1];


% Configuration setup
q = inverseKinematics(xd);
disp(q)
qdot(2:length(q)-1,:) = (q(3:length(q),:)-q(1:length(q)-2,:))/(2*td(2));
qdot(length(q),:)=0;

qd = [q qdot];

TAUd = zeros(length(xd),2);
dosim
title('	Best Tuned PD controller with invdyn')
