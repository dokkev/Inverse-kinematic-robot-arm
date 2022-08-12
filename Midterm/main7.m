clc,clear
%% Alter the feedback of position in the controller by offsetting it by ±5% for each joint and determine the
% outcome trajectories (4 of them).
global qd TAUd;


initParams;

MT = 2;
td=((0:size(xd,1)-1)/(size(xd,1)-1))*MT';

% Controller gain
Kp = 1.05*50*[-10  0;  0   -10];
Kd = 1.05*50*[-1    0;  0   -1];


% Configuration setup
q = inverseKinematics(xd);
qdot(2:length(q)-1,:) = (q(3:length(q),:)-q(1:length(q)-2,:))/(2*td(2));
qdot(length(q),:)=0;

qd = [q qdot];

TAUd = zeros(length(xd),2);

dosim
title('Kp & Kd varied by 5 %')

Kp = 1.03*50*[-10  0;  0   -10]; Kd = 1.03*50*[-1    0;  0   -1];
dosim
title('Kp & Kd varied by3 %')

Kp = 0.97*50*[-10  0;  0   -10]; Kd = 0.97*50*[-1    0;  0   -1];
dosim
title('Kp & Kd varied by -3 %')

Kp = 0.95*50*[-10  0;  0   -10]; Kd = 0.95*50*[-1    0;  0   -1];
dosim
title('Kp & Kd varied by-5 %')