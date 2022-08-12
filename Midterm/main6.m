clc,clear
%%
% OPTION 1: Alter the estimate of mass M of the inverse dynamic controller by ±5% and determine the outcome
% trajectories (4 of them)

clc,clear
global qd TAUd;
M = 1.03*[0.8; 0.8];
initParams_m;

MT = 2;
td=((0:size(xd,1)-1)/(size(xd,1)-1))*MT';

% Controller gain
Kp = 50*[-10  0;  0   -10];
Kd = 50*[-1    0;  0   -1];



% Configuration setup
q = inverseKinematics(xd);
qdot(2:length(q)-1,:) = (q(3:length(q),:)-q(1:length(q)-2,:))/(2*td(2));
qdot(length(q),:)=0;

qd = [q qdot];

TAUd = zeros(length(xd),2);


dosim
title('Mass varied by 5 %')

M = 1.03*[0.8; 0.8];
initParams_m;
dosim
title('Mass varied by 3 %')

M = 0.97*[0.8; 0.8];
initParams_m;
dosim
title('Mass varied by -3 %')

M = 0.95*[0.8; 0.8];
initParams_m;
dosim
title('Mass varied by -5 %')