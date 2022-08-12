clc,clear
%% shortest time with 20Nm torque limit
clc,clear
global td;

initParams2;
% initialize arrarys
q = zeros(length(td),2);
qdot = zeros(length(td),2);
qddot = zeros(length(td),2);
% Configuration setup
q = inverseKinematics(xd);
qdot(2:length(q)-1,:) = (q(3:length(q),:)-q(1:length(q)-2,:))/(2*td(2));
qddot(2:length(q)-1,:) = (qdot(3:length(q),:)-qdot(1:length(q)-2,:))/(2*td(2));
% Calculate Torque

tau = q2tau(L,M,q,qdot,qddot);

% Plot
figure(2);
subplot(2,1,1),plot(td,tau(:,1),'r-');
ylabel('Torque (Nm)')
legend('shoulder')
title('shortest time with 20Nm torque limit')
subplot(2,1,2), plot(td,tau(:,2),'b-');
legend('elbow')
xlabel('time (sec)')
ylabel('torque (Nm)')