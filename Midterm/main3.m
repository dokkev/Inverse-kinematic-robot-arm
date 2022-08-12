%% Qusai Static 10N
clc,clear
global xd;
global td;
global L;

Force = [0,10]';

qd = inverseKinematics(xd);
for i = 1:length(qd)
    % Jacobian
    c1 = cos(qd(i,1));
    s1 = sin(qd(i,2));
    c12 = sin(qd(i,1)+qd(i,2));
    s12 = sin(qd(i,1)+qd(i,2));
    J = [-L(1)*s1-L(2)*s12, -L(2)*s12;
            L(1)*c1+L(2)*c12, L(2)*c12];
        torque = J'*Force;
        tau(i,1) = torque(1);
        tau(i,2) = torque(2);              
end

figure(3);

subplot(2,1,1), plot(td,tau(:,1),'r-');
title('Applying 10 N in y-dir Qusai Static')
ylabel('Torque (Nm)')
legend('shoulder')
subplot(2,1,2), plot(td,tau(:,2),'-b');
legend('elbow')
ylabel('Torque (Nm)')
xlabel('time (sec)')
