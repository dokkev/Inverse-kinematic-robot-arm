
% BME_ENG 467 HW3 main file
% Written by Dong Ho Kang

%% Denavit-Hartenberg Forward Kinematics Plot q=[80, 20, 20]

q=[80 20 20]*pi/180;

%A = DHA(length, z, theta, twist)
A1 = DHA(0, 0.3, q(1), pi/2);
A2 = DHA(0.3, 0, q(2), 0);
A3 = DHA(0.2, 0, q(3), 0); 

T1_ = A1;
T2_ = A1*A2;
T3_ = A1*A2*A3;


% Plot Setting
plot3([-.5 .5 .5],[-.5 -.5 .5],[.8 .8 0],'w.') % invisible pts
hold on;
plot3([0 .2],[0 0],[0 0]); % x0 axis
text(.2,0,0,'x0')
plot3([0 0],[0 .2],[0 0]); % y0 axis
text(0,.2,0,'y0')
plot3([0 0],[0 0],[0 .3]); % z0 axis
text(0,0,.2,'z0')
axis equal; 
grid on;

% Plot
plot3([0, T1_(1,4), T2_(1,4), T3_(1,4)],[0,  T1_(2,4), T2_(2,4), T3_(2,4)],[0, T1(3,4), T2_(3,4), T3_(3,4)],'o');
plot3([0, T1_(1,4), T2_(1,4), T3_(1,4)],[0,  T1_(2,4), T2_(2,4), T3_(2,4)],[0, T1(3,4), T2_(3,4), T3_(3,4)],'-');
title('Figure with q=[80, 20, 20]')


%% Figure with MOV=(pi/4:0.08:3*pi/4)
figure
MOV=(pi/4:0.08:3*pi/4);
count = 20;
for i=1:count
    q = [MOV(i), MOV(i), MOV(i)];
    A1 = DHA(0, 0.3, q(1), pi/2);
    A2 = DHA(0.3, 0, q(2), 0);
    A3 = DHA(0.2, 0, q(3), 0); 
    T11 = A1;
    T21 = A1*A2;
    T31 = A1*A2*A3;

    % Plot Setting

    plot3([-.5 .5 .5],[-.5 -.5 .5],[.8 .8 0],'w.') % invisible pts
 
    hold on;
    plot3([0 .2],[0 0],[0 0]); % x0 axis
    text(.2,0,0,'x0')
    plot3([0 0],[0 .2],[0 0]); % y0 axis
    text(0,.2,0,'y0')
    plot3([0 0],[0 0],[0 .3]); % z0 axis
    text(0,0,.2,'z0')
    axis equal; 
    grid on;
 
   
    % Plot
    plot3([0, T11(1,4), T21(1,4), T31(1,4)],[0,  T11(2,4), T21(2,4), T31(2,4)],[0, T11(3,4), T21(3,4), T31(3,4)],'o');
    plot3([0, T11(1,4), T21(1,4), T31(1,4)],[0,  T11(2,4), T21(2,4), T31(2,4)],[0, T11(3,4), T21(3,4), T31(3,4)],'-');
   
     hold off
    pause(.1) % slow down for animation
    title('MOV=(pi/4:0.08:3*pi/4)')
end

%% Inverse Kinematics Problem a - all in 1st quadrant
% Forward Kinematics with given angles to calculate parameters
figure
MOV=(0:0.08:pi/4)'; q=[MOV MOV MOV];
count = 10;
for i=1:count
     q = [MOV(i), MOV(i), MOV(i)];
     
    A1 = DHA(0, 0.3, q(1), pi/2);
    A2 = DHA(0.3, 0, q(2), 0);
    A3 = DHA(0.2, 0, q(3), 0); 
    T1 = A1;
    T2 = A1*A2;
    T3 = A1*A2*A3;

    % Plot Setting

    plot3([-.5 .5 .5],[-.5 -.5 .5],[.8 .8 0],'w.') % invisible pts
    hold on;
    plot3([0 .2],[0 0],[0 0]); % x0 axis
    text(.2,0,0,'x0')
    plot3([0 0],[0 .2],[0 0]); % y0 axis
    text(0,.2,0,'y0')
    plot3([0 0],[0 0],[0 .3]); % z0 axis
    text(0,0,.2,'z0')
    axis equal; 
    grid on;
    % Plot
    plot3([0, T1(1,4), T2(1,4), T3(1,4)],[0,  T1(2,4), T2(2,4), T3(2,4)],[0, T1(3,4), T2(3,4), T3(3,4)],'o');
    plot3([0, T1(1,4), T2(1,4), T3(1,4)],[0,  T1(2,4), T2(2,4), T3(2,4)],[0, T1(3,4), T2(3,4), T3(3,4)],'-');
    pause(.1)
    hold off
    title('Problem a forwardK Plot')

end

% Theta comparision
p = [T3(1,4),T3(2,4),T3(3,4)];
a = [0.3, 0.3, 0.2];
d = [T1(3,4),T2(3,4),T3(3,4)];
alpha = [asin(T1(3,2)),asin(T2(3,2)),asin(T2(3,2))];
theta_result = invKin(p,a,d,alpha); %error
disp(theta_result)

figure
scatter([1,2,3],[theta_result(1),theta_result(2),theta_result(3)],'filled')
hold on
scatter([1,2,3],[pi/4,pi/4,pi/4,],'filled')
hold on
grid on
legend('theta after invK','original theta')
xlim([-1,4])
title('Problem a result Plot')



%% Inverse Kinematics Problem b - "jump"
figure
MOV=(pi/8:0.08:3*pi/2)'; q=[MOV MOV MOV];
count = 20;
for i=1:count
     q = [MOV(i), MOV(i)/2, MOV(i)/3];
     
    A1 = DHA(0, 0.3, q(1), pi/2);
    A2 = DHA(0.3, 0, q(2), 0);
    A3 = DHA(0.2, 0, q(3), 0); 
    T1 = A1;
    T2 = A1*A2;
    T3 = A1*A2*A3;

    % Plot Setting
  
    plot3([-.5 .5 .5],[-.5 -.5 .5],[.8 .8 0],'w.') % invisible pts
    hold on;
    plot3([0 .2],[0 0],[0 0]); % x0 axis
    text(.2,0,0,'x0')
    plot3([0 0],[0 .2],[0 0]); % y0 axis
    text(0,.2,0,'y0')
    plot3([0 0],[0 0],[0 .3]); % z0 axis
    text(0,0,.2,'z0')
    axis equal; 
    grid on;
    % Plot
    plot3([0, T1(1,4), T2(1,4), T3(1,4)],[0,  T1(2,4), T2(2,4), T3(2,4)],[0, T1(3,4), T2(3,4), T3(3,4)],'o');
    plot3([0, T1(1,4), T2(1,4), T3(1,4)],[0,  T1(2,4), T2(2,4), T3(2,4)],[0, T1(3,4), T2(3,4), T3(3,4)],'-');
    pause(.2)
    hold off
    title('Problem b forwardK Plot')

end
% Theta Comparision
p = [T3(1,4),T3(2,4),T3(3,4)];
a = [0.3, 0.3, 0.2];
d = [T1(3,4),T2(3,4),T3(3,4)];
alpha = [asin(T1(3,2)),asin(T2(3,2)),asin(T2(3,2))];
theta_result = invKin(p,a,d,alpha); %error
disp(theta_result)

figure
scatter([1,2,3],[theta_result(1),theta_result(2),theta_result(3)],'filled')
hold on
scatter([1,2,3],[pi/4,pi/8,pi/12,],'filled')
hold on
grid on
legend('theta after invK','original theta')
xlim([-1,4])
title('Problem b result Plot')
