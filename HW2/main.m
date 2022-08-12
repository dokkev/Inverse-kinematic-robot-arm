% main file that runs codes for hw2
% written by Dongho Kang
clc,clear


%% Tip & Elbow Positions Plot using Transformation
Xi = [-0.1,0.3];
Xf = [0.1,0.3];
xi = -0.1; % xi increase up to 0.1 for each interval
yi = 0.3; % This will stay constant throughout the plot
count = 20; % Intervals

figure
for i=1:count+1
    n = (Xf(1)-Xi(1))/count; 

    % Angles of two joints during the plot
    q_hw2 = inverseKinematics([xi,yi]);
    A1 = fkh(0.3, q_hw2(1));            % Elbow Matrix
    A2 = fkh(0.32, q_hw2(2));          % End-Effector Matrix


    % Current Positions of Elbow plot
    M1 = A1;
    x1 = M1(1,3);
    y1 = M1(2,3);
    scatter(x1,y1,'filled','Red')
    hold on
    
    % Current Positions of End-Effector
    M2 = A1*A2;
    x2 = M2(1,3);
    y2 = M2(2,3);
    scatter(x2,y2,'filled','blue')
    hold on
    
    % Arm Plot
    plot([0,x1],[0,y1],'red','LineWidth',2)
    hold on
    plot([x1,x2],[y1,y2],'blue','LineWidth',2)
    hold off
    
    %%%%%%% PLOT SETTING %%%%%%%%%%%%%%%
    % Legend
    legend('elbow','tip','L1','L2')
    grid on
    xlim([-0.15 0.5])
    ylim([0 0.4])
    title('Tip & Elbow Positions Plot using Transformationi')
    xlabel("x")
    ylabel("y")

    pause(.01) % slow down for animation
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Next Interval
    xi = xi + n;
end

%% Positions Comparison with the old one Plot

% Configuration in x-y
Xi = [-0.1,0.3];
Xf = [0.1,0.3];
xi = -0.1;
yi = 0.3;

% make a new window for the animation
figure
pause(1.0)
count = 20; % Intervals

for i=1:count+1
    %%%%%%%%%%%%%%%% OLD PLOT %%%%%%%%%%%%%%%%
    % amount of increasement in x per cycle
    n = (Xf(1)-Xi(1))/count; 
   
    % Angles of two joints during the plot
    q_ani = inverseKinematics([xi,yi]);
    
    % Current configurations of elbow and tip
    [tip_curr,elbow_curr] = forwardKinematics(q_ani);
    
    % Increase in x for the next cycle
    xi = xi + n;
    
    % Joints for the animation
    x_comp = [elbow_curr(1)];
    y_comp = [elbow_curr(2)];
    scatter(x_comp,y_comp,'filled','red')
    hold on
    
    % End-effector for the animation
    x_comp = [tip_curr(1)];
    y_comp = [tip_curr(2)];
    scatter(x_comp,y_comp,'filled','green')
    hold on
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%% NEW PLOT %%%%%%%%%%%%%%%%%%%
    A1 = fkh(0.3, q_ani(1));            % Elbow Matrix
    A2 = fkh(0.32, q_ani(2));% End-Effector Matrix
    
    % Current Positions of Elbow plot
    M1 = A1;
    x1 = M1(1,3);
    y1 = M1(2,3);
    scatter(x1,y1,'*','blue')
    hold on
    
    % Current Positions of End-Effector
    M2 = A1*A2;
    x2 = M2(1,3);
    y2 = M2(2,3);
    scatter(x2,y2,'*','k')
    hold on
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%% PLOT SETTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Label
    xlabel('X-position')
    ylabel('Y-position') 
    % Limit setting
    xlim([-0.15 0.4])
    ylim([0 0.4])
    % Legend
    legend('Old Elbow','Old Tip','New Elbow','New Tip')
    grid on
    title(' Positions Comparison with the old one Plot')
    xlabel("x")
    ylabel("y")

    pause(.01) % slow down for animation
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

%% Plot with 3 links in 2 configuration
clc,clear
figure
q1 = [pi/4,pi/2,-pi/3];
q2 = [(2*pi/3),-pi/4,pi/2];
L = [0.32,0.3,0.4];
 
% check arbitrary number of links
num1 = length(q1);
num2 = length(q2);
disp("Number of links for Arm 1: ")
disp(num1)
disp("Number of links for Arm 2: ")
disp(num2)

A1 = fkh(L(1), q1(1));
A2 = fkh(L(2), q1(2));
A3 = fkh(L(3), q1(3));

A4 = fkh(L(1), q2(1));  
A5 = fkh(L(2), q2(2));
A6 = fkh(L(3), q2(3));

scatter(0,0,'filled','red')
scatter(0,0,'*','k')
hold on
M1 = A1;
x1 = M1(1,3);
y1 = M1(2,3);
scatter(x1,y1,'filled','red')
plot([0,x1],[0,y1],'red','LineWidth',2)
hold on

M2 = A1*A2;
x2 = M2(1,3);
y2 = M2(2,3);
scatter(x2,y2,'filled','red')
plot([x1,x2],[y1,y2],'red','LineWidth',2)
hold on

M3 = A1*A2*A3;
x3 = M3(1,3);
y3 = M3(2,3);
plot([x2,x3],[y2,y3],'red','LineWidth',2)
hold on

M4 = A4;
x4 = M4(1,3);
y4 = M4(2,3);
scatter(x4,y4,'*','k')
plot([0,x4],[0,y4],'k','LineWidth',2)
hold on

M5 = A4*A5;
x5 = M5(1,3);
y5 = M5(2,3);
scatter(x5,y5,'*','k')
plot([x4,x5],[y4,y5],'k','LineWidth',2)
hold on

M6 = A4*A5*A6;
x6 = M6(1,3);
y6 = M6(2,3);
plot([x5,x6],[y5,y6],'k','LineWidth',2)
hold on


%%%%% PLOT SETTING %%%%%
grid on
title("Plot with 3 links in 2 configuration")
xlim([-0.5,0.4]);
ylim([-0.1,0.9]);
xlabel("x")
ylabel("y")


 