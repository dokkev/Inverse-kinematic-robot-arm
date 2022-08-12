% main file that runs codes for hw1
% written by Dongho Kang
clc,clear

%% Forward Kinematics
q = [pi/4,pi/3];
[tip,elbow] = forwardKinematics(q);

% Joint Plot
figure
x_comp = [0,elbow(1)];
y_comp = [0,elbow(2)];
scatter(x_comp,y_comp,'filled')
hold on

% Plot Setting
xlabel('X-position')
ylabel('Y-position')

% Arm Plot
plot([0,elbow(1)],[0,elbow(2)],'LineWidth',2)
hold on
plot([elbow(1),tip(1)],[elbow(2),tip(2)],'LineWidth',2)
grid on;
legend('joints','L1','L2')


%% Inverse Kinematics

% Accuracy Test
X_test = [-0.08,0.3];
accuracy_test = forwardKinematics(inverseKinematics(X_test));
if (abs(accuracy_test-X_test) < 1e-02)
    disp ("Accuracy Test Passed!");
else
    disp ("Accuracy Test Failed.");
end

%% Animation Plot

% Configuration in x-y
Xi = [-0.1,0.3];
Xf = [0.5,0.3];
xi = -0.1;
yi = 0.3;

% make a seprate window for the animation
figure

pause(.5)

count = 20; % Intervals

for i=1:count
    % amount of increasement in x per cycle
    n = (Xf(1)-Xi(1))/count;
    
    % Angles of two joints during animation
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
    scatter(x_comp,y_comp,'filled','blue')
    hold on
    
    % Label
    xlabel('X-position')
    ylabel('Y-position')
    
    % Limit setting
    xlim([-0.15 1.0])
    ylim([-0.1 0.4])
    
    % Arm animation
    plot([0,elbow_curr(1)],[0,elbow_curr(2)],'LineWidth',2)
    hold on
    plot([elbow_curr(1),tip_curr(1)],[elbow_curr(2),tip_curr(2)],'LineWidth',2)
    grid on
    hold off
    
     % Legend
    legend('elbow','tip','L1','L2')
    pause(.3) % slow down for animation
    
 
   
end

