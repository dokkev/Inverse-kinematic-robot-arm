
%% Plot with 3 links in 2 configuration
clc,clear
figure
q1 = [pi/4,pi/2,-pi/3];
q2 = [(2*pi/3),-pi/4,pi/2];
L = [0.32,0.3,0.4];
 
% check arbitrary number of links

A1 = fkh(L(1), q1(1));
A2 = fkh(L(2), q1(2));
A3 = fkh(L(3), q1(3));

A4 = fkh(L(1), q2(1));  
A5 = fkh(L(2), q2(2));
A6 = fkh(L(3), q2(3));

scatter(0,0,'filled','k')
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
scatter(x3,y3,'filled','red')
plot([x2,x3],[y2,y3],'red','LineWidth',2)
hold on

M4 = A4;
x4 = M4(1,3);
y4 = M4(2,3);
scatter(x4,y4,'filled','k')
plot([0,x4],[0,y4],'k','LineWidth',2)
hold on

M5 = A4*A5;
x5 = M5(1,3);
y5 = M5(2,3);
scatter(x5,y5,'filled','k')
plot([x4,x5],[y4,y5],'k','LineWidth',2)
hold on

M6 = A4*A5*A6;
x6 = M6(1,3);
y6 = M6(2,3);
scatter(x6,y6,'filled','k')
plot([x5,x6],[y5,y6],'k','LineWidth',2)
hold on


%%%%% PLOT SETTING %%%%%
grid on
xlim([-0.5,0.4]);
ylim([-0.1,0.9]);
