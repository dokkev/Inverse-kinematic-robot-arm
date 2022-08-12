clc,clear
% Define t as a variable
syms t;
% Define delta time
dt = 0.1;

% Configuration of the 2 DOF Robot Arm
Xi = [-0.1,0.3];
Xf = [0.1,0.3];

% End-Effector inital and final positions
x0 = Xi;
xf = Xf;

% x(t) according to min-jerk
T = 5;
xt = x0 + (x0-xf)*(15 * (t/T)^4 - 6*(t/T)^5 - 10*(t/T)^3);

% Inverse Kinematics q(t) = theta(t)
qt = inverseKinematics(xt);
% Estimate q(t)dot and q(t)ddot
tp= t + dt;
tm = t - dt;
qtdot = (subs(qt,t,tp) - subs(qt,t,tm))/(2*dt);
qtddot = (subs(qtdot,t,tp) - subs(qtdot,t,tm))/(2*dt);
% Parameters for the function
L = [0.3, 0.32];
m = [1.2, 0.9];


%% Torque Plot
nexttile
for time = 0:dt:5
    tau = q2tau(L,m,qt,qtdot,qtddot);
    tau1 = subs(tau(1),t,time); tau2 = subs(tau(2),t,time);
    scatter(time,tau1,'filled','red')
    hold on
    scatter(time,tau2,'filled','blue')
end 
grid on
title("Torque on Joints over time")
legend("Tau1","Tau2")
xlabel("time (seconds)")
ylabel("Torque (N*m)")
%% x(t) plot
nexttile
 for time = 0:dt:5
    x_pos = subs(xt(1),t,time); y_pos = subs(xt(2),t,time);
    scatter(time,x_pos,'filled','red')
    hold on
    scatter(time,y_pos,'filled','blue')
end
grid on
title("x(t) Over time")
legend("x-position","y-position")
xlabel("time (seconds)")
ylabel("displacement(m)")
ylim([-0.15,0.5])

%% q(t) plot
nexttile
 for time = 0:dt:5
    q1_pos = subs(qt(1),t,time); q2_pos = subs(qt(2),t,time);
    scatter(time,q1_pos,'filled','red')
    hold on
    scatter(time,q2_pos,'filled','blue')
end
grid on
title("q(t) over time")
legend("q1","q2")
xlabel("time (seconds)")
ylabel("Angle (rad)")

%% q(t)dot plot
nexttile
 for time = 0:dt:5
    qd1_pos = subs(qtdot(1),t,time); qd2_pos = subs(qtdot(2),t,time);
    scatter(time,qd1_pos,'filled','red')
    hold on
    scatter(time,qd2_pos,'filled','blue')
end
grid on
title("q(t)dot over time")
legend("qdot1","qdot2")
xlabel("time (seconds)")
ylabel("Speed (rad/s)")

%% q(t)ddot plot
nexttile
 for time = 0:dt:5
    qdd1_pos = subs(qtddot(1),t,time); qdd2_pos = subs(qtddot(2),t,time);
    scatter(time,qdd1_pos,'filled','red')
    hold on
    scatter(time,qdd2_pos,'filled','blue')
end
grid on
title("q(t)ddot over time")
legend("qddot1","qddot2")
xlabel("time (seconds)")
ylabel("Accerelation (rad/s^2)")
