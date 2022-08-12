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
disp(qt)
disp(qtdot)
disp(qtddot)
% Parameters for the function
L = [0.3, 0.32];
m = [1.2, 0.9];

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
