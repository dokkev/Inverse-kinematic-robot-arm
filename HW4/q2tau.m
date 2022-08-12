    %% Inverse dynamics.
% tau = q2tau(L, m, q, qdot, qddot)
% L (2 x 1): lengths of links;
% m (2 x 1): masses of links;
% q (2 x 1): relative joint angles;
% qdot (2 x 1): joint angular velocities;
% qddot (2 x 1): joint angular accelerations;
% tau (2 x 1): joint torques.

% Written by Dong Ho Kang

function tau = q2tau(L,m,q,qdot,qddot)

% Parmeters Definition
m1 = m(1); m2 = m(2);
L1 = L(1); L2 = L(2);
q1 = q(1); q2 = q(2);
g = 9.8;

Lc = L1/2;  Lc1 = L1/2; Lc2 = L2/2;
I1 = (m1*L1^2)/12;  I2 = (m2*L2^2)/12;


% From Spong, M.W., Hutchinson, S. and Vidyasagar. “Robot Motion and Control”. Wiley, 2006.
% Planar Elbow Manipulation
d11 = m1*Lc1^2+m2*(L1^2+Lc2^2+2*L1*Lc2^2+2*L1*Lc2*cos(q2))+I1+I2;
d12 = m2*(Lc2^2+L1*Lc2*cos(q2))+I2;
d21 = d12;
d22 = m2*Lc2^2+I2;

c111 = 0;
c121 = -m2*L1*Lc2*sin(q2);
h = -m2*L1*Lc2*sin(q2);
c221 = c121;
c211 = h;
c112 = -h;
c122 = 0;
c212 = 0;
c222 = 0;
phi1 = (m1*Lc1+m2*L1)*g*cos(q1)+m2*Lc2*g*cos(q1+q2);
phi2 = m2*Lc2*cos(q1+q2);

tau1 = d11*qddot(1)+d12*qddot(2)+c121*qdot(1)*qdot(2)+c211*qdot(2)*qdot(1) + c221*qdot(2)^2+phi1;
tau2 = d21*qddot(1)+d22*qddot(2)+c112*qdot(1)^2+phi2;

tau = [tau1, tau2];


end