function tau = q2tau(L, m, q, qdot, qddot)
% Inverse dynamics.
% tau = q2tau(L, m, q, qdot, qddot)
% L (2 x 1): lengths of links;
% m (2 x 1): masses of links;
% q (2 x N): relative joint angles;
% qdot (2 x N): joint angular velocities;
% qddot (2 x N): joint angular accelerations;
% tau (2 x N): joint torques;

g = 0;
global I R; %Inertia and COM 

I1 = I(1); I2 = I(2); 
Lc1 = R(1); Lc2 = R(2);
m1 = m(1); m2 = m(2);
L1 = L(1); L2 = L(2);


loop = size(q);

for i=1:loop(1)
    
    
q1 = q(i,1); q2 = q(i,2);
q1dot = qdot(i,1); q2dot = qdot(i,2);
q1ddot = qddot(i,1); q2ddot = qddot(i,2);
    
d11 = m1*Lc1^2+m2*(L1^2+Lc2^2+2*L1*Lc2^2+2*L1*Lc2*cos(q2))+I1+I2;
d12 = m2*(Lc2^2+L1*Lc2*cos(q2))+I2;
d21 = d12;
d22 = m2*Lc2^2+I2;

% c111 = 0;
c121 = -m2*L1*Lc2*sin(q2);
h = -m2*L1*Lc2*sin(q2);
c221 = c121;
c211 = h;
c112 = -h;
% c122 = 0;
% c212 = 0;
% c222 = 0;
phi1 = (m1*Lc1+m2*L1)*g*cos(q1)+m2*Lc2*g*cos(q1+q2);
phi2 = m2*Lc2*cos(q1+q2);

tau(i,1) = d11*q1ddot+d12*q2ddot+c121*q1dot*q2dot+c211*q2dot*q1dot + c221*q2dot^2+phi1;
tau(i,2) = d21*q1ddot+d22*q2ddot+c112*q1dot^2+phi2;


end

end