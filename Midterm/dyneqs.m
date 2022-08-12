function sdot = dyneqs(t, s) 
%  sdot = dyneqs(t, s) 
%  state equations for the RK simulator. Calculates for current time t.
%        NOTE THAT t SHOULD BE A SINGLE NUMBER ( NOT AN ARRAY)
%	thus, we can use u(i) and not u(i,:)
%   state s = [q1, q2, q1p, q2p]' 

global L;
global M2L1L2_2 I2M2L2 I22 I1I2ML M2L1L2; 

L1=L(1); L2=L(2);

Pert = 2.0;

% Controller torque 
% The function controlfun(s,t) must be user-supplied. It calculates the 
% control toruqe (shoulder and elbow) given the current state and time
[Cons,Cone] = controlfun(s,t);
animate(s,t);

% Inertia matrix 
c2 = cos(s(2)); 
in(1,1) = I1I2ML + M2L1L2*c2; 
in(1,2) = I2M2L2 + M2L1L2_2*c2; 
in(2,1) = in(1,2); 
in(2,2) = I22;
invi = inv(in); 

% Coriolis + Centripetal
inter= M2L1L2_2 * sin(s(2)); 
g(1) =  - inter * (s(4)^2 + 2 * s(3)*s(4)); 
g(2) =  inter * (s(3)^2);

if (t>0.8) && (t<0.9)
    g(2) = g(2)+Pert;
end

% The state equations 
sdot(1,1) = s(3); 
sdot(2,1) = s(4); 
sdot(3,1) = invi(1,1)*(Cons - g(1)) + invi(1,2)*(Cone - g(2)); 
sdot(4,1) = invi(2,1)*(Cons - g(1)) + invi(2,2)*(Cone - g(2)); 

