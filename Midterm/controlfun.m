    function [Cons, Cone] = controlfun(s, t)
% [Cons, Cone] = controlfun(s, t)
% Control fuction template. Should be supplied (modified) by the user.
% s=STATE:   s(1)=q1; s(2)=q2; s(3)=q1_dot; s(4)=q2_dot;
% t=time
% [Cons, Cone] = output control torque.
% In this template, the control torque is a simple PD controller that 
% tries to move the arm on a desired path (in joint coordinates)

global TAUd qd td Kp Kd;             % desired x, time td (see initParams)

% __error calulation:__
qdi=interp1(td,qd,t);               % interpolate for the position desired at this time
e = s - qdi(1:4)';                  % CALCULATE STATE ERROR



% __PD Control output:__
P=Kp*e(1:2);                        % P-control 
D=Kd*e(3:4);                        % D-control 
%D=Kd*s(3:4);                       % Damping control alternative (doesn't work as well)
 tc=P+D;                           % PD control with Inverse dynamics control

 
 
 
% __Inverse Dynamics Control output (uncomment for inverse dynamics control)__
TAUdi=interp1(td,TAUd,t);           % interpolate for torque desired at this time
tc=tc+TAUdi(1:2)';                 % PD control with Inverse dynamics control

% __compile output variables:__
Cons=tc(1);
Cone=tc(2);
