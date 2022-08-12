% MATLAB m-function 
% returns a matrix that generates the Denavit-Hartenberg
% forward kinematics rotation matrix.
%
% syntax:   A=DHA(a,d,theta,alpha);
%  where:       a=length
%          d=offset
%           theta=angle
%           alpha=twist

% Written by Dong Ho Kang
  
function A = DHA(a,d,theta,alpha)


A = [[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)]
     [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)]
     [0,            sin(alpha),                 cos(alpha),         d        ]
     [0,                0,                           0,             1        ]];


end