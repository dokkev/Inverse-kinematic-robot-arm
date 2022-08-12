% Forward kinematics using homogeneous transformations  for a single link.
% Written by Dongho Kang

function A = fkhLink(Li, qi)
% A = fkhLink(Li, qi)
% Forward kinematics using homogeneous transformations
% for a single link.
% Li (1 x 1): length of the link;
% qi (1 x 1): relative joint angle.
% A(3 x 3): homogeneous representation of the link with
%   respect to the previous frame.


A = zeros(3); %reset

A(1,1) = cos(qi);   A(1,2) = -sin(qi);    A(1,3) = Li*(cos(qi));
A(2,1) = sin(qi);   A(2,2) = cos(qi);     A(2,3) = Li*(sin(qi));
A(3,1) = 0;         A(3,2) = 0;           A(3,3) = 1;

end