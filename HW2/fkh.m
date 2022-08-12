function T = fkh(L, q)
% T = fkh(L, q)
% Forward kinematics using homogeneous transformations.
% L (1 x n): lengths of links, n is the number of links;
% q (1 x n): relative joint angles;
% T (3 x 3): homogeneous representation of the end effector
%   with respect to the base frame.

T = zeros(3); % reset
n = length(q);



for i = 1:n
    T = fkhLink(L(i), q(i));

    
end

end