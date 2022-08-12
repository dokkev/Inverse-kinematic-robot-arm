% Matlab Script for dynamics simulations
% 
% Initialize global and other variables:
%initParams

% prepare graphics
setGraphics

% Simulate the forward dynamics. The forward dynamics uses the  
% function dyneqs.m which calls controlfun.m as a controller. 
fprintf('\nSimulation....'); pause(.1);
[t,states]=ode23('dyneqs', outputTime, initialStates); 
fprintf('Done.\n'); pause(.01);

[tip,elbow]=forwardKinematics(states);
plot(tip(:,1),tip(:,2),'.','MarkerSize',10);         % Plot

