function animate(s,t)
% place the arm in a new location based on current state

global ARMGRAPH % this is the graphics handle that points to the plot of the manipulator 

[tip,elbow]=forwardKinematics(s(1:2)');
set(ARMGRAPH,'xdata',[0 elbow(1) tip(1)], 'ydata',[0 elbow(2) tip(2)])
drawnow