% armframe script
% Plot the workspace

global ARMGRAPH


% figure window setup
figure(1); clf; 
axis equal
hold on;

% plot axes 
plot([-L(1) - L(2), L(1) + L(2)]', [0, 0]','b:');
plot([0, 0]', [-.1, L(1) + L(2)]','b:');

% plot workspace boundary
t=[0:pi/100:pi]';
x=(L(1)+L(2))*cos(t);
y=(L(1)+L(2))*sin(t);
plot(x,y,'b-');

% plot desired trajectory points 
plot(xd(:,1),xd(:,2),'g^')

% plot initial configuration
[tip,elbow]=forwardKinematics(q0); % initial positions from initial angles
ARMGRAPH=plot([0 elbow(1) tip(1)],[0 elbow(2) tip(2)], 'r', 'linewidth',3); % sets object handle 
drawnow
