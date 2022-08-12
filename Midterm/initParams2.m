% matlab script
% Init  global variables, paramters, and COMPOSITE VARIABLES
%  MKS units  
 
global qd Kp Kd TAUd;
global L M R I xd td;
global M2L1L2_2 I2M2L2 I22 I1I2ML M2L1L2; 

outputTime=0:.05:2;    % timescheme for all outputs of simulation

L  = [0.3; 0.3];                          % link lengths
R = .75 * L;                               % COM (75% of length)    
M = [0.8; 0.8];                            % link masses
I = [0.2; 0.2];                            % link inertias

% precaluated items for use later
M2L1L2_2=M(2)*L(1)*L(2)/2;        
I2M2L2=I(2)+M(2)*L(2)^2/4; 
I1I2ML = I(1) + I(2) + (M(1)*L(1)^2 + ...
  M(2)*L(2)^2)/4 + M(2)*L(1)^2; 
M2L1L2 = M(2)*L(1)*L(2); 
I22 = I2M2L2; 

% Set the desired movement time
MT=0.77;  

xd = xd_Smooth;
td=((0:size(xd,1)-1)/(size(xd,1)-1))*MT';
% initial configuration:
[q0]=inverseKinematics(xd(1,:));  % inverse kinematics -- init at desired
initialStates=[q0(1),q0(2),0,0]'; % initial states with zero velocity
