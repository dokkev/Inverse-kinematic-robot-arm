% MATLAB m-function % returns a 
% syntax:   theta=invKin(p,a,d,alpha);
%  where:       p=endpoint position
%               a=lengths
%               d=offsets
%               alpha=twists
%               theta=angle

function theta = invkin (p,a,d,alpha)
a1=a(1);          
a2=a(2);         
a3=a(3);        
alpha1=alpha(1);    
alpha2=alpha(2);       
alpha3=alpha(3);        
pw_x=p(1);   
pw_y=p(2);
pw_z=p(3);
c3=(pw_x^2+pw_y^2+pw_z^2-a2^2-a3^2)/(2*a2*a3); 
s3=-sqrt((1-c3^2));     
s3 = abs(s3);
theta3=atan2(s3,c3);
c2=(sqrt(pw_x^2+pw_y^2)*(a2+a3*c3)+pw_z*a3*s3)/(a2^2+a3^2+2*a2*a3*c3);      
s2=(pw_z*(a2+a3*c3)-sqrt(pw_x^2+pw_y^2)*a3*s3)/(a2^2+a3^2+2*a2*a3*c3);     
theta2=atan2((a2+a3*c3)*pw_z-a3*s3*sqrt(pw_x^2+pw_y^2),(a2+a3*c3)*sqrt(pw_x^2+pw_y^2)+a3*s3*pw_z);
theta1=atan2(pw_y,pw_x);
theta=[theta1 theta2 theta3]';   
end