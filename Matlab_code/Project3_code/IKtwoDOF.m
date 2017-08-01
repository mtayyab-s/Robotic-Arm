%%Saif Sayed
%%Inverse Kinematics
function [q1_1, q2_1, q1_2, q2_2] = IKtwoDOF(px, py, pz)

%% Links Length
l1 = 0.1125 ;
l2 = 0.14 ;

p0e = [ px py pz ]' ;
disp('Desired position =')
disp(p0e)
if norm(p0e) > l1+l2
    error('desired position is out of the workspace')
end


%% %%%%%%%%%%%% Inverse Kinematics of a 2DOF manipulator %%%%%%%%%%%%%%%%%

%% Find q2
c2 = (px^2+py^2-l1^2-l2^2)/(2*l1*l2) ;
s2_1 = sqrt(1-c2^2) ;
s2_2 = - sqrt(1-c2^2) ;
% We have two solutions for q2
q2_1 = atan2(s2_1,c2) ;
q2_2 = atan2(s2_2,c2) ;


%% Find q1

% We define the constants
k1 = l1+l2*c2 ;
k21 = l2*s2_1 ;
k22 = l2*s2_2 ;
gama1 = atan2(k21,k1) ;
gama2 = atan2(k22,k1) ;
r = sqrt(k1^2+k21^2) ; % r is the same for both values of q2

% We find q1 depending on the value of q2
q1_1 = atan2(py,px) - gama1 ;
q1_2 = atan2(py,px) - gama2 ;

x = l1*cos(q1_1) + l2*cos(q1_1+q2_1);
y = l1*sin(q1_1) + l2*sin(q1_1+q2_1);

q1_1 = q1_1 * 180/pi;
q1_2 = q1_2 * 180/pi;
q2_1 = q2_1 * 180/pi;
q2_2 = q2_2 * 180/pi;

disp('1 solution')
disp([ q1_1 q2_1 ]*180/pi)


disp('1 solution')
disp([ q1_2 q2_2 ]*180/pi)
end