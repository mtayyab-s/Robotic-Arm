%%Saif Sayed
%%Forward Kinematics

function [x_3, y_3, z_3] = FKtwoDOF(f1,f2)
%Get all the angles for joint 1 to joint 5

%Convert to Rad

 f1_r=f1*pi/180;
 f2_r=f2*pi/180;

%%Link Lenghts in cm

l1 = 0.0;
l2 = 0.1125;
l3 = 0.14;

%% Trigonometric abbreviations

c1 = cos(f1_r);
c12 = cos(f1_r+f2_r);

s1 = sin(f1_r);
s12 = sin(f1_r+f2_r);

%%Position of Base
x_0=0;
y_0=0;
z_0=0;

%%Position of Joint 1
x_1=0;
y_1=0;
z_1=l1;

%%Position of Joint 2
x_2 = l2*c1;
y_2 = l2*s1;
z_2 = z_1;

%%Position of Joint 3
x_3 = x_2 + (l3 * c12);
y_3 = y_2 + (l3 * s12);
z_3= z_2;


%%Matrix Containing the position of each Joint
xx=[x_0;x_1;x_2;x_3];
yy=[y_0;y_1;y_2;y_3];
zz=[z_0;z_1;z_2;z_3];

p0e = [ x_3 y_3 z_3 ]' ;
disp('FK position =');
disp(p0e);

%Plot the robot according to Forward Kinematics
view([90 90]);
figure (3)
pause(0.3);
plot3(xx,yy,zz,'ko-','Linewidth',2)
% plot3(x_3,y_3,z_3,'r*')
hold on
zlabel('z (cm)')
ylabel('y (cm)');
xlabel('x (cm)');
% axis ([-0.20 0.40 -0.80 0.20 0 0.80])
grid on

end