%%Beginning code-controls IK, FK and polygon creation
%%Saif Sayed

clc;
clear all;
close all;
x_input=[-0.1471 -0.1029 -0.1029 -0.1471];
y_input=[0.05882 0.05882 0.1765 0.1765 ];
%x_input = x_input/40;
%y_input = y_input/40;

% x_input = [0.259300583138924;0.257209320897774;0.253747108477585;0.248948539160404;0.242861558664652;0.235546986087355;0.227077906219828;0.217538939308603;0.207025395557893];
% y_input = [0.0139766783305559;0.0278137063113086;0.0413728289325876;0.0545185679232111;0.0671195754045884;0.0790499462753050;0.0901904762132768;0.100429852725933;0.109665767347848];

figure;
plot(x_input,y_input);  % to plot polygon
fill(x_input,y_input,'g');  % to fill the polygon

len = length(x_input);
i = 1;
x_out = zeros(1,100);
y_out = zeros(1,100);
count = 1;
while (i < len+1)
    x1 = x_input(i);
    y1 = y_input(i);
    if i == len
        x2 = x_input(1);
        y2 = y_input(1);
    else
    x2 = x_input(i+1);
    y2 = y_input(i+1);
    end  
    a = y1-y2;
    b = x2-x1;
    c = (x1-x2)*y1 + (y2-y1)*x1;
    
    if abs(x2-x1)>abs(y2-y1)
        if x1>x2
            iter_mult = -1;
        else
            iter_mult = 1;
        end
        for x = x1:iter_mult*0.01:x2
            count = count + 1;
            x_out(count) = x;
            y_out(count) = (-c - (a*x))/b;
            [q11, q21, q12, q22] = IKtwoDOF(x_out(count),y_out(count),0);
            [x_3, y_3, z_3] = FKtwoDOF(q12,q22);
        end
    else
        if y1>y2
            iter_mult = -1;
        else
            iter_mult = 1;
        end
        for y = y1:iter_mult*0.01:y2
            count = count + 1;
            y_out(count) = y;
            x_out(count) = (-c - (b*y))/a;
            [q11, q21, q12, q22] = IKtwoDOF(x_out(count),y_out(count),0);
            [x_3, y_3, z_3] = FKtwoDOF(q12,q22);
%             [x_31, y_31, z_31] = FKtwoDOF(q11,q21);
%             theta = IKtwoDOF_s(x_out(count),y_out(count),0);
%             [x_3, y_3, z_3] = FKtwoDOF(theta(1),theta(2));
        end
    end
    
    i = i+1;
end

% figure;
% plot(x_input,y_input);  % to plot polygon
% fill(x_input,y_input,'g');  % to fill the polygon
hold on;
plot(x_out,y_out,'r*');  % to plot polygon
legend('x','y')';