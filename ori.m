clc;
clear all;
a =  0.5236; % yaw
b =  0; % pitch
c =  -0.5236; % roll
% a =  0; % yaw
% b =  0; % pitch
% c =  0; % roll

Rx = [1,      0,       0;
      0, cos(c), -sin(c);
      0, sin(c),  cos(c)];

Ry = [cos(b), 0, sin(b);
           0, 1,     0;
     -sin(b), 0, cos(b)];

Rz = [cos(a),-sin(a), 0;
      sin(a), cos(a), 0;
           0,      0, 1];

R = Rx*Ry*Rz;



Rbox_ee1 = [1, 0, 0;
            0, 0, 1
            0,-1, 0];

Rbox_ee2 = [1,0,0;
        0,0, -1
        0,1,0];



Ree1 = R*Rbox_ee1;
Ree2 = Rbox_ee1*R;
angles1=rotm2eul(Ree1,"XYZ") - [-1.5708,0,0];
angles2=rotm2eul(Ree2,"XYZ");
disp("Orientazione panda 1")
disp(angles1);
disp("Orientazione panda 2")
disp(angles2);



% tp = theaterPlot('XLimit',[-2 2],'YLimit',[-2 2],'ZLimit',[-2 2]);
% op = orientationPlotter(tp,'DisplayName','base','LocalAxesLength',2);
% plotOrientation(op,R); 
% 
% 
% tp = theaterPlot('XLimit',[-2 2],'YLimit',[-2 2],'ZLimit',[-2 2]);
% op = orientationPlotter(tp,'DisplayName','ee1',...
%     'LocalAxesLength',2);
% plotOrientation(op, Ree1); 




