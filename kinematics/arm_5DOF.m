%% Program: arm.m

clc;
clear all;
close all;

syms q1 q2 q3 q4 q5 x y;

% robot link lengths
d3 = 0.17914; % upperarm in meters
d5 = 0.18159; % forearm in meters

%% Transformation from shoulder to wrist

% LSP to LSR (0 to 1, T_01)
A1 = [cos(q1), -sin(q1), 0, 0;
      sin(q1),  cos(q1), 0, 0;
            0,        0, 1, 0;
            0,        0, 0, 1];

A2 = [1, 0,  0, 0;
      0, 0, -1, 0;
      0, 1,  0, 0;
      0, 0,  0, 1];

T_01 = A1*A2;
disp(T_01);

% LSR to LSY (1 to 2, T_12)
A1 = [-sin(q2), -cos(q2),  0, 0;
       cos(q2), -sin(q2),  0, 0;
        0,       0,        1, 0;
        0,       0,        0, 1];

A2 = [1, 0,  0, 0;
      0, 0, -1, 0;
      0, 1,  0, 0;
      0, 0,  0, 1];

T_12 = A1*A2;
disp(T_12);

% LSY to LEB (2 to 3, T_23)
A1 = [-sin(q3), -cos(q3),  0, 0;
       cos(q3), -sin(q3),  0, 0;
             0,        0,  1, 0;
             0,        0,  0, 1];

A2 = [1, 0, 0, 0;
      0, 1, 0, 0;
      0, 0, 1, d3;
      0, 0, 0, 1];

A3 = [1, 0,  0, 0;
      0, 0, -1, 0;
      0, 1,  0, 0;
      0, 0,  0, 1];
  
T_23 = A1*A2*A3;
disp(T_23);  

% LEB to LWY (3 to 4, T_34)
A1 = [-cos(q4),  sin(q4),  0, 0;
       sin(q4), -cos(q4),  0, 0;
        0,       0,        1, 0;
        0,       0,        0, 1];

A2 = [1, 0,  0, 0;
      0, 0, -1, 0;
      0, 1,  0, 0;
      0, 0,  0, 1];
  
T_34 = A1*A2;
disp(T_34);

% LWY to LWP (4 to 5, T_45)
A1 = [cos(q5), -sin(q5), 0, 0;
      sin(q5),  cos(q5), 0, 0;
            0,        0, 1, 0;
            0,        0, 0, 1];

A2 = [1, 0, 0,  0;
      0, 1, 0,  0;
      0, 0, 1, d5;
      0, 0, 0,  1];

A3 = [1, 0,  0, 0;
      0, 0, -1, 0;
      0, 1,  0, 0;
      0, 0,  0, 1];

T_45 = A1*A2*A3;  
disp(T_45);

% shoulder to wrist transformation (0 to 2, T_02)
T_05 = simplify(T_01 * T_12 * T_23 * T_34 * T_45);

disp('Transformation Matrix from Shoulder to Wrist:');
disp(T_05);

% %% Given position vector position_ref and orientation 
% %  matrix orientation_ref determine q1, q2
% 
% syms px py pz;
% 
% % desired (reference) position
% px = 0.25; 
% py = 0.1;
% pz = 0;
% 
% position_ref = sprintf('px: %f\npy: %f\npz: %f\n', px, py, pz);
% disp(position_ref);
% 
% % desired (reference) orientation
% orientation_ref = [1, 0, 0;
%                    0, 1, 0;
%                    0, 0, 1];
% 
% % desired (reference) position and orientation
% ref = [1, 0, 0, px;
%        0, 1, 0, py;
%        0, 0, 1, pz;
%        0, 0, 0, 1]; 
% 
% disp('Reference Orientation & Position of End-Effector:');
% disp(ref);
% 
% % solve for cos(q2), which here equals x
% % px = l2*cos(q1+q2) + l1*cos(q1)
% % py = l2*sin(q1+q2) + l1*sin(q1)
% % px^2 + py^2 = [l2*cos(q1+q2) + l1*cos(q1)]^2 + 
% %               [l2*sin(q1+q2) + l1*sin(q1)]^2 
% 
% % This reduces to:
% % px^2 + py^2 = l2^2*(cos(q1+q2)^2 + sin(q1+q2)^2) +
% %               l1^2*(cos(q1)^2 + sin(q1)^2) +
% %               2*l1*l2*(cos(q1)*cos(q1+q2) + sin(q1)*sin(q1+q2))
% 
% % This reduces to:
% % px^2 + py^2 = l1^2 + 
% %               l2^2 + 
% %               2*l1*l2*(c1(c1c2 - s1s2) + s1(c1s2 + s1c2))
% 
% % This reduces to:
% % px^2 + py^2 = l1^2 + l2^2 + 2*l1*l2*cos(q2)
% 
% % so, cos(q2) = [px^2 + py^2 - l1^2 - l2^2] / (2*l1*l2)
% 
% cos2 = ((ref(1,4))^2 + (ref(2,4))^2 - l1^2 - l2^2) / (2*l1*l2);
% if (cos2 >= -1 && cos2 <= 1)
%     x = cos2;
% else
%     error('Target not in workspace');
% end
% % solve for sin(q2), which here equals y, using cos^2 + sin^2 = 1
% y = sqrt(1-x^2);
% 
% % solve for q2 using arctan2(sin(q2), cos(q2))
% q2 = atan2(y, x);
% 
% % From john craigs intro to robotics page 111
% % solve for q1 using q1 = 90 - q2
% q1 = atan2(py, px) - atan2(l2*sin(q2), l1+l2*cos(q2));
% 
% % display joint values
% joints = sprintf('q1: %f rad (%f deg)\nq2: %f rad (%f deg)\n',...
%                 q1, q1*180/pi, q2, q2*180/pi);
% disp(joints);
% 
% figure;
% plot([0,l1], [0,0], '--gs', 'MarkerFaceColor','g');
% hold on;
% plot([l1,l1+l2], [0,0], '--gs', 'MarkerFaceColor','g');
% hold on;
% plot([0, l1*cos(q1)], [0, l1*sin(q1)], '--rd', 'MarkerFaceColor','r');
% hold on;
% plot([l1*cos(q1), (l1*cos(q1) + l2*cos(q1+q2))], [l1*sin(q1), ...
%     (l1*sin(q1) + l2*sin(q1+q2))], '--rd', 'MarkerFaceColor','r');
% hold on;
% plot(px, py, 'bp', 'MarkerSize',10);
% hold on;
% axis([-.2 .5 -.4 .4]);
% title('Start & End Positions for Hubos Arm');
% xlabel('x-distance (m)');
% ylabel('y-distance (m)');
% text(px+.01,py,'\leftarrow Target','HorizontalAlignment','left');
% text(-.01,0,'Shoulder','HorizontalAlignment','right');
