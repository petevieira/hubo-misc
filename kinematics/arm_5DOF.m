%% Program: arm.m

clc;
clear all;
close all;

syms q1 q2 q3 q4 q5 pxp pyp pzp;

% robot link lengths
d3 = 0.17914; % upperarm in meters
d5 = 0.18159; % forearm in meters

%% Transformation from shoulder to wrist (Paper)

%%| joint | theta |  d  |  a  | alpha |
%%|  0-1  |-90+q1 |  0  |  0  |  90   |
%%|  1-2  | 90+q2 |  0  |  0  | -90   |
%%|  2-3  | 90+q3 | -d3 |  0  |  90   |
%%|  3-4  |  q4   |  0  |  0  |  90   |
%%|  4-5  |  q5   |  d5 |  0  |  90   |
 
%LSP to LSR (0 to 1, T_01) theta=-90+q1, alpha=90
A1 = [sin(q1), cos(q1), 0, 0;  % cos(-90+q1), -sin(-90+q1)
     -cos(q1), sin(q1), 0, 0;  % sin(-90+q1),  cos(-90+q1)
            0,        0, 1, 0;
            0,        0, 0, 1];

A2 = [1, 0,  0, 0;
      0, 0, -1, 0;
      0, 1,  0, 0;
      0, 0,  0, 1];

T_01 = A1*A2;
T_10 = A2*A1;
%disp(T_01);

% LSR to LSY (1 to 2, T_12) theta=90+q2, alpha=-90
A1 = [sin(q2), -cos(q2),  0, 0;
       cos(q2), sin(q2),  0, 0;
        0,       0,        1, 0;
        0,       0,        0, 1];

A2 = [1, 0,  0, 0;
      0, 0, 1, 0;
      0, -1,  0, 0;
      0, 0,  0, 1];

T_12 = A1*A2;
T_21 = A2*A1;
%disp(T_12);

% LSY to LEB (2 to 3, T_23) theta=90+q3, alpha=90, d=-d3
A1 = [sin(q3), -cos(q3),  0, 0;
       cos(q3), sin(q3),  0, 0;
             0,        0,  1, 0;
             0,        0,  0, 1];

A2 = [1, 0, 0, 0;
      0, 1, 0, 0;
      0, 0, 1, -d3;
      0, 0, 0, 1];

A3 = [1, 0,  0, 0;
      0, 0, -1, 0;
      0, 1,  0, 0;
      0, 0,  0, 1];
  
T_23 = A1*A2*A3;
T_32 = A3*A2*A1;
%disp(T_23);  

% LEB to LWY (3 to 4, T_34) theta=q4, alpha=90
A1 = [cos(q4),  -sin(q4),  0, 0;
       sin(q4), cos(q4),  0, 0;
        0,       0,        1, 0;
        0,       0,        0, 1];

A2 = [1, 0,  0, 0;
      0, 0, -1, 0;
      0, 1,  0, 0;
      0, 0,  0, 1];
  
T_34 = A1*A2;
T_43 = A2*A1;
%disp(T_34);

% LWY to LWP (4 to 5, T_45) theta=q5, alpha=90, d5=d5
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
T_54 = A3*A2*A1;
%disp(T_45);

% shoulder to wrist transformation (0 to 2, T_02)
T_05 = simplify(T_01 * T_12 * T_23 * T_34 * T_45);
T_50 = simplify(T_54 * T_43 * T_32 * T_21 * T_10);

disp('Transformation Matrix from Shoulder to Wrist (Paper):');
disp(T_05);
disp('');
disp(T_50);
disp('');

posEqs = [pxp == T_50(1,4),...
          pyp == T_50(2,4),...
          pzp == T_50(3,4)];

S = solve(posEqs);
disp(S);
disp(S.q4);
disp(S.q5);


%v = [q1, q2, q3, q4, q5];
%R = jacobian(T_05, v);
%R_inv = inv(R);

%disp('Jacobian:')
%disp(R);

%q1 = 0;
%q2 = 0;
%q3 = 0;
%q4 = 0; 
%q5 = 0;
 
% disp('Evaluate T_05 given joint angles');
% disp(eval(T_05));

%elbow = eval(T_01*T_12*T_23);
%hand = eval(T_05);

%x0 = [0, 0, 0];
%y0 = [0, 0, 0];
%z0 = [0, d3, d3+d5];

%x1 = [0, elbow(1,4), hand(1,4)];
%y1 = [0, elbow(2,4), hand(2,4)];
%z1 = [0, elbow(3,4), hand(3,4)];


%figure;
%plot3(x0, y0, z0, '--gs', 'MarkerFaceColor', 'g');
%hold on;
%plot3(x1, y1, z1, '--rd', 'MarkerFaceColor', 'r');
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
