%% Program: arm_5DOF.m

clc;
clear all;
close all;

syms q1 q2 q3 q4 q5 q6... %joint angles
     lA1 lA2 lA3 lA4...             %neck_to_shoulder_offset, wrist_to_hand_offset
     nxp nyp nzp sxp syp szp axp ayp azp pxp pyp pzp;   %inverse coordinates

% robot link lengths
%lA2 = 0.17914; % upperarm in meters
%lA3 = 0.18159; % forearm in meters

%% Transformation from Neck to Hand (Paper)

%%| joint | theta |  d  |  a  | alpha |
%%|neck-0 |  90   | lA1 |  0  |  90   |
%%|  0-1  |-90+q1 |  0  |  0  |  90   |
%%|  1-2  | 90+q2 |  0  |  0  | -90   |
%%|  2-3  | 90+q3 |-lA2 |  0  |  90   |
%%|  3-4  |  q4   |  0  |  0  |  90   |
%%|  4-5  |  q5   | lA3 |  0  |  90   |
%%|  5-6  | 90+q6 |  0  | lA4 |   0   |

%Neck to Shoulder (Neck to 0, T_NO) theta=90, alpha=90, d=lA1
A1 = [0, -1, 0, 0;
      1,  0, 0, 0;
      0,  0, 1, 0;
      0,  0, 0, 1];
  
A2 = [1, 0,  0, 0; 
      0, 0, -1, 0;
      0, 1,  0, 0;
      0, 0,  0, 1];        
  
A3 = [1, 0, 0, 0;
      0, 1, 0, 0;
      0, 0, 1, lA1;
      0, 0, 0, 1];

T_N0 = A1*A2*A3;
T_0N = inv(T_N0);
%disp(T_N0);
%disp(T_0N);

%LSP to LSR (0 to 1, T_01) theta=-90+q1, alpha=90
A1 = [sin(q1), cos(q1), 0, 0;  % cos(-90+q1), -sin(-90+q1)
     -cos(q1), sin(q1), 0, 0;  % sin(-90+q1),  cos(-90+q1)
            0,       0, 1, 0;
            0,       0, 0, 1];
        
A2 = [1, 0,  0, 0;
      0, 0, -1, 0;
      0, 1,  0, 0;
      0, 0,  0, 1];

T_01 = A1*A2
T_10 = inv(T_01);
%disp(simplify(T_01));
%disp(simplify(inv(T_01)));
%disp(simplify(T_10));

% % LSR to LSY (1 to 2, T_12) theta=90+q2, alpha=-90
A1 = [-sin(q2), -cos(q2),  0, 0; % cos(90+q1), -sin(90+q1)
      cos(q2),  -sin(q2),  0, 0; % sin(90+q1),  cos(90+q1)
            0,        0,  1, 0;
            0,        0,  0, 1];

A2 = [1, 0, 0, 0;
      0, 0, 1, 0;
      0,-1, 0, 0;
      0, 0, 0, 1];

T_12 = A1*A2
T_21 = inv(T_12);
%disp(simplify(T_12));
%disp(simplify(inv(T_12)));
%disp(simplify(T_21));
 
% % LSY to LEB (2 to 3, T_23) theta=90+q3, alpha=90, d=-lA2
A1 = [-sin(q3), -cos(q3),  0, 0; % cos(90+q1), -sin(90+q1)
      cos(q3),  -sin(q3),  0, 0; % sin(90+q1),  cos(90+q1)
            0,        0,  1, 0;
            0,        0,  0, 1];

A2 = [1, 0, 0,   0;
      0, 1, 0,   0;
      0, 0, 1, -lA2;
      0, 0, 0,   1];

A3 = [1, 0,  0, 0;
      0, 0, -1, 0;
      0, 1,  0, 0;
      0, 0,  0, 1];
    
T_23 = A1*A2*A3
T_32 = inv(T_23);
%disp(T_23);
%disp(simplify(inv(T_23)));
%disp(simplify(T_32));

% LEB to LWY (3 to 4, T_34) theta=q4, alpha=90
A1 = [cos(q4), -sin(q4),  0, 0;
      sin(q4),  cos(q4),  0, 0;
            0,        0,  1, 0;
            0,        0,  0, 1];   

A2 = [1, 0,  0, 0;
      0, 0, -1, 0;
      0, 1,  0, 0;
      0, 0,  0, 1];
  
T_34 = A1*A2
T_43 = inv(T_34);
%disp(T_34);
%disp(simplify(inv(T_34)));
%disp(simplify(T_43));

% LWY to LWP (4 to 5, T_45) theta=q5, alpha=90, lA3=lA3
A1 = [cos(q5), -sin(q5), 0, 0;
      sin(q5),  cos(q5), 0, 0;
            0,        0, 1, 0;
            0,        0, 0, 1];

A2 = [1, 0, 0,  0;
      0, 1, 0,  0;
      0, 0, 1, lA3;
      0, 0, 0,  1];
  
A3 = [1, 0,  0, 0;
      0, 0, -1, 0;
      0, 1,  0, 0;
      0, 0,  0, 1];

T_45 = A1*A2*A3
T_54 = inv(T_45);
%disp(T_45);
%disp(simplify(inv(T_45)));
%disp(simplify(T_54));


%LWP to Hand (5 to H, T_5H) theta=90+q1, a=lA4
A1 = [-sin(q6), -cos(q6), 0, 0;  % cos(90+q1), -sin(90+q1)
      cos(q6),  -sin(q6), 0, 0;  % sin(90+q1),  cos(90+q1)
            0,        0, 1, 0;
            0,        0, 0, 1];

A2 = [1, 0, 0, lA4;
      0, 1, 0,  0;
      0, 0, 1,  0;
      0, 0, 0,  1];

T_56 = A1*A2
T_65 = inv(T_56);
%disp(T_56);
%disp(simplify(inv(T_56)));
%disp(simplify(T_65));

 disp('shoulder to wrist transformation (0 to 6, T_06)');
 disp('Calculating T_06');
 T_06 = simplify(T_01 * T_12 * T_23 * T_34 * T_45 * T_56);
% %disp(T_06);
% 
 disp('hand to shoulder transformation (6 to 0, T_60)');
 disp('Calculating T_60');
 T_60 = simplify(T_65 * T_54 * T_43 * T_32 * T_21 * T_10);
% %disp(T_60);
% 
 refInv = [nxp, sxp, axp, pxp;
           nyp, syp, ayp, pyp;
           nzp, szp, azp, pzp;
             0,   0,   0,   1];
% 
% disp('refInv');
% disp(refInv);
% 
 disp('Using the inverse transform method,');
 disp('Start with T_60 = refInv');
 disp('multiply both sides by T_56')
 disp('to obtain T_50 = T_56 * refInv');
% 
 disp('Computing Inverse Transform of T_50');
 G2R = simplify(T_56 * T_60)
 G2L = simplify(T_56 * refInv)

% Display G2(LHS) & G2(RHS)
G2L(1,4)
G2L(2,4)
G2L(3,4)

G2R(1,4)
G2R(2,4)
G2R(3,4)

disp('Get inverse transform of T_30');
G4R = simplify(T_34 * T_45 * G2R)
G4L = simplify(T_34 * T_45 * G2L)
% 
% pxp + lA4 = rcos(psi)
% pyp = rsin(psi)
% 

% % Set:
% pxp + lA4 = rcos(psi)
% pyp = rsin(psi)
% 
% % So these equations,
% -rsin(q6)cos(psi) - rsin(psi)cos(q6) = lA2sin(q4)cos(q5)
% rcos(q6)cos(psi) - rsin(psi)sin(q6) = -lA2cos(q4) - lA3
% pzp = lA2sin(q4)sin(q5)
% 
% % Become these:
% rsin(q6+psi) = -lA2sin(q4)cos(q5)
% rcos(q6+psi) = -lA2cos(q4) - lA3
% pzp = lA2sin(q4)sin(q5)
% 
% % where
% r = sqrt((pxp + lA4)^2 + (pyp)^2)
% psi = atan2(pyp, pxp + lA4)
% 
% % By squaring all three equations
% r^2 * (sin(q6+psi))^2 = lA2^2 * sin(q4)^2 * cos(q5)^2
% r^2 * cos(q6+psi))^2 = lA2^2 * cos(q4)^2 + 2*lA2*lA3*cos(q4) + lA3^2
% pzp^2 = lA2^2 * sin(q4)^2 * sin(q5)^2
% 
% % And then adding them
% r^2 + pzp^2 = lA2^2 + 2*lA2*lA3*cos(q4) + lA3^2
% 
% % which simplifies to
% C4 = (pxp + lA4)^2 + pyp^2 + pzp^2 - lA2^2 - lA3^2
%           ---------------------------------------------
%                        2*lA2*lA3
%                        
% q4 = atan2(+/- sqrt(1 - C4^2), C4)
% 
% S5 = pzp / lA2*sin(q4)
% 
% q5 = atan2(S5, +/- sqrt(1 - S5^2))
% 
% cos(q5)
% 
% tan(q6+psi) = sin(q6+psi) / cos(q6+psi) = lA2*sin(q4)*cos(q5)
%                                           -------------------
%                                           lA2*cos(q4) - lA3
%                                          
% q6 = atan2((lA2*sin(q4)*cos(q5)), (lA2*cos(q4) - lA3)) - psi

% now multiply both sides of G2 by (T_34 * T_45) to obtain G4

% compare element (2,3) of both sides of G4
%S2 = [-azp*sin(q4)*sin(q5) - axp*cos(q4)*cos(q6) + ayp*cos(q4)*sin(q6) - ayp*cos(q5)*sin(q4)*cos(q6) - axp*cos(q5)*sin(q4)*sin(q6)]
%S2 = axp(cos(q4)cos(q6) + cos(q5)sin(q4)sin(q6)) - ayp(cos(q4)sin(q6) - cos(q5)sin(q4)cos(q6) - azp(sin(q4)sin(q5)

%q2 = atan2(S2, +/-sqrt(1 - S2^2))

%By comparing the elements (1,3) and (3,3) of LHS and
%RHS of G4, we get two equations. By dividing these two
%equations, we can find the joint solution 3,
%cos(q2)*sin(q3) = -axp*(sin(q4)*cos(q6) - cos(q4)*cos(q5)*sin(q6)) + ayp*(sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6)) - azp*cos(q4)*sin(q5)
%cos(q2)*cos(q3) = -azp*cos(q5) - ayp*cos(q6)*sin(q5) - axp*sin(q5)*sin(q6)

%q3 = atan2(-axp*(sin(q4)*cos(q6) - cos(q4)*cos(q5)*sin(q6)) + ayp*(sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6)) - azp*cos(q4)*sin(q5) ,
%           -azp*cos(q5) - ayp*cos(q6)*sin(q5) - axp*sin(q5)*sin(q6))

%if S2 < 0, then q3 = q3 + pi

%By comparing the elements
%(2,1) and (2,2) of LHS and RHS of G4, we get two equations.
%By dividing these two equations, we find the joint angle q1

%cos(q2)*sin(q1) = -nzp*sin(q4)*sin(q5) + cos(q4)*(nxp*cos(q6) - nyp*sin(q6)) + cos(q5)*sin(q4)*(nyp*cos(q6) + nxp*sin(q6))
%cos(q1)*cos(q2) = szp*sin(q4)*sin(q5) - cos(q4)*(sxp*cos(q6) - syp*sin(q6)) - cos(q5)*sin(q4)*(syp*cos(q6) + sxp*sin(q6))
  
%q1 = atan2(-nzp*sin(q4)*sin(q5) + cos(q4)*(nxp*cos(q6) - nyp*sin(q6)) + cos(q5)*sin(q4)*(nyp*cos(q6) + nxp*sin(q6)) , 
%            szp*sin(q4)*sin(q5) - cos(q4)*(sxp*cos(q6) - syp*sin(q6)) - cos(q5)*sin(q4)*(syp*cos(q6) + sxp*sin(q6)))
  

% 
% posEqs = [pxp == -8957*cos(q4 + q5)*cos(q5)/50000 - (18159*cos(q6))/100000 - lA4,...
%           pyp == -8957*sin(q4 + q6)*cos(q5)/50000 - (18159*sin(q6))/10000,...
%           pzp == (8957*sin(q4)*sin(q5))/50000];
      
% T_60 = [n' s' a' p'
%         0  0  0  1]

% T_56 * T_60 = T_56 * [n' s' a' p'
%:q    =T_50              0  0  0  0]

%T_50a = (simplify(T_54 * T_43 * T_32 * T_21 * T_10));
%disp('T_50a');
%disp(T_50a);
%T_50 = simplify(T_56 * T_60);
%disp('T_50');
%disp(T_50);
%RHS = simplify(T_56 * refInv);
%disp('RHS');
%disp(RHS);

%disp('');
%disp('Transformation Matrix from Hand to Shoulder (Paper):');
%disp(T_06i);
%disp(T_60);
% disp('');
% 

%       
%disp(posEqs);

%S = solve(posEqs);
%disp(S);
%disp(S.q4);
%disp(S.q5);


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
%z0 = [0, lA2, lA2+lA3];

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
