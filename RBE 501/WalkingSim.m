%Michael Chan
function [] = WalkingSim(fj1,fj2,fj3)
% theta = 90;
% phi = 90;

syms a b c
syms theta1 theta2 theta3
L1 = a;
L2 = b;
L3 = c;
% stand = c;


%joint 1(only translation in y)
% x and y translation

R1 = [cos(theta1),-sin(theta1),0,0;
     sin(theta1),cos(theta1),0,0;
     0,0,1,0;
     0,0,0,1];

L1y = a*sin(theta1);
L1x = a*cos(theta1);

T1y = [1,0,0,0;
    0,1,0,-L1y;
    0,0,1,0;
    0,0,0,1];

T1x = [1,0,0,-L1x;
    0,1,0,0;
    0,0,1,0;
    0,0,0,1];

T1 = mtimes(T1x,T1y);
H1 = mtimes(T1,R1);

pos1Vect = H1(1:3,end);

%joint 2
%rotation
 R2 = [cos(theta2),-sin(theta2),0,0;
     sin(theta2),cos(theta2),0,0;
     0,0,1,0;
     0,0,0,1];


%translation
%actual x value of joint 1
L2x = cos(theta2)*b;
%actual y value of joint 1
L2y = sin(theta2)*b;

Tx2 = [1,0,0,-L2x;
    0,1,0,0;
    0,0,1,0;
    0,0,0,1];
Ty2 = [1,0,0,0;
    0,1,0,-L2y;
    0,0,1,0;
    0,0,0,1];

T2 = mtimes(Tx2,Ty2);

H2 = mtimes(T2,R2);
Hf2 = mtimes(H1,H2);
pos2Vect = Hf2(1:3,end);

%joint 3
%rotation
 R3 = [cos(theta3),-sin(theta3),0,0;
     sin(theta3),cos(theta3),0,0;
     0,0,1,0;
     0,0,0,1];


%translation
L3x = cos(theta3)*c;
L3y = sin(theta3)*c;


Tx3 = [1,0,0,-L3x;
      0,1,0,0;
      0,0,1,0;
      0,0,0,1];
  
Ty3 = [1,0,0,0;
       0,1,0,-L3y;
       0,0,1,0;
       0,0,0,1];
   
T3 = mtimes(Tx3,Ty3);

H3 = mtimes(T3,R3);

Hf3 = H1 * H2 * H3;
pos3Vect = Hf3(1:3,end);

% figure(1)
% clf
% hold on;
% axis([-20 20 -20 20])
% plot(x,y);
% line([0,0],[0,H0(14)]);
% line([H0(13),Hf1(13)],[H0(14),Hf1(14)]);
% line([Hf1(13),x],[Hf1(14),y]);
% drawnow
% hold off;

%jacobian at foot

rotMat = [0,0,0;
          0,0,0;
          1,1,1];
  
femurJaco = simplify(vertcat(jacobian(pos1Vect,[theta1,theta2,theta3]),rotMat))
tibiaJaco = simplify(vertcat(jacobian(pos2Vect,[theta1,theta2,theta3]),rotMat))
footJaco = simplify(vertcat(jacobian(pos3Vect,[theta1,theta2,theta3]),rotMat))

% find the torques
% j'F = tau

femurJaco = transpose(femurJaco)
tibiaJaco = transpose(tibiaJaco)
footJaco = transpose(footJaco)

femurTorque = femurJaco*fj1;
tibiaTorque = tibiaJaco*fj2;
footTorque = footJaco*fj3;

end