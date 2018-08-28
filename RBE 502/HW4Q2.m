clc
clear all;
close all;

% parameters for the arm
global a b d I1 I2 m1 r1 m2 l1 l2 r2 g
l1  = 0.26;    l2  = 0.26;
r1  = l1/2;    r2  = l2/2;
m1  = 6.5225;  m2  = 2.0458;
I1  = 0.1213;  I2  = 0.0116;
g   = 9.81;

a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;

x0 = [0, 0, 0, 0];  % initial condition - Format:[theta1,theta2,dtheta1,dtheta2]
tf = 10;            % time span

[T,X] = ode45(@(t,x) pelican(t,x),[0 tf],x0);


% plotting
b1 = pi/4; b2 = pi/3; c1 = pi/9; c2 = pi/6; w1 = 4; w2 = 3;

Xd = horzcat(b1*(1-exp(-2*T.^3)) + c1*(1-exp(-2*T.^3)).*sin(w1*T), ...
             b2*(1-exp(-2*T.^3)) + c2*(1-exp(-2*T.^3)).*sin(w2*T));
         
figure, plot(T, Xd(:,1) - X(:,1), 'r', T, Xd(:,2) - X(:,2), 'b')
title('Computed torque control'), xlabel('Time (s)'), ylabel('Angle (rad)')
grid on, legend('error \theta_{1}', 'error \theta_{2}', 'Location', 'best')



function [dx] = pelican(t,x)
theta_d   = [(pi/4)*(1-exp(-2*t^2))+((pi/9)*(1-exp(-2*t^3))*sin(4*t));
             (pi/3)*(1-exp(-2*t^2))+((pi/6)*(1-exp(-2*t^3))*sin(4*t))];
dtheta_d  = [6*(pi/4)*(t^2)*exp(-2*t^3)*sin(4*t)+((pi/9)-(pi/9)*exp(-2*t^3)*cos(4*t)*4);
             6*(pi/3)*(t^2)*exp(-2*t^3)*sin(3*t)+((pi/6)-(pi/6)*exp(-2*t^3)*cos(3*t)*3)];
ddtheta_d = [(12*(pi/4)*t*exp(-2*t^3))-(36*(pi/4)*(t^4)*exp(-2*t^3))+(12*(pi/9)*t*exp(-2*t^3)*sin(4*t))-(36*(pi/9)*(t^4)*exp(-2*t^3)*sin(4*t))+(12*(pi/9)*(t^2)*exp(-2*t^3)*cos(4*t)*4)-((pi/9)-(pi/9)*exp(-2*t^3))*sin(4*t)*4;
             (12*(pi/3)*t*exp(-2*t^3))-(36*(pi/3)*(t^4)*exp(-2*t^3))+(12*(pi/6)*t*exp(-2*t^3)*sin(3*t))-(36*(pi/3)*(t^4)*exp(-2*t^3)*sin(3*t))+(12*(pi/6)*(t^2)*exp(-2*t^3)*cos(3*t)*3)-((pi/6)-(pi/6)*exp(-2*t^3))*sin(3*t)*3]; %Second time derivated of theta_d
theta= x(1:2,1);
dtheta= x(3:4,1);

global Mmat Cmat Mmatd Cmatd a b d m1 r1 m2 l1 r2 g Gmat

Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
Mmatd = [a+2*b*cos(theta_d(2)), d+b*cos(theta_d(2));  d+b*cos(theta_d(2)), d];
Cmatd = [-b*sin(theta_d(2))*dtheta_d(2), -b*sin(theta_d(2))*(dtheta_d(1)+dtheta_d(2));
          b*sin(theta_d(2))*dtheta_d(1),  0];
Gmat = [(m1*r1 + m2*l1)*g*sin(x(1)) + m2*r2*g*sin(x(1) + x(2));
          m2*r2*g*sin(x(1) + x(2))];
invM  = inv(Mmat);
invMC = invM*Cmat;

tau = computeTorque(theta_d, dtheta_d, ddtheta_d, theta, dtheta);

dx      = zeros(4,1);
dx(1)   = x(3);
dx(2)   = x(4);
dx(3:4) = (invM * tau) - (invMC * x(3:4)) - (invM * Gmat); 
end

function tau =computeTorque(theta_d, dtheta_d,ddtheta_d, theta, dtheta)
global Mmat Cmat Gmat
Kp  = diag([1500, 14000]);
Kv  = diag([77.46, 236.64]);
e   = theta_d-theta; % position error
de  = dtheta_d - dtheta; % velocity error
tau = Mmat*(ddtheta_d + Kp*e + Kv*de) + Cmat*dtheta + Gmat;
end



% clc
% clear all;
% close all;
% 
% 
% l1  = 0.26;
% l2  = 0.26;
% lc1 = 0.0983;
% lc2 = 0.0229;
% m1  = 6.5225;
% m2  = 2.0458;
% I1  = 0.1213;
% I2  = 0.0116;
% g   = 9.81;
% 
% 
% a = I1 + I2 + m1*lc1^2+ m2*(l1^2+ lc2^2);
% b = m2*l1*lc2;
% d = I2 + m2*lc2^2;
% 
% 
% x0 = [0,0,0,0]; %You can change the initial condition here. %q1 %q2 %q1_dot %q2_dot
% 
% tf = 0.5;
% 
% global err
% err = [];
% 
% options = odeset('RelTol', 1e-4, 'AbsTol', [1e-4, 1e-4, 1e-4, 1e-4]);
% [T,X]   = ode45(@(t,x)planarArmODE(t,x),[0 tf],x0,options);
% dim_mat = length(X);
% 
% 
% 
% 
% % theta_d = horzcat(pi/10*ones(size(T,1),1), pi/30*ones(size(T,1),1));
% figure(1);
% plot(1:40, err(1,1:40), 'r', 1:40, err(2,1:40), 'b');
% legend('error1', 'error2');
% xlabel('Time (s)');
% ylabel('Error in Joint Angles (rad)');
% grid on;
% 
% 
% function dx = planarArmODE(t,x)
% 
% w1 = 38.7;   w2 = 118.3;
% b1 = pi/4;   b2 = pi/3;
% c1 = pi/9;   c2 = pi/6;
% 
% theta_d   = [b1*(1-exp(-2*t^3)) + c1*(1-exp(-2*t^3))*sin(w1*t);
%              b2*(1-exp(-2*t^3)) + c2*(1-exp(-2*t^3))*sin(w2*t)];
% dtheta_d  = [6*(pi/4)*(t^2)*exp(-2*t^3)*sin(4*t)+((pi/9)-(pi/9)*exp(-2*t^3)*cos(4*t)*4);
%              6*(pi/3)*(t^2)*exp(-2*t^3)*sin(3*t)+((pi/6)-(pi/6)*exp(-2*t^3)*cos(3*t)*3)]; 
% theta     = x(1:2);
% dtheta    = x(3:4);
% 
% global Mmat Cmat Gmat % Dynamic Model of Robot
% global a b d m1 m2 lc2 lc1 l1 g err
% 
% Mmat = [a + 2*b*cos(theta(2)),     d + b*cos(theta(2));
%         d + b*cos(theta(2))  ,                      d];
% Gmat = [(m1*lc1 + m2*l1)*g*sin(theta(1)) + m2*lc2*g*sin(theta(1) + theta(2));
%          m2*lc2*g*sin(theta(1) + theta(2))];
% Cmat = [-b*sin(theta(2))*dtheta(2), -b*sin(theta(2))*(dtheta(1) + dtheta(2));
%                  b*sin(theta(2))*dtheta(1),                                       0];
% 
% 
% % Tuning parameters: Gain Values
% Kp = diag([1500, 14000]);
% Kv = diag([77.46, 236.64]);
% e =  theta_d - theta; % position error
% de = dtheta_d - dtheta; % velocity error
% 
% err = [err, e];
% 
% % tau= Mmat*(ddtheta_d + Kp*e + Kv*de) + Cmat*dtheta + Gmat;
% 
% dx      = zeros(4,1);
% dx(1)   = x(3);
% dx(2)   = x(4);
% dx(3:4) = -Kp*e - Kv*de;
% 
% end