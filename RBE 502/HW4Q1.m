clc
clear all;
close all;

global err prev_time
err        = [0; 0];   % for accumulating the Integral error
prev_time  = 0;        % Integral error = error * dt

x0 = [-0.5;      0.2;   0.1;   0.1]; % initial condition: [theta1,theta2,dtheta1,dtheta2]
xf = [pi/30;   pi/30;     0;     0];   % final conditions
tf = 2;                 % time span

% the options for ode - Optional!
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);

[T,X] = ode45(@(t,x) pelican_PID(t,x,xf), [0 tf], x0, options);

figure, plot(T, xf(1) - X(:,1), 'r', T, xf(2) - X(:,2), 'b')
title('PID control'), xlabel('Time (s)'), ylabel('Radians'), grid on
legend('e_{{\theta}_{1}}', 'e_{{\theta}_{2}}', 'Location', 'best')

function dXdt = pelican_PID(t,X,xf)
dXdt = zeros(4,1);

% Link masses and lengths
l1   = 0.26;    l2   = 0.26;
lc1  = l1/2;    lc2  = l2/2;
m1   = 6.5225;  m2   = 2.0458;
I1   = 0.1213;  I2   = 0.0116;
g    = 9.81;

A  = m2*lc2^2 + I2;
B  = A + m1*lc1^2 + m2*l1^2 + I1;
D  = m2*l1*lc2;
F  = 2*m2*l1*l2;
L  = (m1*lc1 + m2*l1) * g;
H  = m2*lc2*g;

% global M C G Gd
Gd = [L * sin(xf(1)) + H * sin(xf(1) + xf(2));
      H * sin(xf(1) + xf(2))];
Gd = vpa(Gd, 4);

M = [B + F*cos(X(2)),      A + D*cos(X(2));
     A + D*cos(X(2)),      A];
 
C = [-D*sin(X(2))*X(4),       -D*sin(X(2))*(X(3)+X(4));
      D*sin(X(2))*X(3),        0];
  
G = [L * sin(X(1)) + H * sin(X(1) + X(2));
     H * sin(X(1) + X(2))]; 

invM  = inv(M);
invMC = invM * C; 

e   = xf(1:2) - X(1:2);  % position error
de  = xf(3:4) - X(3:4);  % velocity error

% PID constants
Ki = diag([70, 100]);
Kv = diag([7, 3]);
Kp = diag([30, 30]);


global err prev_time
err = err + e * (t - prev_time);   % for adding up Ki errors

tau = (Kp * e) + (Kv * de) + (Ki * (err - (inv(Ki) * Gd)));

dXdt(1:2) = X(3:4);

dXdt(3:4) = invM*tau - invMC*X(3:4) + invM*Gd - invM*G;
                 
prev_time = t;

end

% Implement the computed Torque control.
% index = 1;


% figure('Name','Theta_1 under Computed Torque Control');
% plot(T, X(:,1),'r-');
% 
% figure('Name','Theta_2 under Computed Torque Control');
% plot(T, X(:,2),'r--');
% hold on
% plot(T, sin(2*T),'b-');

% figure('Name','Input_Computed Torque Control');
% plot(T, torque(1,1:size(T,1)),'-' );
% hold on
% plot(T, torque(2,1:size(T,1)),'r--');
% torque=[];

% % Implement the PD control plus Feedforward.
% index=2;
% [T,X] = ode45(@(t,x)plannarArmODE(t,x,index),[0 tf],x0,options);
% figure('Name','Theta_1 under PD Control plus Feedforward');
% plot(T, X(:,1),'r-');
% hold on
% plot(T, w*ones(size(T,1),1),'b-');
% figure('Name','Theta_2 under PD Control plus Feedforward');
% plot(T, X(:,2),'r--');
% hold on
% plot(T, sin(2*T),'b-');
% hold on


% figure('Name','Input_ PD control plus Feedforward');
% plot(T, torque(1,1:size(T,1)),'-' );
% hold on
% plot(T, torque(2,1:size(T,1)),'r--');
% hold on



% %% 
% function [dx ] = plannarArmODE(t,x,idx)
% theta_d= [w;sin(2*t)]; %You can insert any desired joint trajectory here!
% dtheta_d =[0; 2*cos(2*t)]; %Time derivative of theta_d
% ddtheta_d = [0; -4*sin(2*t)]; %Second time derivated of theta_d
% theta= x(1:2,1);
% dtheta= x(3:4,1);
% 
% global Mmat Cmat Mmatd Cmatd
% Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
% Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
% Mmatd = [a+2*b*cos(theta_d(2)), d+b*cos(theta_d(2));  d+b*cos(theta_d(2)), d];
% Cmatd = [-b*sin(theta_d(2))*dtheta_d(2), -b*sin(theta_d(2))*(dtheta_d(1)+dtheta_d(2)); b*sin(theta_d(2))*dtheta_d(1),0];
% invM = inv(Mmat);
% invMC = invM*Cmat;
% switch idx
%     case 1
%         tau = computeTorque(theta_d, dtheta_d, ddtheta_d, theta, dtheta); %
%         
%     case 2
%         tau = PDplusFeedforwawrd(theta_d, dtheta_d, ddtheta_d, theta, dtheta);
% end
% torque =[torque, tau];
% dx=zeros(4,1);
% dx(1) = x(3);
% dx(2) = x(4);
% dx(3:4) = -invMC* x(3:4) +invM*tau; % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau
% end
% 
% function tau =computeTorque(theta_d, dtheta_d,ddtheta_d, theta, dtheta)
% global Mmat Cmat
% Kp=100*eye(2);
% Kv=100*eye(2);
% e=theta_d-theta; % position error
% de = dtheta_d - dtheta; % velocity error
% tau= Mmat*(Kp*e + Kv*de) + Cmat*dtheta + Mmat*ddtheta_d;
% end
% 
% function tau = PDplusFeedforwawrd(theta_d, dtheta_d,ddtheta_d, theta, dtheta)
% global Mmatd Cmatd
% Kp=100*eye(2);
% Kv=100*eye(2);
% e=theta_d-theta; % position error
% de = dtheta_d - dtheta; % velocity error
% tau= (Kp*e + Kv*de) + Cmatd*dtheta_d + Mmatd*ddtheta_d;
% end
% disp('Finish.');



%% My shit that doesn't work
% clear all
% clc 
% 
% x0 = [ -0.5;     0.2;   0.1;   0.1];    % Initial conditions = [q1 q2 dq1 dq2]
% xf = [pi/10;   pi/30;     0;     0];   % final conditions
% tf = 2;                        % Time span
% 
% global err prev_time
% err        = [0; 0];   % for accumulating the Integral error
% prev_time  = 0;        % Integral error = error * dt
% 
% % Link masses and lengths
% l1  = 0.26;    l2  = 0.26;
% lc1 = l1/2;    lc2 = l2/2;
% m1  = 6.5225;  m2  = 2.0458;
% I1  = 0.1213;  I2  = 0.0116;
% g   = 9.81;
% 
% global A B F D H L Gd
% A  = m2*lc2^2 + I2;
% B  = A + m1*lc1^2 + m2*l1^2 + I1;
% D  = m2*l1*lc2;
% F  = 2*m2*l1*l2;
% L  = (m1*lc1 + m2*l1) * g;
% H  = m2*lc2*g;
% Gd = [L * sin(xf(1)) + H * sin(xf(1) + xf(2));
%       H * sin(xf(1) + xf(2))];
% Gd = vpa(Gd, 4);
% 
% disp('Solving for states now')
% % options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
% % [t, X]  = ode45(@(t,X) pelican(t,X,xf), [0 tf], x0, options);
% [t, X]  = ode45(@(t,X) pelican(t,X,xf), [0 tf], x0);
% disp('Obtained all states')
% 
% % figure, plot(t, X(:,1), 'r', t, X(:,2), 'b', t, zeros(length(X),1), 'm', 'LineWidth', 2)
% % legend('\theta_1', '\theta_2', 'reference', 'Location', 'best'), 
% % grid on, xlabel('Time'), ylabel('Radians')
% % disp('Plotted')
% 
% function dXdt = pelican(t,X,xf)
% dXdt = zeros(4,1);
% 
% global A B F D H L Gd
% 
% M = [B + F*cos(X(2)),      A + D*cos(X(2));
%      A + D*cos(X(2)),      A];
%  
% C = [-D*sin(X(2))*X(4),       -D*sin(X(2))*(X(3)+X(4));
%       D*sin(X(2))*X(3),        0];
%   
% G = [L * sin(X(1)) + H * sin(X(1) + X(2));
%      H * sin(X(1) + X(2))];   
% 
% M  = vpa(M, 4);
% C  = vpa(C, 4);
% G  = vpa(G, 4);
% 
% % PID constants
% Ki = diag([70, 100]);
% Kv = diag([7, 3]);
% Kp = diag([30, 30]);
% 
% global err prev_time
% err = err + (xf(1:2) - X(1:2)) * (t - prev_time);   % for adding up Ki errors
% 
% dXdt(1:2) = -X(3:4);
% 
% dXdt(3:4) = vpa(-M \ (Kp * (xf(1:2) - X(1:2)) + Kv * (xf(3:4) - X(3:4)) + ...
%                      Ki * (err - (Ki \ Gd)) - C * X(3:4) - G), 4);
%                  
% prev_time = t;
% 
% % xf(1:2) - X(1:2)
% end