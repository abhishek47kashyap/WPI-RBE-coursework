clear all
close all
clc

Tf = 100; % Total Time

xinit_vec = [1; 0.5; 0];   % robot starting point [x0, y0, theta0]
xdes_vec  = [0; 0; 0];     % Initial Desired configuration
x0        = xdes_vec - xinit_vec;

% x0 - initial error, which is the initial condition to ode45
[T, errors] = ode45(@(t,x) errordynamics(t,x), [0 Tf], x0); % ode to solve edot=A*e

% trajectory generation
center.x = 0; center.y = 0; R = 3;  % circle parameters
wd = 1/15;  % desired steering velocity

% des_x    = center.x + (R * cos(wd*T));
% des_y    = center.y + (R * sin(wd*T));
des_x    = center.x + (R * sin(2*wd*T));
des_y    = center.y + (R * sin(wd*T));
des_t    = zeros(size(T));
des_traj = [des_x, des_y, des_t];

actualx = des_traj - errors;  % Actual configuration of the robot

figure(1), plot(actualx(:,1), actualx(:,2), 'r', ...
                des_x, des_y, '--b')
legend('Actual position', 'Reference trajectory', 'Location', 'best')
xlabel('X (m)'), ylabel('Y (m)')
title('Tracking a reference trajectory')

error_norm = sqrt(errors(:,1).^2 + errors(:,2).^2 + errors(:,3).^2);
figure(2), plot(T, error_norm), grid on
title('||error||'), xlabel('Time (s)'), ylabel('(m)')


function [de] = errordynamics(t,e)
wd = 1/15;
R  = 3;
dr = 0.7;          % damping ratio 
% vd = R * wd;       % desired linear velocity 
vd = sqrt((R*wd*cos(2*wd*t))^2 + (R*wd*cos(wd*t))^2);
nf = 1;            % natural frequency 

% controller gains
k1 = 2 * dr * nf;       
% k2 = (nf^2-wd^2) / vd; 
k2 = 1;
k3 = k1;

A = [-k1,   wd,    0;
     -wd,    0,   vd;
       0,  -k2,  -k3];

de = A * e;  % e_dot = A * e -- Equation 11.68 in Siciliano's book
end