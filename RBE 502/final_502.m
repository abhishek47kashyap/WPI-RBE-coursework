clc
clear all
close all

Ts = [0 15]; % time span
X0 = initRobotConfig(0.1, -0.3); % initial condition for ode45
Kp = [15 0; 0 15];    % proportional gain
Kv = [12 0; 0 12];    % derivative gain
Ki = [0.1 0; 0 0.1];  % integral gain

global err fl1 fl2 fl3 fl4 fl5 fl6 QD
err = [0; 0];
fl1 = 0; fl2 = 0; fl3 = 0; fl4 = 0; fl5 = 0; fl6 = 0;
QD  = [];   % recording desired trajectory of end-effector

% The 6 flags, fl(i), are to refresh the 'err' for the Integral error
% at the beginning of every sub-trajectory. Not using them causes errors
% from Phase 1 to carry over to Phase 2 which is not helpful.

[T, Y] = ode45(@(t, x) robot(t, x, Kp, Kv, Ki), Ts, X0);

% Plot the joint angles over time
figure(1), plot(T,Y(:,3), T, Y(:,4))
title('Joint angles (radian) vs time'), grid on
xlabel('Time (sec)'), ylabel('Angle (radians)')
legend('\theta_1', '\theta_2', 'Location', 'best')
% print('-f1','JointAnglesRadian','-dpng')

% Plot the joint angles over time
figure(2), plot(T,rad2deg(Y(:,3)), T, rad2deg(Y(:,4)))
title('Joint angles (degree) vs time'), grid on
xlabel('Time (sec)'), ylabel('Angle (degree)')
legend('\theta_1', '\theta_2', 'Location', 'best')
% print('-f11','JointAnglesDegree','-dpng')

% Plot the joint velocities over time
figure(3), plot(T,Y(:,5), T, Y(:, 6))
title('Velocities vs time'), grid on
xlabel('Time (sec)'), ylabel('Angular velocity (rad/s)')
legend('\omega_1', '\omega_2', 'Location', 'best')
% print('-f2','Velocities','-dpng')

% Plot the joint torques over time
T1 = diff(Y(:,7))./diff(T);
T2 = diff(Y(:,8))./diff(T);
tt = 0:(T(end)/(length(T1)-1)):T(end);

figure(4), plot(tt, T1, tt, T2)
axis([0 5 -20 20]), grid on
title('Control Input as a function of time')
xlabel('Time (sec)'), ylabel('Joint torques (Nm)')
legend('\tau_1', '\tau_2', 'Location', 'best')
% print('-f3','ControlInput','-dpng')

% Plot the end effector position over time
temp_xl2 = [];
temp_yl2 = [];
for iterator = 1:length(Y)
    ret_x = getForwardKinematics(Y(iterator, 3), Y(iterator, 4));
    temp_xl2 = [temp_xl2; ret_x(1)];
    temp_yl2 = [temp_yl2; ret_x(2)];
end

figure(5), plot(temp_xl2, temp_yl2, QD(1,:), QD(2,:))
title('End effector position and desired trajectory')
xlabel('Time (sec)'), ylabel('Position (m)'), grid on
legend('pos_{actual}', 'pos_{des}', 'Location', 'best')
% print('-f4','EndPose','-dpng')

% Plot end-effector position x and y vs. time
figure(6), plot(T, temp_xl2, T, temp_yl2)
title('End-effector position (x,y) vs. time')
xlabel('Time (sec)'), ylabel('Position (m)'), grid on
legend('x-position', 'y-position', 'Location', 'best') 

% Plot end-effector velocity x and y vs. time
V1 = diff(temp_xl2)./diff(T);
V2 = diff(temp_yl2)./diff(T);
tt = 0:(T(end)/(length(V1)-1)):T(end);

figure(7), plot(tt, V1, tt, V2)
title('End-effector velocity (v_x,v_y) vs. time')
xlabel('Time (sec)'), ylabel('Velocity (m)'), grid on
legend('v_x', 'v_y', 'Location', 'best') 


% Set up first frame of animation
figure(8);
clf('reset');
subplot(2,1,1);
plot(T,temp_xl2,T,temp_yl2);
hh1(1) = line(T(1), temp_xl2(1,1), 'Marker', '.', 'MarkerSize', 20, ...
    'Color', 'b');
hh1(2) = line(T(1), temp_yl2(1,1), 'Marker', '.', 'MarkerSize', 20, ...
    'Color', [0 .5 0]);
xlabel('time (sec)'); ylabel('Position');
subplot(2,1,2);
hh2 = line([0, 1*cos(Y(3,1))],[0, 1*sin(Y(3,1))]);
% % axis equal;
axis([-0.4 0.4 -0.5 0]);

line([-0.1 0.1], [-0.3 -0.3], 'Color', 'black');
ht = title(sprintf('Time: %0.2f sec\nPosition: %0.2f\nAngle: %0.2f(Radian) %0.2f(Degree) ', T(1), temp_xl2(1,1), temp_yl2(1,1), rad2deg(temp_yl2(1,1))));
ylabel('Quadruped leg control');
% Loop through by changing XData and YData
for id = 1:length(T)
    % Update graphics data. This is more efficient than recreating plots.
    set(hh1(1), 'XData', T(id) , 'YData', temp_xl2(id, 1));
    set(hh1(2), 'XData', T(id) , 'YData', temp_yl2(id, 1));
    
    tempy1 = 0.26*sin(Y(id,3));
    tempx1 = 0.26*cos(Y(id,3));
    
    tempy2= 0.26*sin(Y(id, 3)) + 0.26*sin(Y(id, 3)+Y(id, 4));
    tempx2= 0.26*cos(Y(id, 3))+ 0.26*cos(Y(id, 3)+Y(id, 4));
    
    set(hh2(1), 'XData', [0, tempx1, tempx2],...
        'YData', [0, tempy1, tempy2]);
    set(ht, 'String', sprintf('Time: %0.2f sec \nAngle: %0.2f(Radian) %0.2f(Degree) ', [T(id), Y(id,3), rad2deg(Y(id,3))]));
    % Get frame as an image
    f = getframe(gcf);
    % Create a colormap for the first frame. For the rest of the frames,
    % use the same colormap
    if id == 1
        [mov(:,:,1,id), map] = rgb2ind(f.cdata, 256, 'nodither');
    else
        mov(:,:,1,id) = rgb2ind(f.cdata, map, 'nodither');
    end
end
imwrite(mov, map, 'stickAnimation.gif', 'DelayTime', 0, 'LoopCount', inf);

function [qd] = getInverseKinematics(x, y)
a1 = 0.26;
a2 = 0.26;
q2 = acos((x*x + y*y - a1*a1 - a2*a2)/(2*a1*a2));
q1 = atan2(y, x) - atan2((a2*sin(q2)), (a1 + a2*cos(q2)));
qd = [q1; q2;];
end

function [X] = getForwardKinematics(q1, q2)
a1 = 0.26;
a2 = 0.26;
x = a1*cos(q1) + a2*cos(q1 + q2);
y = a1*sin(q1) + a2*sin(q1 + q2);
X = [x; y];
end

function xdot = robot(t, x, Kp, Kv, Ki)

global err fl1 fl2 fl3 fl4 fl5 fl6 QD

% Model Parameters
g = 9.8; 
a1 = 0.26; 
a2 = 0.26; 
l1 = 0.0983;
l2 = 0.0229;
m1 = 6.5225;
m2 = 2.0458;
I1 = 0.1213;
I2 = 0.0116;

% State variables
q1 = x(3); % theta1
q2 = x(4); % theta2
q3 = x(5); % theta1_dot
q4 = x(6); % theta2_dot

% Desired trajectory components
if t <= 10
    if fl1 == 0
        err = [0; 0];
        fl1 = 1;
    end
    xd = 0.1 - (0.02 * t);
    yd = -0.3;
    qdDot  = [0;0];
    qdDDot = [0;0];
elseif t > 10 && t <= 11
    if fl2 == 0
        err = [0; 0];
        fl2 = 1;
    end
    xd = -0.1;
    yd = -0.3 + 0.1*(t-10)^2;
    qdDot  = [0; 0.1*(t-11)];
    qdDDot = [0; 0.1];   
elseif t > 11 && t <= 12
    if fl3 == 0
        err = [0; 0];
        fl3 = 1;
    end
    xd = -0.1 + 0.1*(t-11)^2;
    yd = -0.2 + (0.1 - 0.1*(t-11))*(t-11);
    qdDot  = [0.1*(t-11);  0.1-0.1*(t-11)];
    qdDDot = [0.1; -0.1];
elseif t > 12 && t <= 13
    if fl4 == 0
        err = [0; 0];
        fl4 = 1;
    end
    xd = 0.1*(t-12);
    yd = -0.2;
    qdDot  = [0.1; 0];
    qdDDot = [0;   0];
elseif t > 13 && t <= 14
    if fl5 == 0
        err = [0; 0];
        fl5 = 1;
    end
    xd = 0.1 + (0.1 - 0.1*(t-13))*(t-13);
    yd = -0.2 - 0.1*(t-13)^2;
    qdDot  = [0.1 - 0.1*(t-13);    -0.1*(t-13)];
    qdDDot = [-0.1; -0.1];
else
    if fl6 == 0
        err = [0; 0];
        fl6 = 1;
    end
    xd = 0.1;
    yd = -0.3 + (-0.1 + 0.1*(t-14))*(t-14);
    qdDot  = [0; -0.1 + 0.1*(t-14)];
    qdDDot = [0; 0.1];
end

QD  = [QD, [xd; yd]];
qd2 = acos((xd*xd + yd*yd - a1*a1 - a2*a2)/(2*a1*a2));
qd1 = atan2(yd, xd) - atan2((a2*sin(qd2)), (a1 + a2*cos(qd2)));
qd  = [qd1; qd2];

% Mass matrix
M = [(m2*a1*a1 + 2*m2*a1*l2*cos(q2) + m1*l1*l1 + m2*l2*l2 + I1 + I2),...
     (m2*(l2*l2 + a1*l2*cos(q2)) + I2);
     (m2*(l2*l2 + a1*l2*cos(q2)) + I2),...
     (m2*l2*l2 + I2)];

Md = [(m2*a1*a1 + 2*m2*a1*l2*cos(qd2) + m1*l1*l1 + m2*l2*l2 + I1 + I2),...
     (m2*(l2*l2 + a1*l2*cos(qd2)) + I2);
     (m2*(l2*l2 + a1*l2*cos(qd2)) + I2),...
     (m2*l2*l2 + I2)];

% Centrifugal/Coriolis matrix
C = [(-m2*a1*l2*sin(qd2)*2*q4),        (-m2*a1*l2*sin(q2)*q4);
     (m2*a1*l2*sin(q2)*q3),                               0];
 
Cd = [(-m2*a1*l2*sin(qd2)*2*qdDot(2)),        (-m2*a1*l2*sin(qd2)*qdDot(2));
      (m2*a1*l2*sin(qd2)*qdDot(1)),                               0];

% Gravity matrix
G = [((m1*l1 + m2*a1)*g*cos(q1) + m2*l2*g*cos(q1 + q2));
     (m2*l2*g*cos(q1 + q2))];

Gd = [((m1*l1 + m2*a1)*g*cos(qd1) + m2*l2*g*cos(qd1 + qd2));
     (m2*l2*g*cos(qd1 + qd2))];


% tau = Kp*(qd - [q1; q2]) + Kv*(qdDot - [q3; q4]) + ...
%          (Md * qdDDot) + (Cd * qdDot) + Gd;   % PD plus FeedForward control -- extremely slow
 
tau = M*(qdDDot + Kv*(qdDot - [q3; q4]) + Kp*(qd - [q1; q2])) ...
          +C*[q3; q4] + G;  % computed torque

% tau = Kv*(qdDot - [q3; q4]) + ...
%       Kp*(qd - [q1; q2]) + G;  % PD with grav. comp. -- extremely slow

err = err + (qd - [q1; q2]);
% tau = Kv*(qdDot - [q3; q4]) + Kp*(qd - [q1; q2] + Ki*err);  % PID  -- extremely slow
% Also, shows this warning:
% Warning: Failure at t=1.721991e+00.  Unable to meet integration tolerances without
% reducing the step size below the smallest value allowed (3.552714e-15) at time t. 
      
qddot = inv(M)*(tau - C*[q3; q4] - G);

%Initialize xdot
xdot        = zeros(12, 1);
xdot(1:2)   = [qd1; qd2] - x(3:4);
xdot(3:4)   = x(5:6);
xdot(5:6)   = double(qddot);
xdot(7:8)   = double(tau);
xdot(9:10)  = getForwardKinematics(qd(1) , qd(2));
xdot(11:12) = getForwardKinematics(qd(1) , qd(2)) - getForwardKinematics(q1, q2);

end

function X = initRobotConfig(xi, yi)
X = [xi; 
     yi; 
     getInverseKinematics(xi, yi); 
     zeros(8,1)];
end
