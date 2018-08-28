%RBE 502 - Assignment3
% PD controller for 2DOF robot manipulator
function HW3Q4()
X1 = input('Enter X coordinate of end effector:');
Y1 = input('Enter Y coordinate of end effector:');
Ts = [0 5]; % time span
X0 = initRobotConfig(0.3, 0.45); % initial condition for ode45
Kp = [10 0; 0 10;];
Kv = [10 0; 0 10;];
qd = getInverseKinematics(X1, Y1);
[T, Y] = ode45(@(t, x) robot(t, x, qd, Kp, Kv), Ts, X0);
dim_mat = length(Y);
temp_xl2 = [];
temp_yl2 = [];

e1 = [];
e2 = [];
eq1 = [];
eq2 = [];

% Task Space Error Calculations
for iterator = 1:dim_mat(1,1)
    ret_x = getForwardKinematics(Y(iterator, 3), Y(iterator, 4));
    temp_xl2 = [temp_xl2; ret_x(1)];
    temp_yl2 = [temp_yl2; ret_x(2)];
    e1 = [e1; (X1 - ret_x(1));];
    e2 = [e2; (Y1 - ret_x(2));];
end

% Joint Space Error Calculations
for iterator = 1:dim_mat(1,1)
    ret_x = [Y(iterator, 3) Y(iterator, 4)];
    eq1 = [eq1; (qd(1) - ret_x(1));];
    eq2 = [eq2; (qd(2) - ret_x(2));];
end
eq1
eq2

% Plot the joint angles over time
figure(1);
plot(T,Y(:,3), T, Y(:,4));
title('Joint angles(Radian) vs time')
xlabel('Time (sec)');
ylabel('Angle (radians)');
legend('q_1', 'q_2');
print('-f1','JointAnglesRadian','-dpng')

% Plot the joint angles over time
figure(11);
plot(T,rad2deg(Y(:,3)), T, rad2deg(Y(:,4)));
title('Joint angles(Degree) vs time')
xlabel('Time (sec)');
ylabel('Angle (degree)');
legend('q_1', 'q_2');
print('-f11','JointAnglesDegree','-dpng')

% Plot the joint velocities over time
figure(2);
plot(T,Y(:,5), T, Y(:, 6));
title('Velocities vs time')
xlabel('Time (sec)');
ylabel('Angular velocity (rad/s)');
legend('\omega_1', '\omega_2');
print('-f2','Velocities','-dpng')
T1=diff(Y(:,7))./diff(T);
T2=diff(Y(:,8))./diff(T);
tt=0:(T(end)/(length(T1)-1)):T(end);

% Plot the joint torques over time
figure(3);
plot(tt, T1, tt, T2);
axis([0 5 -100 100]);
title('Control Input as a function of time')
xlabel('Time (sec)');
ylabel('Joint torques (Nm)');
legend('\tau_1', '\tau_2');
print('-f3','ControlInput','-dpng')

% Plot the end effector position over time
figure(4);
plot(T, temp_xl2, T, temp_yl2);
title('End effector position as a function of time')
xlabel('Time (sec)');
ylabel('Position (m)');
legend('x', 'y');
print('-f4','EndPose','-dpng')

% Plot the error in position over time
figure(5);
plot(T, e1, T, e2);
title('Task Space Error')
xlabel('Time (sec)');
ylabel('Position error (m)');
legend('e_x', 'e_y');
print('-f5','Error','-dpng')
display(Y);
display(qd);
display(dim_mat);
display(temp_xl2(dim_mat(1, 1)));
display(temp_yl2(dim_mat(1, 1)));
display(e1(dim_mat(1, 1)));
display(e2(dim_mat(1, 1)));

% Error matching to part m of the solution
% Plot the error in angles over time
figure(6);
plot(T, eq1, T, eq2);
% text1 = 'e1 = -0.0097';
% text2 = 'e2 = -0.0064';
% text3 = 'The error here matches output of part (m)';
% text(4,0.05,text1)
% text(4,-0.05,text2)
% text(2.6,0.2,text3)
title('Joint Space Error')
xlabel('Time (sec)');
ylabel('Angle error (radian)');
legend('e_q1', 'e_q2');
print('-f6','Error1','-dpng')

% Set up first frame of animation
figure(7);
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
axis equal;
axis([-0.6 0.6 -0.6 0.6]);
ht = title(sprintf('Time: %0.2f sec\nPosition: %0.2f\nAngle: %0.2f(Radian) %0.2f(Degree) ', T(1), temp_xl2(1,1), temp_yl2(1,1), rad2deg(temp_yl2(1,1))));
ylabel('Two link manipulator PD control');
% Loop through by changing XData and YData
for id = 1:length(T)
    % Update graphics data. This is more efficient than recreating plots.
    set(hh1(1), 'XData', T(id) , 'YData', temp_xl2(id, 1));
    set(hh1(2), 'XData', T(id) , 'YData', temp_yl2(id, 1));
    
    tempy1 = 0.3*sin(Y(id,3));
    tempx1 = 0.3*cos(Y(id,3));
    
    tempy2= 0.3*sin(Y(id, 3)) + 0.3*sin(Y(id, 3)+Y(id, 4));
    tempx2= 0.3*cos(Y(id, 3))+ 0.3*cos(Y(id, 3)+Y(id, 4));
    
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
end

function [qd] = getInverseKinematics(x, y)
a1 = 0.3;
a2 = 0.3;
q2 = acos((x*x + y*y - a1*a1 - a2*a2)/(2*a1*a2));
q1 = atan2(y, x) - atan2((a2*sin(q2)), (a1 + a2*cos(q2)));
qd = [q1; q2;];
end

function [X] = getForwardKinematics(q1, q2)
a1 = 0.3;
a2 = 0.3;
x = a1*cos(q1) + a2*cos(q1 + q2);
y = a1*sin(q1) + a2*sin(q1 + q2);
X = [x; y;];
end

function xdot = robot(t, x, qd, Kp, Kv)
% Model Parameters
g = 9.8; 
a1 = 0.3; 
a2 = 0.3; 
l1 = 0.15;
l2 = 0.15;
m1 = 0.05;
m2 = 0.05;
I1 = 1;
I2 = 1;

% State variables
q1 = x(3); % theta1
q2 = x(4); % theta2
q3 = x(5); % theta1_dot
q4 = x(6); % theta2_dot

% Mass matrix
M = [(m2*a1*a1 + 2*m2*a1*l2*cos(q2) + m1*l1*l1 + m2*l2*l2 + I1 + I2),...
    (m2*(l2*l2 + a1*l2*cos(q2)) + I2);
    (m2*(l2*l2 + a1*l2*cos(q2)) + I2),...
    (m2*l2*l2 + I2);];

% Centrifugal/Coriolis matrix
C = [(-m2*a1*l2*sin(q2)*2*q4),...
    (-m2*a1*l2*sin(q2)*q4);
    (m2*a1*l2*sin(q2)*q3),...
    0;];

% Gravity matrix
G = [((m1*l1 + m2*a1)*g*cos(q1) + m2*l2*g*cos(q1 + q2));
    (m2*l2*g*cos(q1 + q2));];

%Initialize xdot
xdot = zeros(8, 1);
qd1 = qd(1);
qd2 = qd(2);
Kp1 = Kp(1, 1);
Kp2 = Kp(2, 2);
Kv1 = Kv(1, 1);
Kv2 = Kv(2, 2);
tau = [0; 0;];
tau(1) = Kp1*(qd1 - x(3)) - Kv1*x(5);
tau(2) = Kp2*(qd2 - x(4)) - Kv2*x(6);

% For part 1 of question
qdot = inv(M)*(tau - C*[q3; q4]);
% For part 2 of question
% qdot = inv(M)*(tau - C*[q3; q4] - G);
xdot(1) = qd1 - x(3);
xdot(2) = qd2 - x(4);
xdot(3) = x(5);
xdot(4) = x(6);
xdot(5) = qdot(1);
xdot(6) = qdot(2);
xdot(7) = tau(1);
xdot(8) = tau(2);
end

function X = initRobotConfig(xi, yi)
X = zeros(8,1);
X(1) = xi;
X(2) = yi;
qi = getInverseKinematics(xi, yi);
X(3) = qi(1);
X(4) = qi(2);
X(5) = 0;
X(6) = 0;
X(7) = 0;
X(8) = 0;
end
