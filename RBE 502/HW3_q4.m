clc
clear all
close all
digits(5)

% qd = input('Please enter desired position as a 2x1 vector = ');
% 
% if isrow(qd)
%     qd = qd';
% elseif ~iscolumn(qd)
%     error('Error!! \nInput must be a vector.', class(qd))
% end
% 
% xd = qd(1);
% yd = qd(2);
% R  = norm(qd);

syms l1  l2 ...
     a1  a2 ...
     I1  I2 ...
     theta1(t) theta2(t) ... 
     m1  m2 ...
     t1r t2r ...
     td1r td2r ...
     t   g;
 
global theta1 theta2
 
th1  = theta1;
th2  = theta2;

x1 = l1 * cos(th1);
y1 = l1 * sin(th1);
X1 = [x1; y1];
x2 = (a1*cos(th1)) + (l2*cos(th1+th2));
y2 = (a1*sin(th1)) + (l2*sin(th1+th2));
X2 = [x2; y2];

XD1 = diff(X1,t);
XD2 = diff(X2,t);

v1s = simplify(XD1.' * XD1);
v2s = simplify(XD2.' * XD2);
k1  = (m1 * v1s * 0.5) + (I1 * (diff(th1, t))^2 * 0.5);
k2  = (m2 * v2s * 0.5) + (I2 * (diff(th1, t) + diff(th2, t))^2 * 0.5);
KE  = simplify(k1 + k2);
p1  = m1 * g * y1;
p2  = m2 * g * y2;
PE  = simplify(p1 + p2);

L = simplify(KE - PE);

aP    = {theta1, theta2, diff(theta1(t), t), diff(theta2(t), t)};
sP    = {   t1r,    t2r,               td1r,           td2r};
torq1 = diff(subs( diff(subs( L, aP, sP), td1r),sP, aP),t) - subs( diff(subs( L, aP, sP), t1r), sP, aP);
torq2 = diff(subs( diff(subs( L, aP, sP), td2r),sP, aP),t) - subs( diff(subs( L, aP, sP), t2r) ,sP, aP);

torques = [simplify(torq1); simplify(torq2)];
torques = torques(t); % convert symfunct to sym
clear aP sP torq1 torq2

M11 = simplify(torques(1) - subs(torques(1),diff(theta1(t), t, t),0)) /diff(theta1(t), t, t);
M12 = simplify(torques(1) - subs(torques(1),diff(theta2(t), t, t),0)) /diff(theta2(t), t, t);
M21 = simplify(torques(2) - subs(torques(2),diff(theta1(t), t, t),0)) /diff(theta1(t), t, t);
M22 = simplify(torques(2) - subs(torques(2),diff(theta2(t), t, t),0)) /diff(theta2(t), t, t);

global M C G
M   = [M11 M12; M21 M22];
clear M11 M12 M13 M14

G = subs(torques, {diff(theta1(t), t, t), diff(theta2(t), t, t), diff(theta1(t), t), diff(theta2(t), t)}, {0,0,0,0});

C1 = simplify(torques(1) - (M(1,:) * [diff(theta1(t), t, t) diff(theta2(t), t, t)].' + G(1)));
C2 = simplify(torques(2) - (M(2,:) * [diff(theta1(t), t, t) diff(theta2(t), t, t)].' + G(2)));
C  = [C1; C2];

global Kp Kv 
a1 = 0.3; a2 = 0.3; l1 = 0.15; l2 = 0.15;
m1 = 0.05; m2 = 0.05; g = 9.8; I1 = 1; I2 = 1;
Kp = 10; Kv = 10;    % PD gains

% Inverse kinematics -- task space to joint space
%     acos() produces imaginary values at times, so df1() and df2()
%     have been used as dummy functions to convert all acos() operations
%     to atan2().
df1 = @(x,y) (x^2 + y^2 + a1^2 - a2^2) / (2 * a1 * sqrt(x^2 + y^2));
df2 = @(x,y) (x^2 + y^2 - a1^2 - a2^2) / (2 * a1 * a2);
IK  = @(x,y) ...
           [atan2(y,x) - atan2(1-df1(x,y)^2, df1(x,y));
            atan2(1-df2(x,y)^2, df2(x,y))];

Q_init = IK(0.3, 0.45);   % initial XY position
Q_des  = IK(-0.3, 0.45);  % final XY position -- input from user


M = subs(M);
C = subs(C);

x_init = [Q_init; 0; 0];        % initial state

global control_input
control_input = [];

[time, states] = ode45(@(t,x) two_link_arm(t,x, [Q_des; 0; 0]), [0 10], x_init);

function error_dot = two_link_arm(t, x, desired_state)
    global M C Kp Kv control_input theta1 theta2
     
    error = desired_state - x;
    
    M = subs(M, [theta1(t), theta2(t)], [x(1), x(2)]);
    C = subs(C, [theta1(t), theta2(t), diff(theta1(t),t), diff(theta2(t),t)], ...
                [     x(1),      x(2),              x(3),              x(4)]);
    
    tau             = Kp * error(1:2) + Kv * error(3:4);
    error_dot       = [error(3:4); M \ (tau - (C .* x(3:4)))];
    control_input   = [control_input, tau];
end