clear all
close all
clc
format short g

% sin and cosine function handles
C = @(x) cos(x);
S = @(x) sin(x);

% symbolic variables
syms b d c e f g     % lengths
syms t1 t2 t3        % tn = theta_n

% DH table = [d theta a alpha]
DH = [b + d,     0,            -c,    -pi/2;    % link R-1
          0,     t1 - pi/2,     e,     0;       % link 1-2
          0,     t2,            f,     0;       % link 2-3
          0,     t3,            g,     0;       % link 3-4
          0,    -pi/2,          0,    -pi/2];   % link 4-T

% answer 2 - position kinematics
TT = eye(4);

for i = 1:size(DH,1)
    T   = dh2mat(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
    TT  = TT * T;
end
TT = simplify(TT);

% answer 4 - numeric solution
b = 361; c = 250; d = 380; e = 328; f = 323; g = 82.4;   % in mm
t1 = pi/6; t2 = pi/2; t3 = pi/6;                         % in radians
t1_dot = pi/4; t2_dot = pi/4; t3_dot = pi/4;             % in radians/sec

forward_PK = double(subs(TT));     % numeric forward position kinematics

% linear velocity Jacobian
Jx = [f*C(t1+t2) + e*C(t1) + g*C(t1+t2+t3),   f*C(t1+t2) + g*C(t1+t2+t3),    g*C(t1+t2+t3);
                                         0,                            0,                0;
     -g*S(t1+t2+t3) - e*S(t1) - f*S(t1+t2),  -g*S(t1+t2+t3) - f*S(t1+t2),   -g*S(t1+t2+t3)];
% angular velocity Jacobian
Jw = [0,   0,   0;
      1,   1,   1;
      0,   0,   0];

J = [Jx; Jw];

tip_vel = J * [t1_dot; t2_dot; t3_dot];  % numeric instantaneous tip velocity

% force = 50 N (Xr direction w.r.t. robot base)
torques = J' * [-50; 0; 0; 0; 0; 0];

% Mobile kinematics numeric solution (Q7 and Q8)
syms a r wL wR theta  % theta = robot rotation Fr

R = [C(theta), -S(theta), 0;
     S(theta),  C(theta), 0;
     0,         0,        1];   % rotation matrix of Robot w.r.t. F0

% answer 7 - mobile kinematics
vel = simplify(inv(R) * inv([1 0 a/2; 1 0 -a/2; 0 1 0]) * [r * wR; r * wL; 0]);

% answer 8 - numeric solution
a = 507; r = 143; wL = 60; wR = 120; theta = pi/4;
vel = double(subs(vel));

% answer 9 - combined position kinematics
TT = simplify(TT);   % from answer 2
syms X Y theta
% X = displacement along X axis of world frame
% Y = displacement along Y axis of world frame
% theta = rotation about Z axis of world frame

T = [C(theta), -S(theta), 0, X;
    -S(theta),  C(theta), 0, Y;
            0,         0, 1, 0;
            0,         0, 0, 1];

Fw = T * TT;

% answer 10 - numeric solution
X = 2000; Y = 1000; theta = pi/4;
T = double(subs(T));

tip_pos = T * forward_PK;   

% answer 11 - combined velocity kinematics
syms t1_dot t2_dot t3_dot t1 t2 t3 t % t = rotation about Z axis of world frame
syms b d c e f g r a wL wR  % r = radius of the wheels, a = robot base width

% linear velocity Jacobian
Jx = [f*C(t1+t2) + e*C(t1) + g*C(t1+t2+t3),   f*C(t1+t2) + g*C(t1+t2+t3),    g*C(t1+t2+t3);
                                         0,                            0,                0;
     -g*S(t1+t2+t3) - e*S(t1) - f*S(t1+t2),  -g*S(t1+t2+t3) - f*S(t1+t2),   -g*S(t1+t2+t3)];
% angular velocity Jacobian
Jw = [0,   0,   0;
      1,   1,   1;
      0,   0,   0];

J = [Jx; Jw];

% Jacobian relating robot wheels' angular velocity to world frame
Jr = [r*C(t)/2,     r*C(t)/2;
     -r*S(t)/2,    -r*S(t)/2;
             0,            0;
             0,            0;
             0,            0;
           r/a,         -r/a];
 
J_net = [Jr, J];
       
% tip velocity w.r.t. world frame
X_tip_velocity = J_net * [wR; wL; t1_dot; t2_dot; t3_dot];

% answer 12 - force propagation
b = 361; c = 250; d = 380; e = 328; f = 323; g = 82.4;   % in mm
t1 = pi/6; t2 = pi/2; t3 = pi/6; t = pi/4;               % in radians
t1_dot = pi/4; t2_dot = pi/4; t3_dot = pi/4;             % in rad/sec
wL = 60; wR = 120;                                       % in rad/sec
a = 507; r = 143;                                        % in mm

J_net = double(subs(J_net));
torques_wheels = double(J_net' * [-50; 0; 0; 0; 0; 0]);
torques_wheels = torques_wheels(1:2);

% function to calculate DH matrix
function T = dh2mat(d, theta, a, alpha)
c = @(x) cos(x);
s = @(x) sin(x);

T = [c(theta),    -s(theta)*c(alpha),    s(theta)*s(alpha),     a*c(theta);
     s(theta),     c(theta)*c(alpha),   -c(theta)*s(alpha),     a*s(theta);
     0,            s(alpha),             c(alpha),              d;
     0,            0,                    0,                     1];

T = simplify(T);
end