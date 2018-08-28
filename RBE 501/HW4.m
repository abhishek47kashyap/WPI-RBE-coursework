clear all
close all
clc

% defining function handles
c = @(x) cos(x);
s = @(x) sin(x);

% answer 1
syms q1 q2 q3 real   % joint angles
syms A B C real      % link lengths

T = @(theta, trans) ...
    [c(theta),   -s(theta),    0,   trans*c(theta);
     s(theta),    c(theta),    0,   trans*s(theta);
            0,           0,    1,                0;
            0,           0,    0,                1];
          
TT = simplify(T(q1,A) * T(q2,B) * T(q3,C));
%subs(TT, [q1, q2, q3], [pi/2, -pi/2, -pi/2])

% answer 2
syms q1_dot q2_dot q3_dot real   % joint velocities

J = jacobian(TT(1:3,4), [q1, q2, q3]);
J = [J; zeros(2,3); ones(1,3)];
                                 
tip_vel = simplify(J * [q1_dot; q2_dot; q3_dot]);

% answer 3
TT = double(subs(TT, [A, B, C, q1, q2, q3], [0.8, 0.4, 0.2, pi/4, pi/12, -pi/6]));
tip_vel = double(subs(tip_vel, ...
               [A,     B,   C,   q1,    q2,    q3, q1_dot, q2_dot, q3_dot], ...
               [0.8, 0.4, 0.2, pi/4, pi/12, -pi/6,   pi/6,   pi/6,   pi/6]));

% answer 4
syms ml g real  % ml = load, g = acc. due to gravity
%assume([ml, g], 'real')

torques = simplify(J' * [0; ml*g; 0; 0; 0; 0]);
torques = double(subs(torques, [A, B, C, q1, q2, q3, ml, g], [0.8, 0.4, 0.2, pi/4, pi/12, -pi/6, 1.5, 9.8]));

% answer 5
A = 0.8; B = 0.4; C = 0.2;                     % link lengths
ma = 2; mb = 1; mc = 0.5; ml = 1.5;            % masses
% q1 = pi/4; q2 = pi/12; q3 = -pi/6;             % joint angles
% q1_dot = pi/6; q2_dot = pi/6; q3_dot = pi/6;   % joint angle velocities

% part (a)
trans.Ma = simplify(T(q1,A/2));                     % 4x4 matrix for MassA
trans.Mb = simplify(T(q1,A) * T(q2,B/2));           % 4x4 matrix for MassB
trans.Mc = simplify(T(q1,A) * T(q2,B) * T(q3,C/2)); % 4x4 matrix for MassC
trans.Ml = simplify(T(q1,A) * T(q2,B) * T(q3,C));   % 4x4 matrix for Load


vel.Ma   = simplify(jacobian(trans.Ma(1:3,4), [q1, q2, q3]) * [q1_dot; q2_dot; q3_dot]);
vel.Mb   = simplify(jacobian(trans.Mb(1:3,4), [q1, q2, q3]) * [q1_dot; q2_dot; q3_dot]);                                                             
vel.Mc   = simplify(jacobian(trans.Mc(1:3,4), [q1, q2, q3]) * [q1_dot; q2_dot; q3_dot]);
vel.tip  = subs(simplify(J(1:3,:) * [q1_dot; q2_dot; q3_dot]));   % from answer 2

% calculating PE and KE
PE.Ma = ma * 9.8 * trans.Ma(2,4);
PE.Mb = mb * 9.8 * trans.Mb(2,4);
PE.Mc = mc * 9.8 * trans.Mc(2,4);
PE.Ml = ml * 9.8 * trans.Ml(2,4);

KE.Ma = simplify(0.5 * ma * dot(vel.Ma, vel.Ma));
KE.Mb = simplify(0.5 * mb * dot(vel.Mb, vel.Mb));
KE.Mc = simplify(0.5 * mc * dot(vel.Mc, vel.Mc));
KE.Ml = simplify(0.5 * ml * dot(vel.tip, vel.tip));

% part (b)
L = simplify((KE.Ma + KE.Mb + KE.Mc + KE.Ml) - (PE.Ma + PE.Mb + PE.Mc + PE.Ml));

% part (c)
syms dq1 dq2 dq3 real  % workaround for differentiating w.r.t time (trick picked up from sample code suggestions)
tau.l1 = jacobian(diff(L, q1_dot), [q1, q2, q3]) * [dq1; dq2; dq3] - diff(L, q1);
tau.l2 = jacobian(diff(L, q2_dot), [q1, q2, q3]) * [dq1; dq2; dq3] - diff(L, q2);
tau.l3 = jacobian(diff(L, q3_dot), [q1, q2, q3]) * [dq1; dq2; dq3] - diff(L, q3);

% DH = [0, q1, A, 0;
%       0, q2, B, 0;
%       0, q3, C, 0];
% 
% DTH = eye(4);
% for i = 1:size(DH,1)
%     DTH = DTH * dh2mat(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
% end
% DTH = simplify(DTH);
% 
% disp(TT)
% disp(DTH)
% 
% % function to calculate DH matrix
% function T = dh2mat(d, theta, a, alpha)
% c = @(x) cos(x);
% s = @(x) sin(x);
% 
% T = [c(theta),    -s(theta)*c(alpha),    s(theta)*s(alpha),     a*c(theta);
%      s(theta),     c(theta)*c(alpha),   -c(theta)*s(alpha),     a*s(theta);
%      0,            s(alpha),             c(alpha),              d;
%      0,            0,                    0,                     1];
% 
% T = simplify(T);
% end