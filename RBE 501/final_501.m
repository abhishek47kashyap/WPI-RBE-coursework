clear all
close all
clc

% defining function handles
C = @(x) cos(x);
S = @(x) sin(x);
dh2mat = @(D, theta, A, alpha) ...
         [C(theta),    -S(theta)*C(alpha),    S(theta)*S(alpha),     A*C(theta);
          S(theta),     C(theta)*C(alpha),   -C(theta)*S(alpha),     A*S(theta);
          0,            S(alpha),             C(alpha),              D;
          0,            0,                    0,                     1];
      
% defining symbolic variables
syms a b c d real                 % link lengths
syms q1 q2 q3 real                % joint variables
syms q1_dot q2_dot q3_dot real    % joint velocities
syms f1 f2 f3 f4 f5 f6 real       % forces
syms mt g real                    % mt = load, g = acc. due to gravity

% DH table
DH = [a + c,    q1 + pi/2,    b,       0;
          0,           q2,    0,   -pi/2;
     d + q3,            0,    0,       0];

% computing frame transformations
TT = eye(4);
for i = 1:size(DH,1)
    T  = simplify(dh2mat(DH(i,1), DH(i,2), DH(i,3), DH(i,4)));
    TT = simplify(TT * T);
end

% computing velocity kinematics
J = simplify(jacobian(TT(1:3,end), [q1, q2, q3]));
J = [J; zeros(2,3); 1, 1, 0];

tip_vel = simplify(J * [q1_dot; q2_dot; q3_dot]);

% Lagrangian
PE = simplify(mt * g * TT(3,4));                            % potential energy
KE = simplify(0.5 * mt * dot(tip_vel(1:3), tip_vel(1:3)));  % kinetic energy
L  = simplify(KE - PE);

syms q1_ddot q2_ddot q3_ddot real
tau1 = simplify(diff(diff(L, q1_dot), q1_dot) * q1_ddot - diff(L, q1));
tau2 = simplify(diff(diff(L, q2_dot), q2_dot) * q2_ddot - diff(L, q2));
tau3 = simplify(diff(diff(L, q3_dot), q3_dot) * q3_ddot - diff(L, q3));

% Force propagation
tau = simplify(J' * [f1; f2; f3; f4; f5; f6]);