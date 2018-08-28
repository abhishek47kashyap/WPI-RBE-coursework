clear all
clc

syms m1 m2 m3 t1(t) t2(t) t3(t) l1 l2 l3 g real

t1_dot = diff(t1, t);
t2_dot = diff(t2, t);
t3_dot = diff(t3, t);

% calculating PE = mgh
PE1 = m1 * g * l1;
PE2 = m2 * g * (l1 + l2*sin(t2));
PE3 = m3 * g * (l1 + l2*sin(t2) + l3*sin(t2+t3));

% calculating KE = 0.5mv^2
KE1 = 0;
KE2 = 0.5 * m2 * ((t1_dot * l2 * cos(t2))^2 + (t2_dot * l2)^2);
KE3 = 0.5 * m3 * (((t2_dot + t3_dot) * (l2 + l3*cos(t3)))^2 + ...
         (t1_dot * (l2 * cos(t2) + l3 * cos(t2+t3)))^2);

% Lagrangian L = KE - PE
L = simplify((KE1 + KE2 + KE3) - (PE1 + PE2 + PE3));

syms dm               % dm = dummy symbolic variable
T = sym('T', [3 1]);  % Torque vector

dL_dQdot = simplify(diff(subs(L, t1_dot, dm), dm));
dL_dQdot = simplify(subs(dL_dQdot, dm, t1_dot));
dL_dQ    = simplify(diff(subs(L, t1, dm), dm));
dL_dQ    = simplify(subs(dL_dQ, dm, t1));
T(1)     = simplify(diff(dL_dQdot, t) - dL_dQ);

dL_dQdot = simplify(diff(subs(L, t2_dot, dm), dm));
dL_dQdot = simplify(subs(dL_dQdot, dm, t2_dot));
dL_dQ    = simplify(diff(subs(L, t2, dm), dm));
dL_dQ    = simplify(subs(dL_dQ, dm, t2));
T(2)     = simplify(diff(dL_dQdot, t) - dL_dQ);

dL_dQdot = simplify(diff(subs(L, t3_dot, dm), dm));
dL_dQdot = simplify(subs(dL_dQdot, dm, t3_dot));
dL_dQ    = simplify(diff(subs(L, t3, dm), dm));
dL_dQ    = simplify(subs(dL_dQ, dm, t3));
T(3)     = simplify(diff(dL_dQdot, t) - dL_dQ);

clear dm

% General form of the dynamical model: 
% T = M(q)*q_ddot + V(q,q_dot) + G(q)

M = sym('M', [3 3]);

M(1,1) = (T(1) - subs(T(1), diff(t1,t,t), 0)) / diff(t1,t,t);
M(1,2) = (T(1) - subs(T(1), diff(t2,t,t), 0)) / diff(t2,t,t);
M(1,3) = (T(1) - subs(T(1), diff(t3,t,t), 0)) / diff(t3,t,t);

M(2,1) = (T(2) - subs(T(2), diff(t1,t,t), 0)) / diff(t1,t,t);
M(2,2) = (T(2) - subs(T(2), diff(t2,t,t), 0)) / diff(t2,t,t);
M(2,3) = (T(2) - subs(T(2), diff(t3,t,t), 0)) / diff(t3,t,t);

M(3,1) = (T(3) - subs(T(3), diff(t1,t,t), 0)) / diff(t1,t,t);
M(3,2) = (T(3) - subs(T(3), diff(t2,t,t), 0)) / diff(t2,t,t);
M(3,3) = (T(3) - subs(T(3), diff(t3,t,t), 0)) / diff(t3,t,t);

M = simplify(M);

G = subs(T, [diff(t1,t,t), diff(t2,t,t), diff(t3,t,t), ...
                   t1_dot,       t2_dot,      t3_dot], ...
                                       [0 0 0 0 0 0]);

% C    = sym('C', [3 1]);                                   
% C(1) = T(1,:) - M(1,:)*[diff(t1,t,t); diff(t2,t,t); diff(t3,t,t)] - G(1);
% C(2) = T(2,:) - M(2,:)*[diff(t1,t,t); diff(t2,t,t); diff(t3,t,t)] - G(2);
% C(3) = T(3,:) - M(3,:)*[diff(t1,t,t); diff(t2,t,t); diff(t3,t,t)] - G(3);
% 
% C = simplify(C);

V = subs(T, [diff(t1,t,t), diff(t2,t,t), diff(t3,t,t), g], [0 0 0 0]);
V = simplify(V);

% dm     = coeffs(T(1), diff(t1,t,t));
% M(1,1) = dm(end);
% dm     = coeffs(T(1), diff(t2,t,t));
% M(1,2) = dm(end);
% dm     = coeffs(T(1), diff(t3,t,t));
% M(1,3) = dm(end);
% 
% dm     = coeffs(T(2), diff(t1,t,t));
% M(2,1) = dm(end);
% dm     = coeffs(T(2), diff(t2,t,t));
% M(2,2) = dm(end);
% dm     = coeffs(T(2), diff(t3,t,t));
% M(2,3) = dm(end);
% 
% dm     = coeffs(T(3), diff(t1,t,t));
% M(3,1) = dm(end);
% dm     = coeffs(T(3), diff(t2,t,t));
% M(3,2) = dm(end);
% dm     = coeffs(T(3), diff(t3,t,t));
% M(3,3) = dm(end);



% M(1,2) = coeffs(T(1), diff(t2,t,t));
% M(1,3) = coeffs(T(1), diff(t3,t,t));
% M(2,1) = coeffs(T(2), diff(t1,t,t));
% M(2,2) = coeffs(T(2), diff(t2,t,t));
% M(2,3) = coeffs(T(2), diff(t3,t,t));
% M(3,1) = coeffs(T(3), diff(t1,t,t));
% M(3,2) = coeffs(T(3), diff(t2,t,t));
% M(3,3) = coeffs(T(3), diff(t3,t,t));