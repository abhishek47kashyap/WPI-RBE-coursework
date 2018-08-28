clear all
clc

syms m1 m2 a1 a2 I1 I2 t1(t) t2(t) g

l1     = a1 / 2;
l2     = a2 / 2;
t1_dot = diff(t1, t);
t2_dot = diff(t2, t);

% calculating PE = mgh
PE1 = m1 * g * l1 * sin(t1);
PE2 = m2 * g * (a1 * sin(t1) + l2 * sin(t1+t2));

% calculating Inertia about origin using Parallel Axis Theorem
I1 = I1 + (m1 * a1^2);
I2 = I2 + (m2 * ((a1 * cos(t1) + l2 * cos(t1+t2))^2 + ...
                 (a1 * sin(t1) + l2 * sin(t1+t2))^2));
             
% calculating KE = 0.5Iw^2
KE1 = 0.5 * I1 * t1_dot^2;
KE2 = 0.5 * I2 * (t1_dot + t2_dot)^2;

% Lagrangian L = KE - PE
L = simplify((KE1 + KE2) - (PE1 + PE2));

syms dm               % dm = dummy symbolic variable
T = sym('T', [2 1]);  % Torque vector

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

clear dm

% General form of the dynamical model: 
% T = M(q)*q_ddot + V(q,q_dot) + G(q)

M = sym('M', [2 2]);

M(1,1) = (T(1) - subs(T(1), diff(t1,t,t), 0)) / diff(t1,t,t);
M(1,2) = (T(1) - subs(T(1), diff(t2,t,t), 0)) / diff(t2,t,t);

M(2,1) = (T(2) - subs(T(2), diff(t1,t,t), 0)) / diff(t1,t,t);
M(2,2) = (T(2) - subs(T(2), diff(t2,t,t), 0)) / diff(t2,t,t);

M = simplify(M);

G = simplify(subs(T, [diff(t1,t,t), diff(t2,t,t), ...
                                 t1_dot, t2_dot], [0 0 0 0]));

V = simplify(subs(T, [diff(t1,t,t), diff(t2,t,t), g], [0 0 0]));