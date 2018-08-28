clear all
clc

A = [0 1; 1 0];
B = [0;1];

syms k1 k2   % gains of the controller
syms lambda  % desired closed loop poles

M = A - (B * [k1 k2]);
N = simplify(det(M) - lambda * eye(length(A)));

eqn1 = subs(N, lambda, -2) == 0;
eqn2 = subs(N, lambda, -3) == 0;

K  = solve([eqn1 eqn2], [k1 k2]);
k1 = double(K.k1);
k2 = double(K.k2);