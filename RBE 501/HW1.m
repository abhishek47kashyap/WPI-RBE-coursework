clc
clear all
close all

% syms t1 t2 t3 t4 t5 t6    % tn = theta_n
% t1 = 0; t2 = 0 - pi/2; t3 = 0; t4 = 0; t5 = 0 - pi; t6 = 0;
t1 = -pi/4; t2 = pi/6; t3 = -pi/6; t4 = -pi/6; t5 = -pi/4; t6 = pi;

X = zeros(1,6); Y = zeros(1,6); Z = zeros(1,6);  % for 3-D plotting

% DH = DH parameters table = [d, theta, a, alpha]
DH = [290,   t1,               0,     -pi/2;
        0,   t2 - pi/2,        0,      0;
      270,   t3,               0,     -pi/2;
      70,    t4,             134,     -pi/2;
      0,     t5 - pi,        168,      pi/2;
      0,     t6,              72,      0];

T_all = eye(4); % Transformation from 1 to 6

for i = 1:length(DH)
    T      = dh2mat(DH(i,2), DH(i,1), DH(i,3), DH(i,4));
    X(i)   = T(1,4);
    Y(i)   = T(2,4);
    Z(i)   = T(3,4);
    T_all  = T_all * T;
end

disp(T_all)
% plot3(X, Y, Z, '-bo', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', [0.8 0 0])
% grid on
% xlabel('X'), ylabel('Y'), zlabel('Z')