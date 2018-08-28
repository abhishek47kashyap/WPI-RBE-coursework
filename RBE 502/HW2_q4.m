clear all
clc

% a = 1/(M + m),
% where M = mass of the cart
%       m is the mass of the bob
% (notation copied from book)

syms X1 X2 X3 X4 u k1 k2 k3 k4 lambda

m = 0.1;    % mass of the bob
l = 1;      % length of the pendulum
g = 10;     % acceleration due to gravity
M = 1;      % mass of the cart
a = 1/(M+m);

x1_dot = X2;
x2_dot = (-m*a*g*(sin(2*X3)/2)+a*l*(X4^2)*(4*m)/3)/(4/3-m*a*(cos(X3)^2)) + ...
         ((4*a/3)*u/(4/3)-m*a*(cos(X3)^2));
x3_dot = X4;
x4_dot = (g*sin(X3)-m*l*a*(X4^2)*(sin(2*X3)/2))/(4*l/3-m*l*a*(cos(X3)^2))-...
         (a*cos(X3)*u)/((4*l/3)-m*l*a*(cos(X3)^2));

X_dot = [x1_dot;
         x2_dot;
         x3_dot;
         x4_dot];

A = jacobian(X_dot,[X1,X2,X3,X4]);
A = subs(A,[X1 X2 X3 X4],[0 0 0 0]);

B = jacobian(X_dot,u);
B = subs(B,[X1 X2 X3 X4 u],[0 0 0 0 0]);

k = [k1 k2 k3 k4];
M = A - (B * k);
M = simplify(det(M - lambda*eye(4)));

eq1 = subs(M,lambda,-1)==0;
eq2 = subs(M,lambda,-2)==0;
eq3 = subs(M,lambda,-1+1i)==0;
eq4 = subs(M,lambda,-1-1i)==0;

k = solve([eq1 eq2 eq3 eq4], [k1 k2 k3 k4]);

k1 = double(k.k1);
k2 = double(k.k2);
k3 = double(k.k3);
k4 = double(k.k4);

k = [k1, k2, k3, k4]