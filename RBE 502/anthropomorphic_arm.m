% Dynamic model for anthropomorphic arm

clc
clear all
close all

syms t g ...
     t1(t) t2(t) t3(t) ...          % joint angles
     m1 m2 m3 ...                   % link masses
     l1 l2 l3 ...                   % link lengths
     t1r t2r t3r td1r td2r td3r ... % replacement variables for subs()
     real
syms q1 q2 q3 a1 a2 a3 real

x1 = 0;
y1 = 0;
z1 = l1;
X1 = [x1; y1; z1];

x2 = l2*cos(t2)*cos(t1);
y2 = l2*cos(t2)*sin(t1);
z2 = l1 + (l2*sin(t2));
X2 = [x2; y2; z2];

x3 = ((l2*cos(t2)) + (l3*cos(t2+t3)))*cos(t1);
y3 = ((l2*cos(t2)) + (l3*cos(t2+t3)))*sin(t1);
z3 = l1 + (l2*sin(t2)) + (l3*sin(t2+t3));
X3 = [x3; y3; z3];

XD1 = diff(X1,t);
XD2 = diff(X2,t);
XD3 = diff(X3,t);

k1 = m1 * (XD1' * XD1) * 0.5;
k2 = m2 * (XD2' * XD2) * 0.5;
k3 = m3 * (XD3' * XD3) * 0.5;
KE = simplify(k1 + k2 + k3);

p1 = m1 * g * z1;
p2 = m2 * g * z2;
p3 = m3 * g * z3;
PE = simplify(p1 + p2 + p3);

L = simplify(KE - PE);

aP    = { t1,  t2,  t3, diff(t1, t), diff(t2, t), diff(t3, t)};
sP    = {t1r, t2r, t3r,        td1r,        td2r,        td3r};
torq1 = diff(subs( diff(subs( L, aP, sP), td1r),sP, aP)) - subs( diff(subs( L, aP, sP), t1r),sP, aP);
torq2 = diff(subs( diff(subs( L, aP, sP), td2r),sP, aP)) - subs( diff(subs( L, aP, sP), t2r),sP, aP);
torq3 = diff(subs( diff(subs( L, aP, sP), td3r),sP, aP)) - subs( diff(subs( L, aP, sP), t3r),sP, aP);

tau = simplify([torq1; torq2; torq3]);
tau = tau(t);

M11 = (tau(1) - subs(tau(1),diff(t1, t, t),0)) /diff(t1, t, t);
M12 = (tau(1) - subs(tau(1),diff(t2, t, t),0)) /diff(t2, t, t);
M13 = (tau(1) - subs(tau(1),diff(t3, t, t),0)) /diff(t3, t, t);
M21 = (tau(2) - subs(tau(2),diff(t1, t, t),0)) /diff(t1, t, t);
M22 = (tau(2) - subs(tau(2),diff(t2, t, t),0)) /diff(t2, t, t);
M23 = (tau(2) - subs(tau(2),diff(t3, t, t),0)) /diff(t3, t, t);
M31 = (tau(3) - subs(tau(3),diff(t1, t, t),0)) /diff(t1, t, t);
M32 = (tau(3) - subs(tau(3),diff(t2, t, t),0)) /diff(t2, t, t);
M33 = (tau(3) - subs(tau(3),diff(t3, t, t),0)) /diff(t3, t, t);

M = simplify([M11 M12 M13; M21 M22 M23; M31 M32 M33]);
M = M(t);

G = simplify(subs(tau, {diff(t1, t, t), diff(t2, t, t), diff(t3, t, t),...
                   diff(t1, t), diff(t2, t), diff(t3, t)}, {0,0,0,0,0,0}));

C = simplify(subs(tau, [diff(t1,t,t), diff(t2,t,t), diff(t3,t,t), g], [0 0 0 0]));

M = subs(M, [t1, t2, t3, l1, l2, l3, diff(t1,t), diff(t2,t), diff(t3,t)], ...
            [q1, q2, q3, a1, a2, a3, diff(q1,t), diff(q2,t), diff(q3,t)]);
        
C = subs(C, [t1, t2, t3, l1, l2, l3, diff(t1,t), diff(t2,t), diff(t3,t)], ...
            [q1, q2, q3, a1, a2, a3, diff(q1,t), diff(q2,t), diff(q3,t)]);
        
G = subs(G, [t1, t2, t3, l1, l2, l3], ...
            [q1, q2, q3, a1, a2, a3]);