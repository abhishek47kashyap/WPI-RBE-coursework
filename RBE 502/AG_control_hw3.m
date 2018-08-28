%%
clc 
clear all

syms m1 lc1 m2 l1 lc2 q2 I1 I2 m2 lc2 I2 q q1 q2_dot q1_dot 

M11 = m1*(lc1^2) + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2)) + I1 + I2 ;
M12 = m2*((lc2^2) + l1*lc2*cos(q2)) + I2;
M21 = m2*(lc2^2 + l1*lc2*cos(q2)) + I2;
M22 = m2*(lc2)^2 + I2;
M = [M11 M12;
    M21 M22];

M = diff(M)*q2_dot;
disp('Q-3');
fprintf('The M dot matrix differentiated will be: \n');
disp(M);
C11 = -m2*l1*lc2*sin(q2)*q2_dot;
C12 = -m2*l1*lc2*sin(q2)*(q1_dot + q2_dot);
C21 = m2*l1*lc2*sin(q2)*q1_dot;
C22 = 0;
C = [C11 C12;
    C21 C22];
fprintf('The C matrix will be: \n');
disp(C);
lhs = simplify(M/2 - C);
fprintf('Equation under consideration (lhs): \n');
disp(lhs);
rhs = -lhs.'; %A square matrix, A, is skew-symmetric if it is equal to the negation of its nonconjugate transpose, A = -A.'.
fprintf('Negation of conjugate of given matrix (rhs): \n');
disp(rhs);
fprintf('Since lhs = rhs, hence given equation is skew symmetric\n');
%%
clc 
clear all

syms l1 l2... Distance of the center
    a1 a2... Link Lengths
    I1 I2... Moment of Inertia
    theta1(t) theta2(t)... Theta's are function of time
    m1 m2... Masses of links
    t1r t2r... Replacement variables for subs 
    td1r td2r... Replacement variable for subs
    t g...                % Time and Gravity
    theta_d dtheta_d ddtheta_d q1_d q2_d q1_d_dot q2_d_dot... %desired position and velocity
    e e_dot...
    Kp Kv...
    q1 q2 q1_dot q2_dot q1_ddot q2_ddot;
%4a
th1  = theta1;th2  = theta2;
x1 = l1 * cos(th1);
y1 = l1 * sin(th1);
X1 = [x1; y1]
x2 = (a1*cos(th1)) + (l2*cos(th1+th2));
y2 = (a1*sin(th1)) + (l2*sin(th1+th2));
X2 = [x2; y2]
XD1 = diff(X1,t)
XD2 = diff(X2,t)


v1s = simplify(XD1.' * XD1)
v2s = simplify(XD2.' * XD2)
k1 = (m1 * v1s * 0.5) + (I1 * (diff(th1, t))^2 * 0.5);
k2 = (m2 * v2s * 0.5) + (I2 * (diff(th1, t) + diff(th2, t))^2 * 0.5);
KE = simplify(k1 + k2)

p1 = m1 * g * y1;
p2 = m2 * g * y2;
PE = simplify(p1 + p2)

L = KE - PE;
L = simplify(L)

aP = {theta1, theta2, diff(theta1(t), t), diff(theta2(t), t)};
sP = {   t1r,    t2r,               td1r,               td2r};
torq1 = diff(subs( diff(subs( L, aP, sP), td1r),sP, aP),t) - subs( diff(subs( L, aP, sP), t1r),sP, aP);
torq2 = diff(subs( diff(subs( L, aP, sP), td2r),sP, aP),t) - subs( diff(subs( L, aP, sP), t2r),sP, aP);
torques = [simplify(torq1); simplify(torq2)]


torques = torques(t); % convert symfunct to sym
M11 = simplify(torques(1) - subs(torques(1),diff(theta1(t), t, t),0)) /diff(theta1(t), t, t);
M12 = simplify(torques(1) - subs(torques(1),diff(theta2(t), t, t),0)) /diff(theta2(t), t, t);
M21 = simplify(torques(2) - subs(torques(2),diff(theta1(t), t, t),0)) /diff(theta1(t), t, t);
M22 = simplify(torques(2) - subs(torques(2),diff(theta2(t), t, t),0)) /diff(theta2(t), t, t);
M = [M11 M12; M21 M22]
G = subs(torques, {diff(theta1(t), t, t), diff(theta2(t), t, t), diff(theta1(t), t), diff(theta2(t), t)}, {0,0,0,0})
C1 = simplify(torques(1) - (M(1,:) * [diff(theta1(t), t, t) diff(theta2(t), t, t)].' + G(1)));
C2 = simplify(torques(2) - (M(2,:) * [diff(theta1(t), t, t) diff(theta2(t), t, t)].' + G(2)));
C11 = simplify(C1 - subs(C1,diff(theta1(t), t),0)) / diff(theta1(t), t); 
C12 = simplify(C1 - subs(C1,diff(theta2(t), t),0)) / diff(theta2(t), t);
C21 = simplify(C2 - subs(C2,diff(theta1(t), t),0)) / diff(theta1(t), t); 
C22 = simplify(C2 - subs(C2,diff(theta2(t), t),0)) / diff(theta2(t), t);
C = [C11 C12; C21 C22]

M = subs(M,{diff(theta1(t), t, t) , diff(theta2(t), t, t), diff(theta1(t), t),diff(theta2(t), t),theta1(t),theta2(t)},{q1_ddot,q2_ddot,q1_dot,q2_dot,q1,q2})
C = subs(C,{diff(theta1(t), t, t) , diff(theta2(t), t, t), diff(theta1(t), t),diff(theta2(t), t),theta1(t),theta2(t)},{q1_ddot,q2_ddot,q1_dot,q2_dot,q1,q2})

%4b
theta = [q1; q2]
theta_dot = [q1_dot ; q2_dot]
theta_d = [q1_d; q2_d]
dtheta_d = [q1_d_dot;q2_d_dot]
e = theta_d - theta
e_dot = dtheta_d - theta_dot
fprintf('Since set point control so thata_d is constant which means:  \n')
e_dot = -theta_dot
u2 = Kp*e + Kv*e_dot

M_inv = inv(M)
M_invC = M_inv * C;

%State Space
%4d
state_space = [-e_dot; ( M_inv*u2) - (M_invC*theta_dot)]
%In copy display pictures for rest of the parts

