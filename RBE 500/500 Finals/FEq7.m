close all
clc

% part (a)
digits(10)
a = [3.351797405; 1.599123929; -9.069082468];

p = acos(a(1) / norm(a));   % pitch angle
r = acos(a(2) / norm(a));   % roll angle
y = 0;                      % yaw angle

c = @(x) cos(x);
s = @(x) sin(x);

dcm = @(p,r,y) ...
       [c(p)*c(y),                  c(p)*s(y),                   -s(p);
        s(r)*s(p)*c(y)-c(r)*s(y),   s(r)*s(p)*s(y)+c(r)*c(y),    s(r)*c(p);
        c(r)*s(p)*c(y)+s(r)*s(y),   c(r)*s(p)*s(y)-s(r)*c(y),    c(r)*c(p)];

Cbn = dcm(p,r,y);

% part (b)
Cnb = Cbn';

% part (c)
quat = @(p,r,y) ...
        [c(y)*c(p)*c(r)+s(y)*s(p)*s(r);
         c(y)*c(p)*s(r)-s(y)*s(p)*c(r);
         c(y)*s(p)*c(r)+s(y)*c(p)*s(r);
         s(y)*c(p)*c(r)-c(y)*s(p)*s(r)];

 Qbn = quat(p/2, r/2, y/2);
 Qnb = Qbn .* [1; -1; -1; -1];
 
 % part (d)
 quat2dcm = @(q0, q1, q2, q3) ...
             [q0^2+q1^2-q2^2-q3^2,  2*(q1*q2+q0*q3),      2*(q1*q3-q0*q2);
              2*(q1*q2-q0*q3),      q0^2-q1^2+q2^2-q3^2,  2*(q2*q3+q0*q1);
              2*(q1*q3+q0*q2),      2*(q2*q3-q0*q1),      q0^2-q1^2-q2^2+q3^2];
 
Cnb2 = quat2dcm(Qnb(1), Qnb(2), Qnb(3), Qnb(4));

% printing out results
disp('Part (a)')
disp('Direction cosine matrix - navigation frame to body frame:')
disp(Cbn)
disp('Part (b)')
disp('Direction cosine matrix - body frame to navigation frame:')
disp(Cnb)
disp('Part (c)')
disp('Quaternion - navigation frame to body frame:')
disp(Qbn)
disp('Quaternion - body frame to navigation frame:')
disp(Qnb)
disp('Part (d)')
disp('Direction cosine matrix - body frame to navigation frame:')
disp(Cnb2)