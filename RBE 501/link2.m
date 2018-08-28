close all
clear all
clc

syms t1 t2 l1 l2
DH = [0, t1, l1, 0;
      0, t2, l2, 0];
  
TT = eye(4);
for i = 1:size(DH,1)
    T   = dh2mat(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
    TT  = TT * T;
end

% t1 = pi/2;
% t2 = -pi/2;
% TT = subs(TT);
simplify(TT(1:3, 4))