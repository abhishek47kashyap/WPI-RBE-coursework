clear all
clc

syms a b c d real

A = [-1  1  0  0  0;
      0 -1  1  0  0;
      0  0 -1  0  0;
      0  0  0 -2  1;
      0  0  0  0 -2]; 
  
% A = [-1  1  0  0  0;
%       0 -1  1  0  0;
%       0  0 -1  0  0;
%       0  0  0 -1  1;
%       0  0  0  0 -1]; 
  
B = [0 0;
     0 0;
     a b;
     0 0;
     c d];
 
U  = simplify([B A*B A^2*B A^3*B A^4*B]);
U3 = simplify(rref(U));


U2 = U';

row_indices   = (nchoosek(1:length(U2), 5))';
determinants  = sym('x', [nchoosek(length(U2), 5), 1]);

disp('Calculating determinants... ')
for i = 1:length(determinants)
    determinants(i) = simplify(det(U2(row_indices(:,i),:)));
end

determinants = unique(determinants);
fprintf('Determinants calculated!')

fprintf('\nSolving for "a" ...')
sol = [];

for i = 1:length(determinants)
    sol = [sol; solve(determinants(i) == 0, b)];
end
fprintf('Solutions obtained!\n')

sol2 = unique(sol)