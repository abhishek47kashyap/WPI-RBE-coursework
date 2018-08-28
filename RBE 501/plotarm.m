function T = plotarm(q1, q2, q3, q4, q5, q6)

% q1, q2, q3, q4, q5, q6 = joint angles
% T = 4 X 4 transformation matrix

X = zeros(1,6); Y = zeros(1,6); Z = zeros(1,6);  % for 3-D plotting

% DH = DH parameters table = [d, theta, a, alpha]
DH = [290,   q1,               0,     -pi/2;
      270,   q2 - (pi/2),      0,      0;
      0,     q3,               0,     -pi/2;
      70,    q4,             134,     -pi/2;
      0,     q5 - pi,        168,      pi/2;
      0,     q6,              72,      0];

T = eye(4); % Transformation from 1 to 6

for i = 1:length(DH)
    t    = dh2mat(DH(i,2), DH(i,1), DH(i,3), DH(i,4));
    X(i) = t(1,4);
    Y(i) = t(2,4);
    Z(i) = t(3,4);
    T    = T * t;
end

plot3(X, Y, Z, '-bo', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', [0.8 0 0])
hold on, plot3([t(1,4) (t(1,4)+10)], [t(1,4) (t(1,4)+10)], [t(1,4) (t(1,4)+10)])
grid on, xlabel('X'), ylabel('Y'), zlabel('Z')

%smimport('/Users/abhishek47kashyap/Downloads/ABB_cad_model/base')

end