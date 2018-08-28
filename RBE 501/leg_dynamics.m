clear all
close all
clc
pause('on')
format short g

syms t s f real  % t = thigh length, s = shin length, f = foot length
syms h k a real  % h = hip joint angle, k = knee joint angle, a = ankle joint angle

% computing 4x4 transformation matrices
T = @(theta, trans) ...
    [cos(theta),  -sin(theta),  0,  trans*cos(theta);
     sin(theta),   cos(theta),  0,  trans*sin(theta);
              0,            0,  1,                 0;
              0,            0,  0,                 1];
          
T_knee  = T(h,t);                         % knee transformation
T_ankle = simplify(T_knee * T(k,s));      % ankle transformation
T_toes  = simplify(T_ankle * T(a,f));     % toes transformation

% reading OpenSim angles from angles.xlsx
angles        = xlsread('/Users/abhishek47kashyap/Downloads/angles.xlsx', 'D2:F52');
angles        = repmat(angles, 4, 1);
hip_angles    = deg2rad(angles(:,1) - 90);
knee_angles   = deg2rad(angles(:,2));
ankle_angles  = deg2rad(angles(:,3) + 90);

% Jacobians
J_knee   = [simplify(jacobian(T_knee(1:3,end), [h, k, a])); zeros(2,3); 1, 0, 0];
J_ankle  = [simplify(jacobian(T_ankle(1:3,end), [h, k, a])); zeros(2,3); 1, 1, 0];
J_toes   = [simplify(jacobian(T_toes(1:3,end), [h, k, a])); zeros(2,3); ones(1,3)];

% reading joint torques from OpenSim
hip_torques    = xlsread('/Users/abhishek47kashyap/Downloads/hip_flex.xlsx', 'C2:C52');
knee_torques   = xlsread('/Users/abhishek47kashyap/Downloads/example_torque.xlsx', 'C2:C52');
ankle_torques  = xlsread('/Users/abhishek47kashyap/Downloads/ankle moment.xlsx', 'C2:C52');
torques        = [hip_torques'; knee_torques'; ankle_torques'];

% computing forces
F = zeros(length(J_toes), length(torques));
for i = 1:length(torques)
    J_toes = subs(J_toes, [h, k, a, t, s, f], ...
                  [hip_angles(i), knee_angles(i), ankle_angles(i), ...
                   0.417, 0.45, 0.076]);
    F(:,i) = double(pinv(J_toes') * torques(:,i));
end

% drawing plots
time = 0:50;
%figure(1), plot(time, F(1,:), '-o', time, F(2,:), '-o', time, F(6,:), '-o', 'LineWidth', 2)
%title('Force profiles'), ylabel('Newtons'), xlabel('Time'), grid on
%legend('Hip', 'Knee', 'Ankle', 'Location', 'best')
figure(1), plot(time, F(1,:), '-o'), title('Hip')
figure(2), plot(time, F(2,:), '-o'), title('Knee')
figure(3), plot(time, F(6,:), '-o'), title('Ankle')

% plotting walking model
% figure(4)
% for i = 1:length(angles)
%     knee = subs(T_knee(1:2,end), [t, h,], ...
%                 [0.417, hip_angles(i)]);
%     figure(2), plot([0, knee(1)], [0, knee(2)], '-o', 'LineWidth', 3)
%     axis([-1 1 -1.5 0.5]), hold on
%     
%     ankle = subs(T_ankle(1:2, end), [t, s, h, k], ...
%                  [0.417, 0.45, hip_angles(i), knee_angles(i)]);
%     figure(2), plot([knee(1), ankle(1)], [knee(2), ankle(2)], '-o', 'LineWidth', 3), hold on
%     
%     toes = subs(T_toes(1:2, end), [t, s, f, h, k, a], ...
%                  [0.417, 0.45, 0.076, hip_angles(i), knee_angles(i), ankle_angles(i)]);
%     figure(2), plot([ankle(1), toes(1)], [ankle(2), toes(2)], '-o', 'LineWidth', 3)
%     
%     figure(2), pause(0.01), hold off
% end