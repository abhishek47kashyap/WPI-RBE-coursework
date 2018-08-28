close all
clc

file = csvread('/Users/abhishek47kashyap/Downloads/LIDAR100ms_static.csv');
N    = length(file);

% Part (a)
dt          = 0.1;              % measurements after every 100ms
state_init  = [2000; 0];        % initial state = 20m, 0cm/s
cov_init    = [1000 0; 0 100];  % initial covariance matrix

Q = 10;                 % process noise
R = var(file(1:390));   % measurement noise
A = [1 dt; 0 1];        % state transition matrix
H = [1 0];              % measurement matrix

[pos, pos_sd, vel, vel_sd] = KF(file, state_init, cov_init, Q, R, A, H);

% % Part (b)
% Q = 0.1;   % process noise
% [pos, pos_sd, vel, vel_sd] = KF(file, state_init, cov_init, Q, R, A, H)
% 
% 
% % Part (c) - EM algorithm
% M           = mean(file); % mean of the data
% V           = var(file);  % variance of the data
% V_old       = 0;          % used for tracking changes
% lambda      = 1;          % guess for lambda
% lambda_old  = 0;          % to store previous lambda
% C           = [0.5 0.5];  % arbitrary initial weight of components
% C_old       = [0 0];
% 
% while (max(abs(C - C_old)) > 0.001 * max(C)) || ...
%         (abs(V - V_old) > 0.001 * V) || ...
%         (abs(lambda - lambda_old) > 0.001 * lambda)
%     % Gaussian component
%     P1 = C(1) * exp(-(((file - M).^2) / V)) / sqrt(2 * pi * V);
%     
%     % exponential component
%     P2 = C(2) * lambda * exp(-(lambda * file));
%     
%     % sum of these probabilities
%     total = sum([P1 P2], 2);
%     rs = [P1 P2];
%     for i = 1:N
%         rs(i,:) = rs(i,:) / total(i);   % normalization step
%     end
%     
%     C_old = C;
%     V_old = V;
%     lambda_old = lambda;
%     
%     C = sum(rs) / N; % weights for total component
%     V = sum(rs(:,1) .* (file - M).^2) / sum(rs(:,1)); % new variance
%     lambda = sum(rs(:,2)) / sum(rs(:,2) .* file);
% end
% 
% C
% V
% lambda


function [pos, pos_sd, vel, vel_sd] = KF(data, state_prev, cov_prev, Q, R, A, H)

pos     = zeros(size(data));    % positions
pos_sd  = zeros(size(data));    % SD of positions
vel     = zeros(size(data));    % velocities
vel_sd  = zeros(size(data));    % SD of velocities


for i = 1:length(data)
    % prediction
    state_next  = (A * state_prev) + Q;
    cov_next    = (A * cov_prev * A') + Q;
    
    % update
    K           = cov_next * H' * (H * cov_next * H' + R)^(-1);
    state_next  = state_next + (K * (data(i) - (H * state_next)));
    cov_next    = (eye(size(K * H)) - (K * H)) * cov_next;
    
    % recording positions and velocities for plotting
    pos(i)     = state_next(1);
    pos_sd(i)  = sqrt(cov_next(1,1));
    vel(i)     = state_next(2);
    vel_sd(i)  = sqrt(cov_next(2,2));
    
    state_prev = state_next;
    cov_prev   = cov_next;
end

time = (1:length(data)) / 10;   % time in seconds

figure
subplot(2,2,1), plot(time, pos), title('Position')
grid on, xlabel('Time (s)'), ylabel('Distance (cm)'), hold on
subplot(2,2,2), plot(time, pos_sd), title('Position SD')
grid on, xlabel('Time (s)'), ylabel('Distance (cm)'), hold on
subplot(2,2,3), plot(time, vel), title('Velocity')
grid on, xlabel('Time (s)'), ylabel('Velocity (cm/s)'), hold on
subplot(2,2,4), plot(time, vel_sd), title('Velocity SD')
grid on, xlabel('Time (s)'), ylabel('Velocity (cm/s)'), hold on

end