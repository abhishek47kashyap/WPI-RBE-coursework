close all
clc

file = csvread('/Users/abhishek47kashyap/Downloads/LIDAR100ms_constant_vel.csv');

dt     = 0.1;            % time-step
A      = [1 dt; 0 1];    % state transition matrix
D      = [dt; 1];        % D vector
C      = [1 0];          % measurement vector (position only)
Qdt    = 0.1;            % process noise variance
noise  = 16;             % measurement noise variance

cov_init    = diag([1000 100]);     % initial covariance
state_init  = [2500; 0];            % initial state

% Part (a)
time = (1:length(file)) / 10;   % time in seconds

[pos, pos_sd, vel, vel_sd] = KF(file, state_init, cov_init, Qdt, noise, A, C, D);

figure,
plot(time, file, time, pos, time, pos_sd,'LineWidth', 1), title('1a - Position')
grid on, xlabel('Time (s)'), ylabel('Position (cm)'), ylim([-500 3000])
legend('Sensor readings', 'Kalman position',...
    'Position standard deviation', 'Location', 'northeast')
figure,
plot(time, vel, time, vel_sd, 'LineWidth', 1), title('1a - Velocity')
grid on, xlabel('Time (s)'), ylabel('Velcoity (cm/s)')
legend('Kalman velocity', 'Velocity standard deviation', 'Location', 'northeast')

% part (b)
objects = [file(46:70); file(133:155); file(177:208)];
time    = (1:length(objects));  % time in seconds

[pos, pos_sd, vel, vel_sd] = KF(objects, state_init, cov_init, Qdt, noise, A, C, D);

figure,
plot(time, objects, time, pos, time, pos_sd,'LineWidth', 1), title('1b - Position')
grid on, xlabel('Time (s)'), ylabel('Position (cm)'), ylim([-500 3000])
legend('Sensor readings', 'Kalman position',...
    'Position standard deviation', 'Location', 'northeast')
figure,
plot(time, vel, time, vel_sd, 'LineWidth', 1), title('1b - Velocity')
grid on, xlabel('Time (s)'), ylabel('Velcoity (cm/s)')
legend('Kalman velocity', 'Velocity standard deviation', 'Location', 'northeast')

% part (c)
no_obj = file; % for manually removing objects
time   = (1:length(no_obj));  % time in seconds

no_obj(56:60)   = NaN;
no_obj(143:145) = NaN;
no_obj(187:198) = NaN;

[pos, pos_sd, vel, vel_sd] = KF(no_obj, state_init, cov_init, Qdt, noise, A, C, D);

figure,
plot(time, no_obj, 'x', time, pos, time, pos_sd,'LineWidth', 1.2)
title('1c - Position (objects removed)')
grid on, xlabel('Time (s)'), ylabel('Position (cm)'), ylim([-500 3000])
legend('Sensor readings', 'Kalman position',...
    'Position standard deviation', 'Location', 'northeast')
figure,
plot(time, vel, time, vel_sd, 'LineWidth', 1), title('1c - Velocity (objects removed)')
grid on, xlabel('Time (s)'), ylabel('Velcoity (cm/s)')
legend('Kalman velocity', 'Velocity standard deviation', 'Location', 'northeast')


function [pos, pos_sd, vel, vel_sd] = KF(data, state_prev, cov_prev, Q, R, A, H, D)

% Q - process noise variance
% R - measurement noise variance
% A - state transition matrix
% H - measurement vector
% D - D vector
%
% Prediction equations
%    x(k) = Ax(k-1) + Bu(k-1) + De
%    P(k) = (A * P(k-1)* A') + (D * Q * D')
%
% Update equations
%    K    = P * H' * (HPH' + R)^(-1)
%    x(k) = x(k) + K(z - Hx(k))
%    P(k) = (I - KH) * P(k)
%
% where e - zero mean white Gaussian noise with variance Q
%       z - sensor reading
%       P - covariance matrix
%       K - Kalman gain

pos     = zeros(size(data));    % positions
pos_sd  = zeros(size(data));    % SD of positions
vel     = zeros(size(data));    % velocities
vel_sd  = zeros(size(data));    % SD of velocities

for i = 1:length(data)
    % prediction
    state_next  = (A * state_prev) + (D .* (sqrt(Q) * randn(2, 1)));
    cov_next    = (A * cov_prev * A') + (D * Q * D');
    
    % update
    if ~isnan(data(i))
        K           = cov_next * H' * (H * cov_next * H' + R)^(-1);
        state_next  = state_next + (K * (data(i) - (H * state_next)));
        cov_next    = (eye(size(K * H)) - (K * H)) * cov_next;
    end
    
    % recording positions and velocities for plotting
    pos(i)     = state_next(1);
    pos_sd(i)  = sqrt(cov_next(1,1));
    vel(i)     = state_next(2);
    vel_sd(i)  = sqrt(cov_next(2,2));
    
    % moving on to the next cycle
    state_prev = state_next;
    cov_prev   = cov_next;
end

end