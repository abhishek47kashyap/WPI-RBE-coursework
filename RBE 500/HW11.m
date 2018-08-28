close all
clc

data = csvread('/Users/abhishek47kashyap/Downloads/LIDAR100ms_constant_vel.csv');
time = (1:length(data)) / 10;   % time in seconds

dt   = 0.1;            % time-step
A    = [1 dt; 0 1];    % state transition matrix
D    = [dt; 1];        % D vector
C    = [1 0];          % measurement vector (position only)
qdt  = 10;             % process noise variance
Q    = 16;             % measurement noise variance

cov_init    = diag([1000 100]);     % initial covariance
state_init  = [2500; 0];            % initial state

pos     = zeros(size(data));    % positions
pos_sd  = zeros(size(data));    % SD of positions
vel     = zeros(size(data));    % velocities
vel_sd  = zeros(size(data));    % SD of velocities

cov_prev    = cov_init;
state_prev  = state_init;

% engage outlier detection only AFTER Kalman Filter has
% locked on to the actual position
flag_lock   = 0;    

% Kalman Filter
for i = 1:length(data)
    % prediction
    state_next  = (A * state_prev);
    cov_next    = (A * cov_prev * A') + (D * qdt * D');
    
    % automatic outlier detection
    eta    = data(i) - (C * state_next);    % innovation
    N      = (C * cov_next * C') + Q;       % predicted covariance
    delta  = eta' * N^(-1) * eta;           % outlier indicator
    
    % update
    if flag_lock == 0 || (flag_lock == 1 && delta < 4)
        K           = cov_next * C' * (C * cov_next * C' + Q)^(-1);
        state_next  = state_next + (K * (data(i) - (C * state_next)));
        cov_next    = (eye(size(K * C)) - (K * C)) * cov_next;
    end
    
    % recording positions and velocities for plotting
    pos(i)     = state_next(1);
    pos_sd(i)  = sqrt(cov_next(1,1));
    vel(i)     = state_next(2);
    vel_sd(i)  = sqrt(cov_next(2,2));
    
    % AFTER Kalman Filter has locked on
    if abs(pos(i) - data(i)) <= 10
        flag_lock = 1;
    end
    
    % moving on to the next cycle
    state_prev = state_next;
    cov_prev   = cov_next;
end

figure,
plot(time, data, time, pos, time, pos_sd,'LineWidth', 1), title('Position')
grid on, xlabel('Time (s)'), ylabel('Position (cm)'), ylim([-500 3000])
legend('Sensor readings', 'Kalman position',...
    'Position standard deviation', 'Location', 'best')
figure,
plot(time, vel, time, vel_sd, 'LineWidth', 1), title('Velocity')
grid on, xlabel('Time (s)'), ylabel('Velcoity (cm/s)'), ylim([-120 20])
legend('Kalman velocity', 'Velocity standard deviation', 'Location', 'best')