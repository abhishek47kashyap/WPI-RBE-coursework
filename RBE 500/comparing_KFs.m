clear all
close all
clc

%% Erik's KF implementation

static = csvread('/Users/abhishek47kashyap/Downloads/LIDAR100ms_static.csv');
% There are no objects for the first 300 data point of the sensor  data,
% thus we can use this data to calculate the sensor noise 
noise = std(static(1:300))^2;

% Calculate the mean of the Wall position
m = mean(static(static>1950));
format short
dt = .1; % time Step 
times = 0:dt:(length(static)*dt); %The time vector for each state 
state  = [2000;0]; %State Vector [x;xdot] 
sig = [1000 0;0 100]; %initial state variance
A = [1 dt;0 1]; % A matrix 
D = [dt;1]; % The D vector 
qdt = 10; %(cm/s)^2
C = [1 0]; % as the measurement is just the position value 
Q = noise; % measurement variance 
L = 1; 

corr = []; 
for i = 1:length(static)
    state(:,i+1) = A*state(:,i); %Update Positon     
    sig(:,:,i+1) = A*sig(:,:,i)*A'+ D*qdt*D'; %q*Bd; % %Update Variance  matrix
    corr(i) = sig(1,2,i+1)/(sig(1,1,i+1)^0.5*sig(2,2,i +1)^0.5); %Calculate correlation     
    t = i;     
    State = state(:,i+1);     
    Covariance = sig(:,:,i+1);     %Variance Plotting     
    z = static(i);
    K = sig(:,:,end)*C'*(C*sig(:,:,end)*C'+L*Q*L')^-1; %Kalman gain     
    state(:,i+1) = state(:,i+1) + K*(z-C*state(:,i+1)); %Updated State     
    sig(:,:,i+1) = (eye(2)-K*C)*sig(:,:,i+1); %Updated Variance
end

%% my KF implementation

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
time = (1:length(file)) / 10;

%% comparing KF outputs

plot(time, pos, time, state(1,1:1577), time, file, 'x', 'LineWidth', 0.75), 
grid on, legend('My KF', 'Erik KF', 'Sensor readings', 'Location', 'eastoutside'),
title('KF position'), xlabel('Time (s)'), ylabel('Position (cm)')

%% KF function (used by my code)

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
    pos_sd(i)  = cov_next(1,1);
    vel(i)     = state_next(2);
    vel_sd(i)  = cov_next(2,2);
    
    state_prev = state_next;
    cov_prev   = cov_next;
end

% time = (1:length(data)) / 10;   % time in seconds

% figure
% subplot(2,2,1), plot(time, pos), title('Position')
% grid on, xlabel('Time (s)'), ylabel('Distance (cm)'), hold on
% subplot(2,2,2), plot(time, pos_sd), title('Position SD')
% grid on, xlabel('Time (s)'), ylabel('Distance (cm)'), hold on
% subplot(2,2,3), plot(time, vel), title('Velocity')
% grid on, xlabel('Time (s)'), ylabel('Velocity (cm/s)'), hold on
% subplot(2,2,4), plot(time, vel_sd), title('Velocity SD')
% grid on, xlabel('Time (s)'), ylabel('Velocity (cm/s)'), hold on

end