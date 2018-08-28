%% Question 1: Monte Carlo Sampling (Particle Filter Propagation)

clear all
close all
clc

cases = ['A', 'B', 'C', 'D', 'E'];
N = 1000;               % number of points
t = 1;                  % time-step = 1 sec
L = 100;                % length of bicycle = 100 cm
a = [25 -25 25 75 75];  % steering angles
v = [20 20 90 10 90];   % velocities
a_sig = sqrt(36);       % steering angle standard deviation
v_sig = sqrt(v * 5);    % velocities standard deviation

init.x = 0; init.y = 0; init.theta = 0; % initial states

x = zeros(1, N);        % x coordinates
y = zeros(1, N);        % y coordinates

for cs = 1:length(cases)
    for i = 1:N
        % introducing noise
        vel = normrnd(v(cs), v_sig(cs));
        ang = normrnd(a(cs), a_sig);
        
        while (vel < 0 || vel > 100)    % 0 <= vel <= 100
            vel = normrnd(v(cs), v_sig(cs));
        end
        
        while (ang > 80 || ang < -80)   % |ang| <= 80
            ang = normrnd(a(cs), a_sig);
        end
        
        % update equations
        x(i) = init.x + (t * vel * cosd(ang + init.theta));
        y(i) = init.y + (t * vel * sind(ang + init.theta));
    end
    
    figure, scatter(x, y), title(['Case ' cases(cs)])
    grid on, xlabel('X in cm'), ylabel('Y in cm')
end