close all
clc

% Importing data
file = csvread('/Users/abhishek47kashyap/Downloads/LIDAR_100ms_Wander.csv');
N = length(file);

% (a)
OutOfRange = length(find(file > 4000)) / N;

% (b)
MinRange = length(find(file < 5)) / N;

% (c)
actual = file .* (file >= 5 & file <= 4000);
actual = actual(actual ~= 0);
mu     = mean(actual);
sd     = std(actual);

% (d) - i, ii, iii
obj       = 0;      % no. of objects detected
obj_vel   = [];     % in cm/ms
obj_time  = [];     % time for which each object was in range
dt        = 100;    % readings are 100ms apart

for i = 1:(N - 1)
    vel   = (file(i+1) - file(i)) / dt;
    j     = i + 1;
    timer = 100;
    
    while j < N
        vel2 = (file(j+1) - file(j)) / dt;
        if abs(vel2 - vel) <= 0.20   % if velocity differences are not too high
            timer = timer + 100;
        else
            break
        end
        j   = j + 1;
        vel = vel2;
    end
    
    if timer >= 9600
        obj      = obj + 1;
        obj_vel  = [obj_vel ((file(j) - file(i)) / 100)];   % in cm/ms
        obj_time = [obj_time ((j - i) * 100)];
    end
    i = j;
end

Vmax = max(obj_vel);    % (d) - ii
% assuming distances were recorded from time = 0ms
P_object = sum(obj_time) / ((N - 1) * 100);  % (d) - iii