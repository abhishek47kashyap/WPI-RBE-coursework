close all
clc

sd = zeros(1, 36);  % Standard deviation
M  = zeros(1, 36);  % Mean
k  = 1;             % Counter variable

for i = 0:10:350
    % Importing data
    file = strcat('/Users/abhishek47kashyap/Downloads/LIDAR_360_degrees/LIDAR_100ms_Degrees_',...
        num2str(i, '%03i'), '.csv');
    data  = csvread(file);
    sd(k) = std(data);
    M(k)  = mean(data);
    k     = k + 1;
end

% Plotting standard deviation vs. range
figure(1), stem(M, sd), grid on
set(gca,'xscale','log')
xlabel('Range (in cm) in log scale')
ylabel('Standard deviation')
ylim([-50 350])
title('Standard deviation vs. Range')

% Plotting map of the room
%figure(2), polarscatter(deg2rad(0:10:350), M, 'filled')
figure(2), polarplot(deg2rad(0:10:360), [M M(1)])
title('Map of the room'), thetatickformat('degrees')
rticklabels({'0', '10m','20m','30m'})

% Trying out Luis' approach
X = cos((0:10:350)*pi/180)'.*M;
Y = sin((0:10:350)*pi/180)'.*M;

figure(3), plot(X,Y), grid on, title('Luis room')
axis([-3000,3000,-3000,3000])
xlabel('X')
ylabel('Y')