close all
clc

L = zeros(2,8); % LIDAR means
k = 1;          % counter variable

for i = 0:45:359
    % importing data
    file = strcat('/Users/abhishek47kashyap/Downloads/Lidar_Data_Cleaned', ...
        '/Lidar_', num2str(1), '_', num2str(i, '%03i'), '.csv');
    
    data    = csvread(file);
    data(abs(data - mode(data)) > 3) = [];  % filtering anomalous outliers
    L(1,k)  = mean(data);
    
    file = strcat('/Users/abhishek47kashyap/Downloads/Lidar_Data_Cleaned', ...
        '/Lidar_', num2str(2), '_', num2str(i, '%03i'), '.csv');
    
    data    = csvread(file);
    data(abs(data - mode(data)) > 3) = [];  % filtering anomalous outliers
    L(2,k)  = mean(data);
    
    k = k + 1;
end

R = [cos(pi/2) -sin(pi/2); sin(pi/2) cos(pi/2)];    % rotation matrix

% plotting
X     = cos((0:45:350)*pi/180) .* L(1,:);
Y     = sin((0:45:350)*pi/180) .* L(1,:);
coor  = R * [X; Y];
X     = coor(1,:);
Y     = coor(2,:);
figure, plot([X X(1)], [Y Y(1)], 'r', 'LineWidth', 4), title('Lidar 1')
xlabel('X'), ylabel('Y')

X     = cos((0:45:350)*pi/180) .* L(2,:);
Y     = sin((0:45:350)*pi/180) .* L(2,:);
coor  = R * [X; Y];
X     = coor(1,:);
Y     = coor(2,:);
figure, plot([X X(1)], [Y Y(1)], 'r', 'LineWidth', 4), title('Lidar 2')
xlabel('X'), ylabel('Y')