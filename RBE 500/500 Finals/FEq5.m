close all
clc

% importing files
master             = readtable('/Users/abhishek47kashyap/Downloads/Master.csv');
master(:, [3,5])   = [];
master             = table2array(master);

rover1             = readtable('/Users/abhishek47kashyap/Downloads/Rover_1.csv');
rover1(:, [3,5])   = [];
rover1             = table2array(rover1);

rover2             = readtable('/Users/abhishek47kashyap/Downloads/Rover_2.csv');
rover2(:, [3,5])   = [];
rover2             = table2array(rover2);

% part (a) - converting to degrees and calculating average locations
temp          = (master(:,2) - (mod(master(:,2), 100))) / 100;
master(:,2)   = temp + (mod(master(:,2), 100) / 60);
temp          = (master(:,3) - (mod(master(:,3), 100))) / 100;
master(:,3)   = temp + (mod(master(:,3), 100) / 60);

temp          = (rover1(:,2) - (mod(rover1(:,2), 100))) / 100;
rover1(:,2)   = temp + (mod(rover1(:,2), 100) / 60);
temp          = (rover1(:,3) - (mod(rover1(:,3), 100))) / 100;
rover1(:,3)   = temp + (mod(rover1(:,3), 100) / 60);

temp          = (rover2(:,2) - (mod(rover2(:,2), 100))) / 100;
rover2(:,2)   = temp + (mod(rover2(:,2), 100) / 60);
temp          = (rover2(:,3) - (mod(rover2(:,3), 100))) / 100;
rover2(:,3)   = temp + (mod(rover2(:,3), 100) / 60);

clear temp

master_location = [mean(master(:,2)), mean(master(:,3))];
rover1_location = [mean(rover1(:,2)), mean(rover1(:,3))];
rover2_location = [mean(rover2(:,2)), mean(rover2(:,3))];

% part (b) - calculating errors, converting them to meters,
%            and computing their standard deviations
master_error = master(:, 2:3) - master_location;
rover1_error = rover1(:, 2:3) - rover1_location;
rover2_error = rover2(:, 2:3) - rover2_location;

master_error = master_error * [111073.94 0; 0 82804.80];
rover1_error = rover1_error * [111073.94 0; 0 82804.93];
rover2_error = rover2_error * [111073.94 0; 0 82805.06];

master_std = std(master_error);
rover1_std = std(rover1_error);
rover2_std = std(rover2_error);

% part (c)

R1_dis = norm((master_location - rover1_location) .* [111073.94 82804.80]);
R2_dis = norm((master_location - rover2_location) .* [111073.94 82804.80]);

% part (d) - calculating updated standard deviations
r1_error = zeros(size(rover1_error));
r2_error = zeros(size(rover2_error));
r1_count = 0;
r2_count = 0;

for i = 1:length(master)
    % correcting Rover 1
    for j = 1:length(rover1)
        if master(i,1) == rover1(j,1)
            r1_count = r1_count + 1;
            r1_error(r1_count,:) = rover1_error(j) - master_error(i);            
        end
    end
    
    % correcting Rover 2
    for j = 1:length(rover2)
        if master(i,1) == rover2(j,1)
            r2_count = r2_count + 1;
            r2_error(r2_count,:) = rover2_error(j) - master_error(i);            
        end
    end
end

r1_error     = r1_error(1:r1_count,:);
r2_error     = r2_error(1:r2_count,:);
rover1_std2  = std(r1_error);
rover2_std2  = std(r2_error);

% printing out results
sprintf(['Part (a)\nAverage locations (in degrees):\n', ...
    'Master: Latitude = %f, Longitude = %f\n', ...
    'Rover1: Latitude = %f, Longitude = %f\n', ...
    'Rover2: Latitude = %f, Longitude = %f\n\n', ...
    'Part (b)\nStandard deviation of the errors (in meters):\n', ...
    'Master: Latitute = %f, Longitude = %f\n', ...
    'Rover1: Latitude = %f, Longitude = %f\n', ...
    'Rover2: Latitude = %f, Longitude = %f\n\n', ...
    'Part (c)\nDistance from the master (in meters):\n', ...
    'Rover1: %f, Rover2: %f\n\n', ...
    'Part (d)\nUpdated standard deviations of the errors', ...
    '(in meters):\nRover1: Latitude = %f, Longitude = %f\n', ...
    'Rover2: Latitude = %f, Longitude = %f'], ...
    master_location(1), master_location(2), rover1_location(1), ...
    rover1_location(2), rover2_location(1), rover2_location(2), ...
    master_std(1), master_std(2), rover1_std(1), rover1_std(2), ...
    rover2_std(1), rover2_std(2), R1_dis, R2_dis, ...
    rover1_std2(1), rover1_std2(2), rover2_std2(1), rover2_std2(2))