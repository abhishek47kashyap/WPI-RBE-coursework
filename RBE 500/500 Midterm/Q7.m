% importing data (L_U_xm means LIDAR and Ultrasound readings for x meters)
L_U_1m = '/Users/abhishek47kashyap/Documents/500 Foundations/Midterm/Q7_L_U_1m';
L_U_2m = '/Users/abhishek47kashyap/Documents/500 Foundations/Midterm/Q7_L_U_2m';

L1 = csvread(L_U_1m, 'A:A');     % LIDAR readings at 1m
L2 = csvread(L_U_2m, 'A:A');     % LIDAR readings at 2m
U1 = csvread(L_U_1m, 'B:B');     % Ultrasound readings at 1m
U2 = csvread(L_U_2m, 'B:B');     % Ultrasound readings at 2m

% calculating bias (subtracting true value from mean)
bias_L1 = mean(L1) - 100
bias_L2 = mean(L2) - 200
bias_U1 = mean(U1) - 100
bias_U2 = mean(U2) - 200

% calculating standard deviation
std_L1 = sqrt(sum((L1 - mean(L1)) .^ 2) / (length(L1) - 1))
std_L2 = sqrt(sum((L2 - mean(L2)) .^ 2) / (length(L2) - 1))
std_U1 = sqrt(sum((U1 - mean(U1)) .^ 2) / (length(U1) - 1))
std_U2 = sqrt(sum((U2 - mean(U2)) .^ 2) / (length(U2) - 1))

% subtracting bias from each sensor
L1 = L1 - bias_L1;
L2 = L2 - bias_L2;
U1 = U1 - bias_U1;
U2 = U2 - bias_U2;

% optimal estimates (ki represents coefficients from Ques 6)
V_one = 1 / ((1 / std_L1^2) + (1 / std_U1^2));    % variance for 1m
V_two = 1 / ((1 / std_L2^2) + (1 / std_U2^2));    % variance for 2m

k1      = V_one / std_L1^2;
k2      = V_one / std_U1^2;
x_one   = (L1 * k1) + (U1 * k2);    % combining sensor values
std_X1  = std(x_one)                % standard deviation for 1m

k1      = V_two / std_L2^2;
k2      = V_two / std_U2^2;
x_two   = (L2 * k1) + (U2 * k2);    % combining sensor values
std_X2  = std(x_two)                % standard deviation for 2m