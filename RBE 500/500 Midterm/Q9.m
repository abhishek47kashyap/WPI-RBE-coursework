% importing data (All LIDAR and Ultrasound readings are for 2 meters)
L_U_soft = '/Users/abhishek47kashyap/Documents/500 mid-term/Q9/Q9_L_U_2m_45_soft';
L_U_hard = '/Users/abhishek47kashyap/Documents/500 mid-term/Q9/Q9_L_U_2m_45';

Ls = xlsread(L_U_soft, 'A:A');     % LIDAR readings for soft wall
Lh = xlsread(L_U_hard, 'A:A');     % LIDAR readings for hard wall
Us = xlsread(L_U_soft, 'B:B');     % Ultrasound readings for soft wall
Uh = xlsread(L_U_hard, 'B:B');     % Ultrasound readings for hard wall

% calculating bias (subtracting true value from mean)
bias_Ls = mean(Ls) - 200
bias_Lh = mean(Lh) - 200
bias_Us = mean(Us) - 200
bias_Uh = mean(Uh) - 200

% calculating standard deviation
std_Ls = sqrt(sum((Ls - mean(Ls)) .^ 2) / (length(Ls) - 1))
std_Lh = sqrt(sum((Lh - mean(Lh)) .^ 2) / (length(Lh) - 1))
std_Us = sqrt(sum((Us - mean(Us)) .^ 2) / (length(Us) - 1))
std_Uh = sqrt(sum((Uh - mean(Uh)) .^ 2) / (length(Uh) - 1))

% subtracting bias from each sensor
Ls = Ls - bias_Ls;
Lh = Lh - bias_Lh;
Us = Us - bias_Us;
Uh = Uh - bias_Uh;

% optimal estimates (ki represents coefficients from Ques 6)
Vs = 1 / ((1 / std_Ls^2) + (1 / std_Us^2));    % variance for soft wall
Vh = 1 / ((1 / std_Lh^2) + (1 / std_Uh^2));    % variance for hard wall

k1        = Vs / std_Ls^2;
k2        = Vs / std_Us^2;
x_soft    = (Ls * k1) + (Us * k2);    % combining sensor values
std_soft  = std(x_soft)                % standard deviation for soft wall

k1        = Vh / std_Lh^2;
k2        = Vh / std_Uh^2;
x_hard    = (Lh * k1) + (Uh * k2);    % combining sensor values
std_hard  = std(x_hard)                % standard deviation for hard wall