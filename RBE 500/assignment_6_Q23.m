%% Questions 2 and 3

clear all
close all
clc

mean_0 = [0; 0];
sigma_0 = [0 0; 0 0];


[mean_1, sigma_1] = Kalman_filter_without_z(mean_0, sigma_0)
[mean_2, sigma_2] = Kalman_filter_without_z(mean_1, sigma_1)
[mean_3, sigma_3] = Kalman_filter_without_z(mean_2, sigma_2)
[mean_4, sigma_4] = Kalman_filter_without_z(mean_3, sigma_3)
[mean_5, sigma_5] = Kalman_filter_without_z(mean_4, sigma_4)


figure(1);
Plot_ellipse(mean_2, sigma_2)
grid minor;
title('t = 2');

figure(2);
Plot_ellipse(mean_3, sigma_3)
grid minor;
title('t = 3');
figure(3)
Plot_ellipse(mean_4, sigma_4)
grid minor;
title('t = 4');
figure(4)
Plot_ellipse(mean_5, sigma_5)
title('t = 5');
grid minor;

figure(5);
Plot_ellipse(mean_2, sigma_2)
hold on
Plot_ellipse(mean_3, sigma_3)
Plot_ellipse(mean_4, sigma_4)
Plot_ellipse(mean_5, sigma_5)
grid minor;


%Normal distribution parameters before updating the measurement of z = 5m at t = 5
[mean_5_bar, sigma_5_bar] = Kalman_filter_without_z(mean_4, sigma_4)

%Normal distribution parameters after updating the Kalman Filter
[mean_5, sigma_5] = Kalman_filter(mean_4, sigma_4, 5)

figure(6);
Plot_ellipse(mean_5_bar, sigma_5_bar)
grid minor;
title('Uncertainty ellipse before updating');

figure(7);
Plot_ellipse(mean_5, sigma_5)
grid minor;
title('Uncertainty ellipse after updating');

figure(8);
Plot_ellipse(mean_5_bar, sigma_5_bar)
hold on
grid minor;
Plot_ellipse(mean_5, sigma_5)
title('Uncertainty ellipse before updating vs after updating');

function [mean_t, sigma_t] = Kalman_filter_without_z(mean_t_1, sigma_t_1)
    A_t = [1 1; 0 1];
    R_t = 1;
    D = [0.5 ; 1 ];
    D = D * R_t * D';
    mean_t = A_t * mean_t_1;
    sigma_t = A_t * sigma_t_1 * transpose(A_t) + D; %changed equation
end


function [] = Plot_ellipse(mean, sigma)
    x1 = -3:0.2:3;
    x2 = -3:0.2:3;
    [X1, X2] = meshgrid(x1, x2);
    F = mvnpdf([X1(:) X2(:)], mean', sigma);
    F = reshape(F, length(x2), length(x1));

    contour(x1, x2, F, [.001 .01 .15:.1:.85 .99 .999]);
    xlabel('x'); ylabel('xdot');
end


function [mean_t, sigma_t] = Kalman_filter(mean_t_1, sigma_t_1, z_t)
    A_t = [1 1; 0 1];
    R_t = 1;
    C_t = [1 0];
    Q_t = 10;
    D = [0.5 ; 1 ];
    D=D * R_t * D';
    
    % Prediction
    mean_t_bar = A_t * mean_t_1;
    sigma_t_bar = A_t * sigma_t_1 * A_t' + D; %changed equation
    
    % Correction
    K_t = sigma_t_bar * C_t' * inv(C_t * sigma_t_bar * C_t' + Q_t);
    mean_t = mean_t_bar + K_t * (z_t - C_t * mean_t_bar);
    sigma_t = (eye(2) - K_t * C_t) * sigma_t_bar;
end
