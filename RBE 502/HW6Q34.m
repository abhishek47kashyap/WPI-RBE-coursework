clear all
clc

% Desired Trajectory

Tf = 16;	           % Total time for tracking the trajectory
dt = 0.01;             % time increment

tsteps = dt:dt:Tf; % time steps
N      = length(tsteps);


des_x = [dt:dt:4, 4*ones(1,N/4), fliplr(dt:dt:4), dt*ones(1,N/4)];
des_y = [zeros(1,N/4), dt:dt:4, 4*ones(1,N/4), fliplr(dt:dt:4)];
des_t = [zeros(1,N/4), pi/2*zeros(1,N/4), -pi*zeros(1,N/4), -pi/2*zeros(1,N/4)];

des_traj = [des_x; des_y; des_t];

angle_init = 0;
b          = 0.2;     % distance between center and front tip

x_current = [0.5 + b*cos(angle_init);
             -0.5 + b*sin(angle_init);
             angle_init];            % robot starting point [x0, y0, theta0]

X = [];  % [driving velocity; steering velocity]

for i = 1:N-1
    
    err = des_traj(:,i) - x_current(:,i);
    
    % controller gains
    k1 = 2;       
    k2 = 2;
    % no k3 because no tracking for theta
    
    U = [k1 * err(1);
         k2 * err(2)];
    
    theta = x_current(3,i); 
    Tinv  = [ cos(theta),      sin(theta);
             -sin(theta)/b,    cos(theta)/b];
      
    X = [X, Tinv * U];
    
    x_current(:,i+1) = x_current(:,i) + [X(:,i)*dt; des_traj(3,i+1)];
end

figure(1), plot(des_x, des_y, 'b', x_current(1,:), x_current(2,:), 'r')
legend('Reference trajectory', 'Trajectory followed', 'Location', 'best')
title('Tracking square with b = 0.2')
xlabel('X (m)'), ylabel('Y (m)')
axis([-1, 5, -1, 5])

figure(2), subplot(2,1,1)
plot(tsteps(1:1599), X(1,:)), title('Driving velocity')
xlabel('time'), ylabel('m/s')
grid on
subplot(2,1,2), plot(tsteps(1:1599), X(2,:))
title('Steering velocity'), grid on
xlabel('time'), ylabel('rad/s')