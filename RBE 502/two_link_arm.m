function []= two_link_arm()
%clc
clear all
close all

I1=1;  I2 = 1; m1=0.05; a1=0.3; m2=0.05; a2=0.3; l1=0.15; l2=0.15; g=9.8;

a = I1+I2+m1*a1^2+ m2*(l1^2+ a2^2);
b = m2*l1*a2;
d = I2+ m2*a2^2;
ga = (m1*l1 + m2*a1)*g;
gb = m2*l2*g; 

% Inverse kinematics -- task space to joint space
%     acos() produces imaginary values at times, so df1() and df2()
%     have been used as dummy functions to convert all acos() operations
%     to atan2().
df1 = @(x,y) (x^2 + y^2 + a1^2 - a2^2) / (2 * a1 * sqrt(x^2 + y^2));
df2 = @(x,y) (x^2 + y^2 - a1^2 - a2^2) / (2 * a1 * a2);
IK  = @(x,y) ...
           [atan2(y,x) - atan2(1-df1(x,y)^2, df1(x,y));
            atan2(1-df2(x,y)^2, df2(x,y))];

qd = input('Please enter desired position as a 2x1 vector = ');

if isrow(qd)
    qd = qd';
elseif ~iscolumn(qd)
    error('Error!! \nInput must be a vector.', class(qd))
end

xd = qd(1);
yd = qd(2);

x0 = [IK(0.3, 0.45); 0; 0];   % initial XY position
xf = [IK(xd, yd); 0; 0];

tf=10;

global torque
torque=[];

[T,X] = ode45(@(t,x)planarArmODE(t,x,xf),[0 tf],x0);

% syms te1 te2   % equilibrium points
% eq = 10*(xf(1:2) - [te1; te2]) - [ga*sin(te1)+gb*sin(te1+te2); gb*sin(te1+te2)] == 0;
% equilibrium_points = vpasolve(eq, [te1; te2]);

% joint angles vs time
figure(1)
plot(T, X(:,1), 'b')
title('\theta_{1} under PD SetPoint Control')
xlabel('Time'), ylabel('Theta_1 \theta_{1}')
grid on
% hold on
figure(2)
plot(T, X(:,2), 'r')
title('\theta_{2} under PD SetPoint Control')
xlabel('Time'), ylabel('Theta_2 \theta_{2}')
grid on
% hold on

% control input as a function of time
figure(3) 
plot(T, torque(1,1:size(T,1)),'-' )
title('Input PD control input')
grid on
% hold on
plot(T, torque(2,1:size(T,1)),'r--')

% Forward kinematics
end_effector_x = a1*cos(X(:,1)) + a2*cos(X(:,1)+X(:,2));
end_effector_y = a1*sin(X(:,1)) + a2*sin(X(:,1)+X(:,2));
R              = sqrt(end_effector_x.^2 + end_effector_y.^2);
figure(4)
plot(T, R, 'r')
xlabel('Time'), ylabel('End effector position')
title('End-effector position vs. time')
grid on

% Error vs time
err = (sqrt(xd^2+yd^2)*ones(size(T))) - R;
figure(5)
plot(T, err, 'r')
xlabel('Time'), ylabel('end effector error'), grid on
title('Error in end effector position vs. time')

% End-effector trajectory in the vertical plane
% figure(6)
% plot(T, R, 'r*'), xlabel('time'), ylabel('end effector position')
% title('end effector position vs. time'), grid on


    function dx = planarArmODE(~,x,xf)
        theta_d = xf(1:2);  % Desired Set-Point Position
        dtheta_d = xf(3:4); % Desired velocity (Derivative of theta_d)
        
        ddtheta_d=[0;0];
        theta= x(1:2,1);
        dtheta= x(3:4,1);
        
        
        global Mmat Cmat G
        Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
        G    = [ga*sin(x(1))+gb*sin(x(1)+x(2)); gb*sin(x(1)+x(2))];
        invM = inv(Mmat);
        invMC = invM*Cmat;
        
        tau = PDControl(theta_d,dtheta_d,ddtheta_d,theta,dtheta);% - G;
            
        
        torque =[torque tau];
        dx=zeros(4,1);
        dx(1) = x(3); %dtheta1
        dx(2) = x(4); %dtheta2
        dx(3:4) = -invMC* x(3:4) +invM*tau; % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau
    end


 function tau = PDControl(theta_d,dtheta_d,~,theta,dtheta)
        Kp  = 10*eye(2);
        Kv  = 10*eye(2);
        e   = theta_d-theta; % position error
        de  = dtheta_d - dtheta; % velocity error
        tau = (Kp * e) + (Kv * de);
    end
    
disp('Finish.');

end
