function []= Pelican()
clc
clear all;
close all;

I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;

a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;

x0= [-0.5,0.2,0.1,0.1]; % Initial Condition - Format:[theta1,theta2,dtheta1,dtheta2]

tf=10;

%% Solve the closed-loop system nonlinear differential equation (PlanarArmODE) via ode45
%%ode45 solves the differential equation and returns X with respect to T.
global torque
torque=[];

[T,X] = ode45(@(t,x)planarArmODE(t,x),[0 tf],x0);

%% Plot Data
figure('Name','Theta_1 under PD SetPoint Control');
plot(T, X(:,1),'r-');
hold on

figure('Name','Theta_2 under PD SetPoint Control');
plot(T, X(:,2),'r--');
hold on

figure('Name','Input_PD control');
plot(T, torque(1,1:size(T,1)),'-' );
hold on
plot(T, torque(2,1:size(T,1)),'r--');

%torque=[];

%% Defining Functions

    function dx = planarArmODE(t,x)
        theta_d=[0;0]; % Desired Set-Point Position
        dtheta_d=[0;0]; % Desired velocity (Derivative of theta_d)
        ddtheta_d=[0;0];
        theta= x(1:2,1);
        dtheta= x(3:4,1);
        
        
        global Mmat Cmat
        Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
        invM = inv(Mmat);
        invMC = invM*Cmat;
        
        tau = PDControl(theta_d,dtheta_d,ddtheta_d,theta,dtheta);
            
        
        torque =[torque, tau];
        dx=zeros(4,1);
        dx(1) = x(3); %dtheta1
        dx(2) = x(4); %dtheta2
        dx(3:4) = -invMC* x(3:4) +invM*tau; % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau
    end


 function tau = PDControl(theta_d,dtheta_d,ddtheta_d,theta,dtheta)
        Kp=100*eye(2);
        Kv=100*eye(2);
        e=theta_d-theta; % position error
        de = dtheta_d - dtheta; % velocity error
        tau = Kp*e + Kv*de;
    end
    
disp('Finish.');

end
