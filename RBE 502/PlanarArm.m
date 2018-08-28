function []= planarArm()

clc
clear all;
close all;
%parameters for the arm
I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;

% we compute the parameters in the dynamic model
a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;

%%

% initial condition - Format:[theta1,theta2,dtheta1,dtheta2]
x0= [-0.5,0.2,0.1,0.1]; %You can change the initial condition here.

w=0.2;
tf=10;

% the options for ode - Optional!
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);

%% Implement the computed Torque control.
index=1;
global torque
torque=[];
[T,X] = ode45(@(t,x)plannarArmODE(t,x,index),[0 tf],x0,options);
figure('Name','Theta_1 under Computed Torque Control');
plot(T, X(:,1),'r-');
hold on
plot(T, w*ones(size(T,1),1),'b-');
figure('Name','Theta_2 under Computed Torque Control');
plot(T, X(:,2),'r--');
hold on
plot(T, sin(2*T),'b-');

figure('Name','Input_Computed Torque Control');
plot(T, torque(1,1:size(T,1)),'-' );
hold on
plot(T, torque(2,1:size(T,1)),'r--');
torque=[];

%% Implement the PD control plus Feedforward.
index=2;
[T,X] = ode45(@(t,x)plannarArmODE(t,x,index),[0 tf],x0,options);
figure('Name','Theta_1 under PD Control plus Feedforward');
plot(T, X(:,1),'r-');
hold on
plot(T, w*ones(size(T,1),1),'b-');
figure('Name','Theta_2 under PD Control plus Feedforward');
plot(T, X(:,2),'r--');
hold on
plot(T, sin(2*T),'b-');
hold on


figure('Name','Input_ PD control plus Feedforward');
plot(T, torque(1,1:size(T,1)),'-' );
hold on
plot(T, torque(2,1:size(T,1)),'r--');
hold on

%% Functions

    function [dx ] = plannarArmODE(t,x,idx)
        theta_d= [w;sin(2*t)]; %You can insert any desired joint trajectory here!
        dtheta_d =[0; 2*cos(2*t)]; %Time derivative of theta_d
        ddtheta_d = [0; -4*sin(2*t)]; %Second time derivated of theta_d
        theta= x(1:2,1);
        dtheta= x(3:4,1);
        
        global Mmat Cmat Mmatd Cmatd
        Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
        Mmatd = [a+2*b*cos(theta_d(2)), d+b*cos(theta_d(2));  d+b*cos(theta_d(2)), d];
        Cmatd = [-b*sin(theta_d(2))*dtheta_d(2), -b*sin(theta_d(2))*(dtheta_d(1)+dtheta_d(2)); b*sin(theta_d(2))*dtheta_d(1),0];
        invM = inv(Mmat);
        invMC = invM*Cmat;
        switch idx 
            case 1
                tau = computeTorque(theta_d, dtheta_d, ddtheta_d, theta, dtheta); %
            
            case 2
                tau = PDplusFeedforwawrd(theta_d, dtheta_d, ddtheta_d, theta, dtheta);
        end
        torque =[torque, tau];
        dx=zeros(4,1);
        dx(1) = x(3);
        dx(2) = x(4);
        dx(3:4) = -invMC* x(3:4) +invM*tau; % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau
    end

    function tau =computeTorque(theta_d, dtheta_d,ddtheta_d, theta, dtheta)
        global Mmat Cmat
        Kp=100*eye(2);
        Kv=100*eye(2);
        e=theta_d-theta; % position error
        de = dtheta_d - dtheta; % velocity error
        tau= Mmat*(Kp*e + Kv*de) + Cmat*dtheta + Mmat*ddtheta_d;
    end

    function tau = PDplusFeedforwawrd(theta_d, dtheta_d,ddtheta_d, theta, dtheta)
        global Mmatd Cmatd
        Kp=100*eye(2);
        Kv=100*eye(2);
        e=theta_d-theta; % position error
        de = dtheta_d - dtheta; % velocity error
        tau= (Kp*e + Kv*de) + Cmatd*dtheta_d + Mmatd*ddtheta_d;
    end
disp('Finish.');

end
