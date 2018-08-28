function [] = Planer2DArm()
%clc;
%clear all;
    a1 = 0.3;
    a2 = 0.3;
    l1 = 0.3/2;
    l2 = 0.3/2;
    m1 = 0.05;
    m2 = 0.05;
    g = 9.8;
    I1 = 1;
    I2 = 1;

    a = I1+I2+m1*l1^2+ m2*(l2^2+ a1^2);
    b = m2*l2*a1;
    d = I2+ m2*l2^2;
    %4i1
    prompt = 'Please input the desired set point value cordinates (x),2x1 matrix? ';
    x_dash = input(prompt);
    y_dash = input(prompt);


    %4i2
    c2 = (x_dash^2 + y_dash^2 -a1^2 - a2^2)/2*a1*a2;
    s2 = sqrt(1-c2^2);
    q2 = atan2(s2,c2)
    q1 = atan2(y_dash,x_dash) - atan2((a2*s2),(a1 + (a2*c2)))

  
    %4i3
    x0= [q1,q2,0,0]; % Initial Condition - Format:[theta1,theta2,dtheta1,dtheta2]
    tf=10;
    global torque
    torque = [];

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
    torque=[];

    %%Defining Function
    function dx = planarArmODE(t,x)
    
        theta_d = [x_dash;y_dash];
        dtheta_d=[0;0]; % Desired velocity (Derivative of theta_d)
        ddtheta_d=[0;0];
        theta= x(1:2,1);
        dtheta= x(3:4,1);
        
        global Mmat Cmat
        Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        Cmat = [-2*b*sin(x(2))*x(4), -b*sin(x(2))*(2*x(3)+x(4)); b*sin(x(2))*x(3),0];
        invM = inv(Mmat);
        invMC = invM*Cmat;
        tau = PDControl(theta_d,dtheta_d,ddtheta_d,theta,dtheta);
        torque =[torque, tau];
        dx=zeros(4,1);
        dx(1) = x(3); %dtheta1
        dx(2) = x(4); %dtheta2
        dx(3:4) = -invMC* x(3:4) + invM*tau; % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau
    end
    function tau = PDControl(theta_d,dtheta_d,ddtheta_d,theta,dtheta)
        Kp=10*eye(2);
        Kv=10*eye(2);
        e=theta_d-theta; % position error
        de = dtheta_d - dtheta; % velocity error
        tau = Kp*e + Kv*de;
    end
end
