clear all
clc

% Motion Control of Mobile Robots based on Linearization
% Example: Rectilinear
% Method #2 using descrete time evolution of the dynamics.

%% Desired Trajectory (You can do trajectory planning in this section)

% q0 = [0, 0, 0]; % Initial Desired Configuration
% qf = [30, 0, 0]; % Final Desired Configuration
Tf = 10;	% Total time for tracking the trajectory

% In this specific example, it is assumed that the desired trajectory is 
%a rectilinear path in the x direction with a constant velocity of vd=3.
% Therefore, xd=3*t and thetad=0. 


dt=0.01; % time increment
tsteps=[0:dt:Tf]; % time steps
N=size(tsteps,2);

%% Initial Configuration of the robot
X(:,1)=[1, 2, 0]; % Play around with the initital configuration and see 
% the result. format: [x0, y0, theta0]
Xdes(:,1)=[0; 0; 0]; % Desired initial configuraiton
E(:,1) = Xdes(:,1)-X(:,1); % Initial robot's configuration error

%% 

for i=1:N-1
    
    t=tsteps(i);
    Xdes(:,i)=[3*t;0;0]; % Desired trajectory (i.e. from the pllaner)
    X(:,i)=Xdes(:,i)-E(:,i); % Actual robot configuration
    vd=3; % driving velocity
    wd=0; % steering velocity


dr=0.7; %damping ration (play around with it and see the result)
nf=1; %natural frequency (play around with it and see the result)
k1=2*dr*nf; % Equation (11.69)
k3=k1; % Equation (11.69)
k2=(nf^2-wd^2)/vd; % Equation (11.69)

A = [ -k1, wd, 0;
      -wd, 0, vd;
       0, -k2, -k3]; % From Equation (11.68)

Edot = A*E(:,i); % Equation 11.68

E(:,i+1)= Edot*dt+ E(:,i); % Descrete time method of solving differential equation

end

figure % Plot
plot(X(1,:), X(2,:),'LineWidth', 4);
hold on 
plot(Xdes(1,:), Xdes(2,:), 'LineWidth', 4);

