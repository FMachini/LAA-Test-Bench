%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Title: 3 DOF Test Bench Model Using Robotic Notation 
%%% Author: Felipe Machini M. Marques 
%%% Date: 28/04/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all;clc


%% Creating test bench robotic model

BenchModel; %create 3dof bench model
actuatorDynamics; %Create actators transfer functions


%% Robot Simulation
q = homeConfiguration(robot); 
ndof = size(q,2);

% Initializing variables
kend = 2000;
Ts = 0.02;

x = zeros(2*ndof,kend);
x(:,1) = [q zeros(size(q))];
u = zeros(4,kend);
uctr = zeros(4,kend);
utrim = [1.7 1.7 0 0]'; %equilibrium condition
xm1=0;xm2=0;xa1=[0,0];xa2=[0,0];
int_e = 0;
Cc = [1 0 0 0 0 0 0 0;
      0 -1 0 0 0 0 0 0;
      0 0 0 1 0 0 0 0];%get theta1 theta2 theta4 positions
  
Cc2 = [1 0 0 0 0 0 0 0;
      0 -1 0 0 0 0 0 0;
      0 0 0 1 0 0 0 0;
      0 0 0 0 1 0 0 0;
      0 0 0 0 0 -1 0 0;
      0 0 0 0 0 0 0 1]; %get theta1 theta2 theta4 positions and velocities
 %xref = [10;0;0]*pi/180;
 
 % Computing Control Gain Matrices
 [Kp,Kx] = computeGain(Ts); %Compute LQR controller gains
 
for  i = 1:kend
    if i<500
        xref = [0;20;0]*pi/180;
    else
        xref = [10;20;0]*pi/180;
    end
    
    % Calculating control action
    e = xref - Cc*x(:,i);
    int_e = int_e + e*Ts;
    uctr(:,i+1) =  Kp*int_e - Kx*Cc2*x(:,i);
        
%     % Actuator dynamics
     [f1,~,xm1] = lsim(motor1SYS,uctr(1,i+1)*ones(5,1),0.02*(0:0.25:1),xm1(end),'zoh');
     [f2,~,xm2] = lsim(motor2SYS,uctr(2,i+1)*ones(5,1),0.02*(0:0.25:1),xm2(end),'zoh');
     [alpha1,~,xa1] = lsim(alpha1SYS,uctr(3,i+1)*ones(5,1)*180/pi,0.02*(0:0.25:1),xa1(end,:),'zoh');
     [alpha2,~,xa2] = lsim(alpha2SYS,uctr(4,i+1)*ones(5,1)*180/pi,0.02*(0:0.25:1),xa2(end,:),'zoh');
     
    u(:,i+1) = [f1(end);f2(end);alpha1(end)*pi/180;alpha2(end)*pi/180];
    %utotal = uctr(:,i+1) + utrim;
    % Estimating Bicopter forces and moments wrt body coordinate frame
    [fBCS] = Bicopter(u(:,i+1));
    % Projecting joint forces on Inertial Coordinate frame
    fext = externalForce(robot,'endeffector',fBCS,x(1:ndof,i)');
    %calculating friction torques at each joint
    tauFric = -diag([0.4 0.4 0.4 0.4])*x(ndof+1:end,i);
    % Calculating joint accelerations
    qddot = forwardDynamics(robot,x(1:ndof,i)',x(ndof+1:end,i)',tauFric',fext);
    qdot = x(ndof+1:end,i) + qddot'*Ts; 
    q = x(1:ndof,i) + qdot*Ts;
    x(:,i+1) = [q;qdot];
end


%% Plotting Data

plotData;

%% Simulation

% figure('rend','painters','pos',[10 10 1000 800])
% for i = 1:kend
%     show(robot,x(1:ndof,i+1)','PreservePlot',false);
%     axis([-1,1,-1,1,-0.5,1])
%     pause(0.02)    
% end


