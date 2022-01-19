%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Title: 3 DOF Test Bench Model Using Robotic Notation 
%%% Control using MPC controller with State Observer
%%% Attitude and position dynamics decoupled
%%% Author: Felipe Machini M. Marques 
%%% Date: 28/04/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all;clc


%% Creating test bench robotic model

BenchModel; %create 3dof bench model
actuatorDynamics; %Create actators transfer functions
Ts = 0.02; % Sampling time


%% Loading Model Predictive Control Matrices and State observer
 
SystemParameters;
%[Hqp, cqpaux, deltaUmax, deltaUmin, Phi, Sqp, Ymax, Ymin] = MPCMatrices(Ampc, Bmpc, Cmpc, Qmpc, Rmpc, N, M, deltaumax, deltaumin, ymax, ymin);
[Hqp, cqpaux,deltaUmax, deltaUmin, Phi, Sqp, Ymax, Ymin, umax, umin, Klqr ] = loadMpcMatrices();
%% System dimensions
nxp = 6; % includes acceleration
nup = 4;
nxlqr = size(Alqrd,1);
nulqr = size(Blqrd, 2);
numpc = size(Bmpc, 2);
nympc = size(Cmpc, 1);

%% Robot Definitions
q = homeConfiguration(robot); 
ndof = size(q,2);

% Initializing variables
kend = 1000;


%% Scopes
up     = zeros(nup, kend);
ulqr    = zeros(nulqr, kend);
umpc   = zeros(numpc, kend);
deltaumpc   = zeros(numpc, kend);
JJ      = zeros(1,kend);
time   = zeros(1,kend);
flag   = zeros(1,kend);
ympc     = zeros(nympc, kend);
tt = zeros(1,kend);
x = zeros(2*ndof,kend);
u = zeros(4,kend);
xmpck = zeros(nxp,kend);

dummy = zeros(N*numpc,1);

%% Solvers parameters
options1 = optimset('Algorithm','interior-point-convex','Display','off'); %quadprog
options2 = odeset('Reltol',1e-3,'AbsTol',1e-3); %ode45

%% Setiing initial conditions
x(:,1) = [q zeros(size(q))];
u = zeros(4,kend);
utrim = [1.7 1.7 0 0]'; %equilibrium condition
xm1=0;xm2=0;xa1=[0,0];xa2=[0,0];
int_e = 0;
Cc1 = zeros(3,6);
Cc1(1,1) = 1; Cc1(2,3)=1; Cc1(3,5)=1;

Cc2 = [0 0 0 1 0 0 0 0;
      0 0 0 0 0 0 0 1;
      1 0 0 0 0 0 0 0;
      0 0 0 0 1 0 0 0;
      0 -1 0 0 0 0 0 0;
      0 0 0 0 0 -1 0 0]; %get Theta4 Theta4Dot Theta1 Theta1Dot Theta2 Theta2Dot

xmpck(:,1) = Cc2*x(:,1);  
  
%% Reference
yref = pi/180*[30;0;-18.7;0];
%yref = 30*pi/180*[1;0];
rmpc = repmat(yref, N,1);
alphavec1 = zeros(2,kend); %tilt angular dynamics vector
alphavec2 = zeros(2,kend);
 
for  i = 1:kend
    % Redefining States order
    X = Cc2*x(:,i);
    % State Observer prediction
    %yout =  Cc1*X;
    %ye = Cd*xmpck(:,i);
    %xmpck(:,i+1) = Ad*xmpck(:,i) + L*(yout - ye) + Bd*up(:,i);
    
    % Calculating control action
    
    % LQR control action
    ulqr(:,i) = -Klqr*X(1:2);
    
    % MPC Control Action
    fmpc = Phi*[X; umpc(:,i)];
   
    bqp = [deltaUmax;
         -deltaUmin;
          repmat(umax - umpc(:,i), M,1);
          repmat(umpc(:,i) - umin, M,1);
          Ymax - fmpc;
          fmpc - Ymin];
       
    cmpc = cqpaux*(fmpc - rmpc);
   
    [dummy, JJ(i), flag(:,i)] = quadprog(Hqp, cmpc, Sqp, bqp, [], [],[],[],[dummy(numpc+1:end,1); zeros(numpc, 1)],options1);
    deltaumpc(:,i) = dummy(1:numpc, 1);
    umpc(:,i+1) = umpc(:,i) + deltaumpc(:,i);
   
      %% Calculating full control input
    up(:,i) =  [ulqr(:,i) + umpc(end,i)*ones(2,1);
               umpc(1:end-1,i) ];    
     % Actuator dynamics
     [f1,~,xm1] = lsim(motor1SYS,up(1,i)*ones(3,1),0.02*(0:0.5:1),xm1(end),'zoh');
     [f2,~,xm2] = lsim(motor2SYS,up(2,i)*ones(3,1),0.02*(0:0.5:1),xm2(end),'zoh');
     [alpha1,~,xa1] = lsim(alpha1SYS,up(3,i)*ones(3,1)*180/pi,0.02*(0:0.5:1),xa1(end,:),'zoh');
     [alpha2,~,xa2] = lsim(alpha2SYS,up(4,i)*ones(3,1)*180/pi,0.02*(0:0.5:1),xa2(end,:),'zoh');

    % Calulating Command
    u(:,i) = [f1(end);f2(end);alpha1(end)*pi/180;alpha2(end)*pi/180];

    utotal = u(:,i) + utrim;
    % Estimating Bicopter forces and moments wrt body coordinate frame
    [fBCS] = Bicopter(utotal);
    % Projecting joint forces on Inertial Coordinate frame
    fext = externalForce(robot,'endeffector',fBCS,x(1:ndof,i)');
    %calculating friction torques at each joint
    tauFric = -diag([0.4112 0.33 0.1 2.6471])*x(ndof+1:end,i);
    % Calculating joint accelerations
    qddot = forwardDynamics(robot,x(1:ndof,i)',x(ndof+1:end,i)',tauFric',fext);
    qdot = x(ndof+1:end,i) + qddot'*Ts; 
    q = x(1:ndof,i) + qdot*Ts;
    x(:,i+1) = [q;qdot];
    
end
%xp = [x(4,1:end-1)' x(8,1:end-1)' x(1,1:end-1)' x(5,1:end-1)' -x(2,1:end-1)' -x(6,1:end-1)']*180/pi;

%% Plotting Data
%dlmwrite('resultsNonLinear.txt',[xp,up(1:2,:)'+1.7,up(3:4,:)'*180/pi]);
plotData;
PlotExpData;

%% Simulation

% figure('rend','painters','pos',[10 10 1000 800])
% for i = 1:kend
%     show(robot,x(1:ndof,i)','PreservePlot',false);
%     axis([-1,1,-1,1,-0.5,1])
%     pause(0.001)    
% end


