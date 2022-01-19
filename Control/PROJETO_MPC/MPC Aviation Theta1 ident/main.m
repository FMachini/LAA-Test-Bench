%%
% Herein the implementation of a combination between LQR and MPC
% for stabilization and position reference tracking is implemented
% on a tilt quadcopter
%
% Authors: Felipe Machini, Pedro Augusto Queiroz de Assis"
% 
% Data: 11/03/2020
% UFU
%%
clear all;clc; close all;
%% Model and control parameters
SystemParametersUpdated;
%[Ad,Bd] = LinearIdentifiedModel(Ts);

%% System dimensions
nxp = 7;
nup = 4;
nxlqr = size(Alqrd,1);
nulqr = size(Blqrd, 2);
numpc = size(Bmpc, 2);
nympc = size(Cmpc, 1);

%% Building MPC matrices
% [Hqp, cqpaux, deltaUmax, deltaUmin, Phi, Sqp, Ymax, Ymin] = MPCMatrices(Ampc, Bmpc, Cmpc, Qmpc, Rmpc, N, M, deltaumax, deltaumin, ymax, ymin);
% dlmwrite('Hqp.txt',Hqp);
% dlmwrite('cqpaux.txt',cqpaux);
% dlmwrite('deltaUmax.txt',deltaUmax);
% dlmwrite('deltaUmin.txt',deltaUmin);
% dlmwrite('Phi.txt',Phi);
% dlmwrite('Sqp.txt',Sqp);
% dlmwrite('Ymax.txt',Ymax);
% dlmwrite('Ymin.txt',Ymin);
% dlmwrite('umax.txt',umax);
% dlmwrite('umin.txt',umin);
% dlmwrite('Klqr.txt',Klqr);
[Hqp, cqpaux,deltaUmax, deltaUmin, Phi, Sqp, Ymax, Ymin, umax, umin, Klqr ] = loadMpcMatrices();

%% Simulation parameters
Tfim = 20;
kend = Tfim/Ts;

%% Solvers parameters
options1 = optimset('Algorithm','interior-point-convex','Display','off'); %quadprog
options2 = odeset('Reltol',1e-3,'AbsTol',1e-3); %ode45

%% Scopes
up     = zeros(nup, kend);
ulqr    = zeros(nulqr, kend);
umpc   = zeros(numpc, kend);
deltaumpc   = zeros(numpc, kend);
JJ      = zeros(1,kend);
time   = zeros(1,kend);
flag   = zeros(1,kend);
xp     = zeros(nxp,kend);
ympc     = zeros(nympc, kend);
tt = zeros(1,kend);

dummy = zeros(N*numpc,1);

%% Initial condition
%state vector x = [P Q phi theta U V W X Y Z]'
%control vector u = [omg1 omg2 omg3 omg4 alp1 alp4 alp3 alp4]'
%xp(:,2) = zeros(nxp,1);
xp(:,2) = pi/180 *[0 0 -1.89 0 0 0 0]';
%% Reference
yref = 30*pi/180*[1;0;0;0];
%yref = 30*pi/180*[1;0];
rmpc = repmat(yref, N,1);
%% Loop
k=0;
for k = 2:kend
   tic;
   
   tt(k+1) = tt(k)+Ts;
   xlqrk = xp(1:nxlqr,k);
   xmpck = xp(1:end-1,k);
   
   %% LQR control law
   ulqr(:,k) = -Klqr*xlqrk;

   %% MPC
   fmpc = Phi*[xmpck; umpc(:,k-1)];
   
   bqp = [deltaUmax;
         -deltaUmin;
          repmat(umax - umpc(:,k-1), M,1);
          repmat(umpc(:,k-1) - umin, M,1);
          Ymax - fmpc;
          fmpc - Ymin];
       
    cmpc = cqpaux*(fmpc - rmpc);
   
   [dummy, JJ(k), flag(:,k)] = quadprog(Hqp, cmpc, Sqp, bqp, [], [],[],[],[dummy(numpc+1:end,1); zeros(numpc, 1)],options1);
   deltaumpc(:,k) = dummy(1:numpc, 1);
   umpc(:,k) = umpc(:,k-1) + deltaumpc(:,k);
   
      %% Calculating full control input
   up(:,k) =  [ulqr(:,k) + umpc(end,k)*ones(2,1);
               umpc(1:end-1,k) ];
      
   %% Computacional time
   time(k) = toc();
    
   %% Updating system state
    omg = up(1:2,k);
    beta = zeros(1,4);
    alpha = up(2:4,k);
   %discrete
    xp(:,k+1) = Ad*xp(:,k) + Bd*up(:,k);
    
end

%% Exporting data
save 'Theta1Dynamics'

 
 %dlmwrite('results.txt',[xp(:,1:end-1)'*180/pi,up(1:2,:)'+1.7,up(3:4,:)'*180/pi]);
%%
Plot
PlotExpData;
