%%
% Herein the implementation of a combination between LQR and MPC
% for stabilization and position reference tracking is implemented
% on a tilt quadcopter
% The model takes into account all the identified models for Theta1, Theta4
% and Theta2.
% A state observer is implemented to estimate the velocities and
% accelerations
% The Cost function is altered to include the input in order to drive it to
% zero
% Authors: Felipe Machini, Pedro Augusto Queiroz de Assis"
% 
% Data: 30/03/2020
% UFU
%%
clear all;clc; close all;
%% Model and control parameters
SystemParameters;

%% State Observer
% Cd = zeros(3,7);
% Cd(1,1) = 1; Cd(2,3)=1; Cd(3,5)=1; %Cd(4,7) = 1;
% 
% Qob = diag(0.01*[5 10 0.01 5 30 50 100]);
% 
% Rob = diag(0.1*[100 1000 100]);
% 
% L = dlqr(Ad',Cd', Qob,Rob);
% L = L';
% damp(Ad-L*Cd)
% Kobs = place(Ad',Cd',[-0.7+0.10i -0.7-0.10i -0.5 -0.5 -0.8+0.5i -0.8-0.5i -0.8]);
% Kobs = Kobs';

Cd = zeros(3,7);
Cd(1,1) = 1; Cd(2,3)=1; Cd(3,5)=1;
Cx = [Cd eye(3)];
Ax = [Ad zeros(7,3);zeros(3,7) eye(3)];

% Qob = diag(0.01*[5 10 0.01 5 30 50 100]);
% 
% Rob = diag(0.1*[100 1000 100]);
Qob = diag(0.01*[5000 100000 0.01 0.1 3 5 10]);

Rob = diag(0.1*[10000 100 10]);

L = dlqr(Ad',Cd', Qob,Rob);
L = L';

%% System dimensions
nxp = 7; % includes acceleration
nup = 4;
nxlqr = size(Alqrd,1);
nulqr = size(Blqrd, 2);
numpc = size(Bmpc, 2);
nympc = size(Cmpc, 1);

%% Building MPC matrices
[Hqp, cqpaux, deltaUmax, deltaUmin, Phi, Sqp, Ymax, Ymin, fqpaux] = MPCMatrices(Ampc, Bmpc, Cmpc, Qmpc, Rmpc, N, M, deltaumax, deltaumin, ymax, ymin);

dlmwrite('Hqp.txt',Hqp);
dlmwrite('cqpaux.txt',cqpaux);
dlmwrite('deltaUmax.txt',deltaUmax);
dlmwrite('deltaUmin.txt',deltaUmin);
dlmwrite('fqpaux.txt',fqpaux);
dlmwrite('Sqp.txt',Sqp);
dlmwrite('Ymax.txt',Ymax);
dlmwrite('Ymin.txt',Ymin);
dlmwrite('umax.txt',umax);
dlmwrite('umin.txt',umin);
dlmwrite('Klqr.txt',Klqr);

%% Simulation parameters
Tfim = 120;
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
xp(:,2) = [3.63*pi/180 0 0 0 0 0 0]';
%% Reference
%yref = 30*pi/180*[1;0;1;0];
yref = 30*pi/180*[1;1];
rmpc = repmat(yref, N,1);
%% Loop
k=0;
xmpck = zeros(size(xp));
xmpck(:,2) = xp(:,2);
%ye = zeros(3,1);
for k = 2:kend
   tic;
   yout = Cd * xp(:,k);
   tt(k+1) = tt(k)+Ts;
   xlqrk = xp(1:nxlqr,k);% 3*pi/180 * rand(2,1);
   %xmpck = xp(1:end,k);
   
   % State Observer prediction
   ye = Cd*xmpck(:,k);
   xmpck(:,k+1) = Ad*xmpck(:,k) + L*(yout - ye) + Bd*up(:,k);
   
   
   %% LQR control law
   ulqr(:,k) = -Klqr*xlqrk;

   %% MPC
   fmpc = Phi*[xp(:,k); umpc(:,k-1)];
   bqp = [deltaUmax;
         -deltaUmin;
          repmat(umax - umpc(:,k-1), M,1);
          repmat(umpc(:,k-1) - umin, M,1);
          Ymax - fmpc;
          fmpc - Ymin];
    
    gmpc = [xmpck(:,k+1)' (umpc(:,k-1))']*fqpaux;
    cmpc = (gmpc - rmpc'*cqpaux);
   
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
xp = xmpck;
save 'StateObsOut'

 
 dlmwrite('results.txt',[xp(:,1:end-1)'*180/pi,up(1:2,:)'+1.7,up(3:4,:)'*180/pi]);
%%
Plot
