%%
% Herein the implementation of a combination between LQR and MPC
% for stabilization and position reference tracking is implemented
% on a tilt quadcopter
% The model takes into account all the identified models for Theta1, Theta4
% and Theta2.
% A state observer is implemented to estimate the velocities and
% accelerations
% Authors: Felipe Machini, Pedro Augusto Queiroz de Assis"
% 
% Data: 30/03/2020
% UFU
%%
clear all;clc; close all;
%% Model and control parameters
SystemParameters;
[Atiltd,Btiltd] = tiltDynamics(Ts);
IncludeTiltDynamics;

%% State Observer
Cd = zeros(3,6);
Cd(1,1) = 1;Cd(2,3)=1; Cd(3,5)=1;
Qob = diag(1*[1 1 1 1 1 1 5 5 5]);

 Ax = [Ad zeros(6,3);
        zeros(3,6) eye(3)];
 Bx = [Bd; zeros(3,4)];
 Cx1 = zeros(3,2); Cx1(1,1)=1;Cx1(2,2)=1;
 Cx = [Cd eye(3)];

Rob = diag(100*[0.01 1 1]);

L = dlqr(Ax',Cx', Qob,Rob);
% L = inv(Ax)*L;
L = L';
damp(Ax-L*Cx)

%% System dimensions
nxp = 6; % includes acceleration
nup = 4;
nxlqr = size(Alqrd,1);
nulqr = size(Blqrd, 2);
numpc = size(Bmpc, 2);
nympc = size(Cmpc, 1);

%% Building MPC matrices
[Hqp, cqpaux, deltaUmax, deltaUmin, Phi, Sqp, Ymax, Ymin] = MPCMatrices(Ampc, Bmpc, Cmpc, Qmpc, Rmpc, N, M, deltaumax, deltaumin, ymax, ymin);
% dlmwrite('Matrices/Hqp.txt',Hqp);%,'delimiter','\t','precision',8);
% dlmwrite('Matrices/cqpaux.txt',cqpaux);
% dlmwrite('Matrices/deltaUmax.txt',deltaUmax);
% dlmwrite('Matrices/deltaUmin.txt',deltaUmin);
% dlmwrite('Matrices/Sqp.txt',Sqp);
% dlmwrite('Matrices/Phi.txt',Phi);
% dlmwrite('Matrices/Ymax.txt',Ymax);
% dlmwrite('Matrices/Ymin.txt',Ymin);
% dlmwrite('Matrices/umax.txt',umax);
% dlmwrite('Matrices/umin.txt',umin);
% dlmwrite('Matrices/Klqr.txt',Klqr);
% dlmwrite('Matrices/Lobs.txt',L);
% dlmwrite('Matrices/Ad.txt',Ad);
% dlmwrite('Matrices/Bd.txt',Bd);
% dlmwrite('Matrices/Cd.txt',Cd);
% dlmwrite('Matrices/MPCParameters.txt',[N M nxp nup nympc nxlqr nulqr numpc 750 10 10 Ts]);
% load Mats
% M =6;
%% Simulation parameters
% Tfim = 100;
% kend = Tfim/Ts;
kend = 1000;
%% Solvers parameters
%options1 = optimset('Algorithm','active-set','Display','off'); %quadprog
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
%xp(:,2) = [0*0.865  0*0.509  19.980     0.089   -1.620   -0.000 ]'*pi/180;
alphavec1 = zeros(2,kend); %tilt angular dynamics vector
alphavec2 = zeros(2,kend);
xp(:,2) = 0*[0*0.865  0*0.509 10 0.089 0.509  -1.620]'*pi/180;

%% Reference
yref = 20*pi/180*[1;0;0;0];
%yref = 30*pi/180*[1;0];
rmpc = repmat(yref, N,1);

%% Loop
k=0;
xmpck = zeros(nxp+3,kend);
xmpck(:,2) = [xp(:,2);0;0;0];
ye = zeros(3,1);

out = zeros(10,1);
for k = 2:kend
   tic;
   %xp(:,k) = xp(:,2);
   yout = Cd * xp(:,k);
   tt(k+1) = tt(k)+Ts;
   xlqrk = xp(1:nxlqr,k);
   
   %xmpck(:,k)  = xp(1:end,k);
   
   % State Observer prediction
   %ye = Cxd*xmpck(:,k);
   %xmpck(:,k+1) = Ax*xmpck(:,k) + L*(yout - ye) + Bx*up(:,k);
   
   
   %% LQR control law
   ulqr(:,k) = -Klqr*xlqrk;

   %% MPC
   %fmpc = Phi*[eye(6,9)*xmpck(:,k+1); umpc(:,k-1)];
   fmpc = Phi*[xp(1:end,k); umpc(:,k-1)];
   
   bqp = [deltaUmax;
         -deltaUmin;
          repmat(umax - umpc(:,k-1), M,1);
          repmat(umpc(:,k-1) - umin, M,1);
          Ymax - fmpc;
          fmpc - Ymin];
       
    cmpc = cqpaux*(fmpc - rmpc);
   %Hqpaux = Hqp;
   [dummy, JJ(k), flag(:,k)] = quadprog(Hqp, cmpc, Sqp, bqp, [], [],[],[],[dummy(numpc+1:end,1); zeros(numpc, 1)],options1);
   %[dummy, JJ(k), flag(:,k)] = quadprog(Hqpaux, cmpc, Sqp, bqp, [], [],[],[],[],options1);
   %[dummy2,exitflag] = mpcActiveSetSolver(Hqp, cmpc, Sqp, bqp,[],[]);
   deltaumpc(:,k) = dummy(1:numpc, 1);
   umpc(:,k) = umpc(:,k-1) + deltaumpc(:,k);
   
      %% Calculating full control input
   up(:,k) =  [ulqr(:,k) + umpc(end,k)*ones(2,1);
               umpc(1:end-1,k) ];
      
   %% Computacional time
   time(k) = toc();
    
   %% Updating system state
%     alpha = umpc(1:2,k)*180/pi;
%     alphavec1(:,k) = Atiltd*alphavec1(:,k-1) + Btiltd*alpha(1);
%     alphavec2(:,k) = Atiltd*alphavec2(:,k-1) + Btiltd*alpha(2);
%     
%     alphactr = [alphavec1(1,k);alphavec2(1,k)]*pi/180;
    
    up(:,k) =  [ulqr(:,k) + umpc(end,k)*ones(2,1);
                umpc(1:2,k)];
%     up(:,k) =  [ulqr(:,k) + umpc(end,k)*ones(2,1);
%                 alphactr];
            
    %discrete
     out = Aexd*out + Bexd*up(:,k);
    xp(:,k+1) = eye(6,10)*out;
end
%xp(:,end+1) = [0*0.865  0*0.509 10 0.089   -1.620   -0.000 ]'*pi/180;
%% Exporting data
save 'Theta1Dynamics'
 
 %dlmwrite('results.txt',[xp(:,1:end-1)'*180/pi,up(1:2,:)'+1.7,up(3:4,:)'*180/pi]);
%%
Plot
