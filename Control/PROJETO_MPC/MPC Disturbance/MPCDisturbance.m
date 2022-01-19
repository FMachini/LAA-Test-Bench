%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Herein the implementation of a combination between LQR and MPC
% for stabilization and position reference tracking is implemented
% on a tilt quadcopter
% The model takes into account all the identified models for Theta1, Theta4
% and Theta2.
% The tilt actuation dynamics is included in the model as a second order
% transfer function.
% A state observer is implemented to estimate the velocities and
% accelerations with and without disturbance variable 
% Disturbances are included in the prediction model for steady state error
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model States
% X = [Theta4 Theta4_dot Theta1 Theta1_dot Theta2 Theta2_dot alphaOut1 ...
% aphaOut1_dot alphaOut2 alphaout2_dot d_theta1 d_theta2]
% Model input commands
% u = [f1 f2 alphaDes1 alphaDes2]
% Output vector
% y = [Theta4 Theta4_dot Theta1 Theta1_dot Theta2 Theta2_dot]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Authors: Felipe Machini, Pedro Augusto Queiroz de Assis"
% 
% Data: 20/05/2021
% UFU
%%
clear all;clc; close all;

%% Model and control parameters
SystemParameters; %Load model and observer matrices 
[Atiltd,Btiltd] = tiltDynamics(Ts);

% Check if the system is stabilizable and detectable
[eigvec, eigval] = eig(Ad);
stb =eigval*Bd; % check if system is stabilizable (no zero rows)
dec = Cd*eigvec; % check if system is detactable (no zero columns)
%system is detectable and stabilizable

%% State Observer
% design state observer including disturbance in the model
%Cd(1,1) = 1;Cd(2,3)=1; Cd(3,5)=1;
Qob = diag([1e-3 1e-3 50 10 50 10 1e-4 1e-4 1e-4 1e-4 0.1 0.1]);
Rob = diag([10 10 0.1 1 0.1 1]);

numDis = 2;
Aobs = [Ad zeros(10,numDis);
        zeros(numDis,10) eye(numDis)];
Bobs = [Bd; zeros(numDis,4)];
Cobs1 = zeros(6,numDis); Cobs1(1,1)=1; Cobs1(3,2)=1;
Cobs = [Cd Cobs1];

Lobs = dlqr(Aobs',Cobs', Qob,Rob);
% % L = inv(Ax)*L;
Lobs = Lobs';
damp(Aobs-Lobs*Cobs)
% Lx = [L; eye(3)];

%% MPC model with disturbance

Ax = [Ampc zeros(10,numDis);
        zeros(numDis,10) eye(numDis)];
 Bx = [Bmpc; zeros(numDis,3)];
 Cx1 = zeros(4,numDis); Cx1(1,1)=1; Cx1(3,2)=1;
 Cx = [Cmpc Cx1];

%% System dimensions
nxp = 10; % includes tilt dynamics
nup = 4;
nxlqr = size(Alqrd,1);
nulqr = size(Blqrd, 2);
numpc = size(Bmpc, 2);
nympc = size(Cmpc, 1);

%% Building MPC matrices
[Hqp, cqpaux, deltaUmax, deltaUmin, Phi, Sqp, Ymax, Ymin] = MPCMatrices(Ax, Bx, Cx, Qmpc, Rmpc, N, M, deltaumax, deltaumin, ymax, ymin);

% Saving Matrice in .txt files
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
kend = 1500;
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
%xp(:,2) = zeros(nxp,1);
%xp(:,2) = [0*0.865  0*0.509  19.980     0.089   -1.620   -0.000 ]'*pi/180;
alphavec1 = zeros(2,kend); %tilt angular dynamics vector
alphavec2 = zeros(2,kend);
xp(:,2) = [0 0 0 0 0 0 0 0 0 0]'*pi/180;

%% Reference
yref = 20*pi/180*[1;0;0;0];
rmpc = repmat(yref, N,1);

%% Loop

xmpck = zeros(nxp+2,kend);
xmpck(:,2) = [xp(:,2);0;0];
ye = zeros(6,1);
poles = [1 1 (9.21e-01+6.41e-02i) (9.21e-01-6.42e-02i) (9.21e-01+6.42e-02i) ...
    (9.21e-01-6.41e-02i) 9.9e-1 9.7e-1 (9.65e-01-1.15e-01i) (9.65e-01+1.15e-01i)];
Kplace = place(Ad,Bd,poles);
%Ad = Ad;
Cdaux = zeros(2,size(Ad,2));Cdaux(1,3) =1;Cdaux(2,5) =1;
%Cdaux2 = Cdaux; Cdaux(1,11) = 1; Cdaux(2,12) = 1;
out = zeros(10,1);
for k = 2:kend
   tic;
   %xp(:,k) = xp(:,2);
   yout = Cd * xp(:,k);
   tt(k+1) = tt(k)+Ts;
   xlqrk = xp(1:nxlqr,k);
   
   %xmpck(:,k)  = xp(1:end,k);
   
   % State Observer prediction
   ye = Cobs*xmpck(:,k-1);
   xmpck(:,k) = Aobs*xmpck(:,k-1) + Lobs*(yout - ye) + Bobs*up(:,k-1);
   
   
   %% LQR control law
   ulqr(:,k) = -Klqr*xlqrk;

   %% MPC
   %fmpc = Phi*[eye(6,9)*xmpck(:,k+1); umpc(:,k-1)];
   %stVec = [xmpck(:,k);Cdaux*xmpck(:,k) - Cdaux*xp(:,k)];
   fmpc = Phi*[xmpck(:,k); umpc(:,k-1)];
   
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
    xp(:,k+1) = (Ad-Bd*Kplace)*xp(:,k) + Bd*up(:,k);
    %xp(:,k+1) = eye(6,10)*out;
end
%xp(:,end+1) = [0*0.865  0*0.509 10 0.089   -1.620   -0.000 ]'*pi/180;
%% Exporting data
save 'Theta1Dynamics'
 
 %dlmwrite('results.txt',[xp(:,1:end-1)'*180/pi,up(1:2,:)'+1.7,up(3:4,:)'*180/pi]);
%%
Plot
