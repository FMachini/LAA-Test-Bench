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
clear, close all, clc
%% Model and control parameters
SystemParameters;

%% System dimensions
nxp = 10;
nup = 8;
nxlqr = size(Alqrd,1);
nulqr = size(Blqrd, 2);
numpc = size(Bmpc, 2);
nympc = size(Cmpc, 1);

%% Building MPC matrices
[Hqp, cqpaux, deltaUmax, deltaUmin, Phi, Sqp, Ymax, Ymin] = MPCMatrices(Ampc, Bmpc, Cmpc, Qmpc, Rmpc, N, M, deltaumax, deltaumin, ymax, ymin);

%% Simulation parameters
Tfim = 8;
kend = Tfim/Ts;

%% Solvers parameters
options1 = optimset('Algorithm','interior-point-convex','Display','off'); %quadprog
options_ode = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

%% Scopes
up = zeros(nup, kend);
ulqr = zeros(nulqr, kend);
umpc = zeros(numpc, kend);
deltaumpc = zeros(numpc, kend);
J = zeros(1,kend);
time = zeros(1,kend);
flag = zeros(1,kend);
xp = zeros(12,kend);
xp_lin = zeros(10,kend);
ympc = zeros(nympc, kend);
dummy = zeros(N*numpc,1);

%% Initial condition
% xplanta = [u v w P Q R phi theta psi X Y Z]
%control vector u = [omg1 omg2 omg3 omg4 alp1 alp4 alp3 alp4]'
xp(:,2) = [0 0 0 0 0 0 0*pi/180 0*pi/180 0 0 0 0]';
% xplanta MPC = [P Q phi theta u v w X Y Z]
% xp_lin(:,2) = [0 0 0*pi/180 0*pi/180 0 0 0 0 0 0]';

%% Reference
yref = [1;1;1];
rmpc = repmat(yref, N,1);
%% Loop
for k = 2:kend
   tic;
   
%    xlqrk = xp_lin(1:nxlqr,k);
%    xmpck = xp_lin(:,k);

   xlqrk = xp([4, 5, 7, 8], k);
   xmpck = xp([4, 5, 7, 8, 1, 2, 3, 10, 11, 12], k);
   
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
   
   [dummy, J(k), flag(:,k)] = quadprog(Hqp, cmpc, Sqp, bqp, [], [],[],[],[dummy(numpc+1:end,1); zeros(numpc, 1)],options1);

   deltaumpc(:,k) = dummy(1:numpc, 1);
   umpc(:,k) = umpc(:,k-1) + deltaumpc(:,k);
   
      %% Calculating full control input
   up(:,k) =  [ulqr(:,k) + umpc(end,k)*[1;-1;1;-1];
               umpc(1:end-1,k) ];
           
  %% Total control input
    omg(:, k) = up(1:4,k) + omg_0_vec*1;
    alpha(:, k) = up(5:8,k) + alpha_0;
      
   %% Computacional time
   time(k) = toc();
    
   %% Updating system state
  % discrete
%    xp_lin(:,k+1) = Apd*xp_lin(:,k) + Bpd*up(:,k);
   %continuous
  xini = xp(:,k);
  [t, dummy_m] = ode45(@(t, x) SystemDynamics(t, x, param, omg(:, k), alpha(:, k)), [0 Ts], xini, options_ode);
  xp(:, k+1) = dummy_m(length(t), :)' ;
  % xplanta = [u v w P Q R phi theta psi X Y Z]

end
%%
Plot
% Plot_lin
