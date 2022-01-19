%%
% Herein the implementation of LQR for stabilization is implemented
% on a tilt quadcopter
%
% Authors: Felipe Machini, Pedro Augusto Queiroz de Assis"
% 
% Date: 18/05/2020
% UFU
%%
clear, %close all, clc
%% Model and control parameters
SystemParameters;

% S�o utilizadas fun��es do tipo struct para facilitar a implementa��o das
% fun��es que representam o modelo din�mico
%Todas essas vari�veis s�o alocadas na struct param.

param.n = 4; param.m = m; param.J = J; param.kd=kd; param.l=ones(1,param.n)*l;param.gamma = gamma; param.JJm = ones(1,param.n)*JJm;param.b = b; param.zcg = ones(1,param.n)*zcg; param.g = g; param.omg0 = omg0;
param.k = k;
tilt1 = 4;
param.tilt1 = tilt1;
tilt2 = 0;
param.tilt2 = tilt2;
num_inp = param.n+tilt1+tilt2;

%% System dimensions
nxp = 10;
nup = 4;
nxlqr = size(Alqrd,1);
nulqr = size(Blqrd, 2);

%% Simulation parameters
Tfim = 8;
kend = Tfim/Ts;

%% Solvers parameters
options1 = optimset('Algorithm','interior-point-convex','Display','off'); %quadprog
options2 = odeset('Reltol',1e-3,'AbsTol',1e-3); %ode45

%% Scopes
up     = zeros(nup, kend);
ulqr    = zeros(nulqr, kend);
time   = zeros(1,kend);
xp     = zeros(nxp,kend);
tt = zeros(1,kend);


%% Initial condition
%state vector x = [P Q phi theta U V W X Y Z]'
%control vector u = [omg1 omg2 omg3 omg4 alp1 alp4 alp3 alp4]'
xp(:,2) = [0 0 20*pi/180 0*pi/180 0 0 0 0 0 0]';
icond = zeros(12,1);

%% Loop
k=0;
for k = 2:kend
   tic;
   
   tt(k+1) = tt(k)+Ts;
   xlqrk = xp(1:nxlqr,k);

   %% LQR control law
   ulqr(:,k) = -Klqr*xlqrk;
   
   %% Updating system state
    omg = ulqr(1:4,k); %vetor que cont�m as velocidades de rota��o �ngulares
    beta = zeros(1,4); %vetor que cont�m os �ngulos de tilt laterais
    alpha = zeros(1,4); %vetor que cont�m os �ngulos de tilt longitudinais
  
    % integra��o das equa��es do movimento de t(k) a tt(k+1) = t(k)+Ts
    [time,XX] = ode45(@(t,Y) Hexa(Y,param,omg,alpha,beta,t),[tt(k) tt(k+1)],icond,options2);
    icond = XX(end,:); %atualiza��o das condi��es iniciais da ODE para integra��o do pr�ximo intervalo de amostragem
    xp(:,k+1) = [XX(end,4) XX(end,5) XX(end,7) XX(end,8) XX(end,1:3) XX(end,10:12)]'; % reodena��o do vetor de estado
end

%save 'MPC_no_rest'
%%
%Plot
