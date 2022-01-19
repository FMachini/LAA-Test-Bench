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
clear all , close all, clc
%% Model and control parameters
SystemParameters;

param.n = 4; param.m = m; param.J = J; param.kd=kd; param.l=ones(1,param.n)*l;param.gamma = gamma; param.JJm = ones(1,param.n)*JJm;param.b = b; param.zcg = ones(1,param.n)*zcg; param.g = g; param.omg0 = omg0;
param.k = k;
tilt1 = 4;
param.tilt1 = tilt1;
tilt2 = 0;
param.tilt2 = tilt2;
num_inp = param.n+tilt1+tilt2;

%% System dimensions
nxp = 10;
nup = 8;
nxlqr = size(Alqrd,1);
nulqr = size(Blqrd, 2);

%% Building MPC matrices

%% Simulation parameters
Tfim = 0.8;
kend = Tfim/Ts;

%% Solvers parameters
options1 = optimset('Algorithm','interior-point-convex','Display','off'); %quadprog
options2 = odeset('Reltol',1e-3,'AbsTol',1e-3); %ode45

%% Scopes
ulqr    = zeros(nulqr, kend);
time   = zeros(1,kend);
flag   = zeros(1,kend);
xp     = zeros(nxp,kend);
tt = zeros(1,kend);


%% Initial condition
%state vector x = [P Q phi theta U V W X Y Z]'
%control vector u = [omg1 omg2 omg3 omg4 alp1 alp4 alp3 alp4]'
xp(:,2) = [1 0 0*pi/180 0*pi/180 0 0 0 0 0 0]';
icond = zeros(12,1);
icond(4,1)=1;
%% Loop
k=0;
for k = 2:kend

   tt(k+1) = tt(k)+Ts;
   
   %% LQR control law
   ulqr(:,k) = -Klqr*xp(1:4,k);
   
   %% Updating system state
    omg = ulqr(:,k)';
    beta = zeros(1,4);
    alpha = alpha0;
   %discrete
    %xp(:,k+1) = Apd*xp(:,k) + Bpd*up(:,k);
  %continuous
%    xini = xp(:,k);
% %    [t,dummy4] = ode45(@(t,a) fnlin(t,a,u(:,k)),[0 Td],xini,optionsk2); %default 0.1tspan
%    [t,dummy4] = ode45(@(t,x) fnlin(t,x,up(k),g,Km,Mb,i0,xb0),[0 Td],xini,options2); %default 0.1tspan
%    xp(:,k+1) = dummy4(length(t),:)';
    [time,XX] = ode45(@(t,Y) Hexa(Y,param,omg,alpha,beta,t),[tt(k) tt(k+1)],icond,options2);
    icond = XX(end,:);
    xp(:,k+1) = [XX(end,4) XX(end,5) XX(end,7) XX(end,8) XX(end,1:3) XX(end,10:12)]';
end
up = ulqr;
%save 'MPC_no_rest'
%%
%Plot
