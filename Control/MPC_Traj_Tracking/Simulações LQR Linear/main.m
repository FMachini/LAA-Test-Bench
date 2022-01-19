%%
% Herein the implementation of LQR for stabilization about a hoover
% condition on a tilt quadcopter
%
% Authors: Felipe Machini, Pedro Augusto Queiroz de Assis"
% 
% Date: 18/05/2020
% UFU
%%
clear, close all, clc
%% Model and control parameters
SystemParameters;

%% System dimensions
nxp = 4;
nup = 4;
nxlqr = size(Alqrd,1);
nulqr = size(Blqrd, 2);


%% Simulation parameters
Tfim = 8;
kend = Tfim/Ts;

%% Scopes
ulqr    = zeros(nulqr, kend);
time   = zeros(1,kend);
xp     = zeros(nxp,kend);

%% Initial condition
%state vector x = [P Q phi theta U V W X Y Z]'
%control vector u = [omg1 omg2 omg3 omg4 alp1 alp4 alp3 alp4]'
xp(:,2) = [0 0 20*pi/180 0*pi/180]';

%% Loop
for k = 2:kend
   tic;
   
   xlqrk = xp(1:nxlqr,k);
   xmpck = xp(:,k);
   
   %% LQR control law
   ulqr(:,k) = -Klqr*xlqrk;

   xp(:,k+1) = Alqrd*xp(:,k) + Blqrd*ulqr(:,k);
end
%%
Plot
