clear all; clc; close all;

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
SystemParameters;

A = textread(strcat('Results\MPC_Data_V25.txt'));
%% System dimensions
nxp = 6;
nup = 4;
nxlqr = size(Alqrd,1);
nulqr = size(Blqrd, 2);
numpc = size(Bmpc, 2);
nympc = size(Cmpc, 1);


%% Simulation parameters
Tfim = 30;
kend = 1000;

%% Solvers parameters

%% Scopes
up     = [A(:,7)-1.7 A(:,8)-1.7 A(:,9)*pi/180 A(:,10)*pi/180]';
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
xp(:,2) = A(2,1:6)*pi/180';
%% Reference

%% Loop

for k = 2:kend
   
    xp(:,k+1) = Apd*xp(:,k) + Bpd*up(:,k);

end

%% Exporting data
save 'MPC_no_rest_6'

 
Plot
