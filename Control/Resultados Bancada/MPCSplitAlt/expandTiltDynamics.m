function [Aexd,Bexd,Cexd,L] = expandTiltDynamics(Ac,Bc,Ts)
% This function expands the MPC prediction model to include the tilt
% mechanism dynamics and design a state observer 

% Tilt model Matrices
Act2 = [0 1;-28.228 -8.033];
% Bct2 = [0;28.228];
% Cct2 = eye(2,2);
% Dct2 = [0;0];

Act1 = [0 1;-28.228 -8.033];
% Bct1 = [0;28.228];
% Cct1 = eye(2,2);
% Dct1 = [0;0];

Aex = [Ac(1:4,1:4) zeros(4,4);zeros(4,4) blkdiag(Act1,Act2)];
Aex(4,5) = Bc(4,3);
Aex(4,7) = Bc(4,4);

Bex = [Bc(1:4,1:2) zeros(4,2); zeros(4,4)];

Bex(6,3) = 28.228;Bex(8,4) = 28.228;

% Cexd= zeros(3,10);
% Cexd(1,1) = 1;Cexd(2,3)=1;Cexd(3,5)=1;%Cex(3,5)=1;Cex(4,6)=1;

Cexd = eye(4,8);
[Aexd, Bexd] = c2dm(Aex, Bex, Cexd, zeros(4,4), Ts, 'zoh');

% Q = diag(1*ones(1,10));R = diag(1*ones(1,3));
Q = diag([1e-3 1e-3 20 20 1e-4 1e-4 1e-4 1e-4]);
R = diag([5 10 1 1]);

% Design State Observer
L = dlqr(Aexd',Cexd',Q,R);
L = L';

 end

