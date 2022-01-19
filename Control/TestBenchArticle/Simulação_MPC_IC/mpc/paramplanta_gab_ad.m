%% Gabriel Renato Oliveira Alves 
%% MPC - classico

function [Ad,Bd,Cd,Dd,T,u0,A,B,C,D] = paramplanta_gab_ad()
%% Parametros do modelo
% Theta 4 parameters
a1 = 0.0187;
b1 = 0.0495;
c1 = 0.6925;

% Theta 2 Parameters
b2 = 0.3338; %friction term
c2 = 1.0173; % Motor effectiveness

% Theta 1 Parameters
%a3 = 0.1724;  % term associated with acceleration
%b3 = 0.4112; % friction term
b3 = 1.013;
%c3 = 2.542; % tilt deflection effectiveness
c3 = 6.313;
d3 = 7.756; % Theta 4 Theta 1 coupling term

L1 = 0.225;
L2 = 0.225;
% Condiçoes de contorno/iniciais
u0 = [1.7;1.7];	% [N] - Forças de equilibrio para os conjuntos propulsivos 1/2

%% Modelo no espaço de estados a tempo continuo
A= [0, 0,	   0,	1,	 0,	     0;
    0, 0,      0,	0,	 1,      0;
    0, 0,      0,   0,   0,      1;
    0, 0,    -d3, -b3,   0,      0;
    0, 0,      0,   0, -b2,      0;
    0, 0, -c1/a1,   0,   0, -b1/a1];

B= [     0,      0,  0,   0;
         0,      0,  0,   0;
         0,      0,  0,   0;
         0,      0, c3, -c3;
        c2,     c2,  0,   0;
     L1/a1, -L2/a1,  0,   0];

C = [1, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0];
D = 0;

sysC = ss(A,B,C,D);
%% Modelo no espaço de estados a tempo Discreto
T = 0.02;	% [s] - Periodo de amostragem
sysD = c2d(sysC,T);
Ad = sysD.A;
Bd = sysD.B;
Cd = sysD.C;
Dd = sysD.D;
end

