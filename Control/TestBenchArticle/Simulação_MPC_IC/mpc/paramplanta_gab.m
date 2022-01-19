%% Gabriel Renato Oliveira Alves 
%% MPC - classico

function [Ad,Bd,Cd,Dd,T,u0,A,B,C,D] = paramplanta_gab()
%% Parametros do modelo
l2 = 0.173;             % [m] - Comprimento do elemento 2
Izj0 = 0.5848;          % [kg.m²] - Momento de inercia total em relação a Zj0
Izj1 = 0.3070;          % [kg.m²] - Momento de inercia total em relação a Zj1
Izp = 0.01;             % [kg.m²] - Momento de inercia total em relação a Zp 
b1 = 0.215;             % [m] - Comprimento do braço esquerdo do bicoptero
b2 = 0.215;             % [m] - Comprimento do braço direito do bicoptero
mi2 = 1.5988;           % [kg.m²/s] - Coeficiente de atrito dinamico
mi4 = 0.0959;     	% [kg.m²/s] - Coeficiente de atrito dinamico
gamma4 = 0.6376;	% [kg.m²/s²] - Coeficiente atrelado ao desbalanceamento do bicoptero

% Condiçoes de contorno/iniciais
u0 = [1.7;1.7];	% [N] - Forças de equilibrio para os conjuntos propulsivos 1/2

%% Modelo no espaço de estados a tempo continuo
A= [0, 0,                        0, 1,         0,         0;
    0, 0,                        0, 0,         1,         0;
    0, 0,                        0, 0,         0,         1;
    0, 0, (-l2*(u0(1)+u0(2)))/Izj0, 0,         0,         0;
    0, 0,                        0, 0, -mi2/Izj1,         0;
    0, 0,              -gamma4/Izp, 0,         0,  -mi4/Izp];

B= [       0,       0,             0,              0;
           0,       0,             0,              0;
           0,       0,             0,              0;
           0,       0, l2*u0(1)/Izj0, -l2*u0(2)/Izj0;
     l2/Izj1, l2/Izj1,             0,              0;
      b1/Izp, -b2/Izp,             0,              0];

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

