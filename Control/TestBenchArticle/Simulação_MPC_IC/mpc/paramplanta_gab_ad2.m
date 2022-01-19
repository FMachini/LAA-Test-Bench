%% Gabriel Renato Oliveira Alves 
%% MPC - classico

function [Ad,Bd,Cd,Dd,T,u0,A,B,C,D] = paramplanta_gab_ad2()
%% Parametros do modelo
%% System Parameters
% Test Bench Properties
Iz1 = 0.2876;Iz2 = 0.001; Iz3 = 0; Iz4 = 0.0135;

% Condiçoes de contorno/iniciais
u0 = [1.7;1.7];	% [N] - Forças de equilibrio para os conjuntos propulsivos 1/2
% Test Bench Properties
a = 0.0187;
b = 0.0495;
c = 0.6925;
Iyy2 = 0.289;
m2 = 0.154;
m3 = 0.115;
m4 = 0.600; % bicopter mass
l3 = 0.173;
l2 = 0.605;
L1 = 0.225;
L2 = 0.225;

m_counter_weight = 1.7;
l4 = 0.33;
den2 = Iyy2 + 1/4 * l3^2* m2 + l2^2*(m2+m3+m4) + l4^2*m_counter_weight;
den1 = l2^2*(m2+m3+m4)+Iz1+Iz2+Iz3+Iz4 + l4^2*m_counter_weight;
a1 = -3.4*l2/den1; %based on the trimming motor thrust
% Linear State Space Model
% x = [Theta1 Theta2 Theta4 Theta1_dot Theta2_dot Theta4_dot]
% expanded state vector (including error) z = [(x1ref-x1) (x2ref-x2) x_dot]

A = [0, 0,    0, 1, 0,    0;
     0, 0,    0, 0, 1,    0;
     0, 0,    0, 0, 0,    1;
     0, 0,   a1, 0, 0,    0;
     0, 0,    0, 0, 0,    0;
     0, 0, -c/a, 0, 0, -b/a];

b1 = l2/den1;
b2 = l2/den2;

B = [      0,       0,           0,            0;
           0,       0,           0,            0;
           0,       0,           0,            0;
           0,       0, l2*1.7/den1, -l2*1.7/den1;
     l2/den2, l2/den2,           0,            0;
        L1/a,   -L2/a,           0,            0];

C = [1 0 0 0 0 0
     0 1 0 0 0 0
     0 0 1 0 0 0]; 

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

