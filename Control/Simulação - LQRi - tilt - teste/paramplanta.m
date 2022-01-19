%Gabriel Renato Oliveira Alves 
%MPC com malha interna (lqr) - Controle de altitude e atitude (theta2-theta4)

function [A,B,C,D,T] = paramplanta()
%Parametros do modelo
g = 9.81;
%Criando a bancada
m1 = 2.761;
m2 = 0.154;
m3 = 0.115;
m4 = 0.559;
l1 = 0;
l2 = 0.595;
l3 = 0.173;
l4 = 0;
Ix1 = 0.004165125;
Ix2 = 0.001006095;
Ix3 = 0.00001723333;
Ix4 = 0.01358627;
Iy1 = 0.289994709;
Iy2 = 0.001002423;
Iy3 = 0.00008259913;
Iy4 = 0.00051408;
Iz1 = 0.28761702;
Iz2 = 0.00001835292;
Iz3 = 0.00007608364;
Iz4 = 0.01329993;
b1 = 0.215;
b2 = 0.220;

% Parametros de theta4
mi_d = 0.0495 * 1.0;
CG = 0.6925 * 1.0;

% Parametros do contra-peso
mcp = 0.6730; %0.729;
lcp = 0.322;

fx0 = -g*(m2+m3+m4);
F10 = 2.2;
F20 = 2.35;

den1 = (l2^2)*(m2+m3+m4) + Iz1+Iz2+Iz3+Iz4;
den2 = ((m2+m3+m4)*l2^2+0.25*m2*l3^2+Iy2);
%SS a tempo Continuo
A= [0, 0,                    0, 1, 0,         0;
    0, 0,                    0, 0, 1,         0;
    0, 0,                    0, 0, 0,         1;
    0, 0, (-l2*(F10+F20))/den1, 0, 0,         0;
    0, 0,                    0, 0, 0,         0;
    0, 0,              -CG/Ix4, 0, 0, -mi_d/Ix4];

B= [       0,       0,            0,           0;
           0,       0,            0,           0;
           0,       0,            0,           0;
           0,       0, -l2*F10/den1, l2*F20/den1;
     l2/den2, l2/den2,            0,           0;
      b1/Ix4, -b2/Ix4,            0,           0];

C = [1, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0];
D = 0;
sysC = ss(A,B,C,D);


%Obtendo Modelo Discreto
%SS a tempo Discreto
%Periodo de amostragem
T = 0.02;%(s)
sysD = c2d(sysC,T);
A = sysD.A;
B = sysD.B;
C = sysD.C;
D = sysD.D;



end

