%Gabriel Renato Oliveira Alves 
%LQRi - Controle de atitude e altura(theta24)

function [A,B,C,D,T] = paramplanta()

%Parametros do modelo

%Parametros de Theta4
%c/Ix4 = 63.76;
%b/Ix4 = 9.59;
Ix4 = 0.01;
b4  = 0.0959;
c4  = 0.6376;

%Parametros de Theta2
Iy2 = 0.306954;
b2  = 1.598815;
l2=0.60; % m

d2  = l2/Iy2;   %d = l2/((m4)*l2^2+0.25*m2*l3^2+Iy2);

L1 = 0.215;
L2 = 0.215;

%SS a tempo Continuo
A= [0, 0, 1, 0;
    0, 0, 0, 1;
    0, 0, -b2/Iy2, 0;
    0,-c4/Ix4, 0,-b4/Ix4];

B= [0, 0;
    0, 0;
    d2, d2;
 L1/Ix4, -L2/Ix4];

C = [1, 0, 0, 0;
     0, 1, 0, 0];
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

