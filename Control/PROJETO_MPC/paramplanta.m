function [A,B,C,D,T] = paramplanta()
% x1 = teta
% x2 = Teta'

%Parametros do modelo
a = 0.0187;
b = 0.0495;
c = 0.6925;
p = 5.348;
q = 13.43;
r = 1.319;
L1 = 0.225;
L2 = 0.215;

%SS a tempo Continuo
A= [0, 1;
    -c/a, -b/a];
%B= [0, 0;
%    (L1*r)/(q*a), -(L2*r)/(q*a)];

B= [0, 0;
    L1/a, -L2/a];

C = [1, 0];
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

