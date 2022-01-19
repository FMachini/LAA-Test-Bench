function [A,B,C,D,T] = paramplanta()
% x1 = teta
% x2 = F1
% x3 = F2
% x4 = Teta'
% x5 = F1'
% X6 = F2'

%Parametros do modelo
a = 0.0187;
b = -0.0495;
c = 0.6925;
p = 5.348;
q = 13.14;
r = 13.19;
L1 = 0.215;
L2 = 0.225;
%SS a tempo Continuo
% A= [0, 0, 0, 1, 0, 0;
%     0, 0, 0, 0, 1, 0;
%     0, 0, 0, 0, 0, 1;
%     -c/a, L1/a, -L2/a, -b/a, 0, 0;
%     0, -q, 0, 0, -p, 0;
%     0, 0, -q, 0, 0, -p];
% B= [0, 0;
%     0, 0;
%     0, 0;
%     0, 0;
%     r, 0;
%     0, r];
A = [0 1;
    -c/a,b/a];
% C = [1, 0, 0, 0, 0, 0];
B = [0 0;
    L1/a -L2/a];
C = eye(2);
D = 0;
sysC = ss(A,B,C,D);

%Obtendo Modelo Discreto
%Periodo de amostragem
%SS a tempo Discreto
%Periodo de amostragem
T = 0.02;%(s)
sysD = c2d(sysC,T);
A = sysD.A;
B = sysD.B;
C = sysD.C;
D = sysD.D;
end

