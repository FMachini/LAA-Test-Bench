function [A,B,C,D,T] = paramplanta()
% x1 = teta
% x2 = Teta'

%Parametros do modelo
a = 0.0187;
b = 0.06;%0.0495;
c = 0.6;%0.6925;
p = 5.348;
q = 13.43;
r = 1.319;
L1 = 0.225;
L2 = 0.215;
%SS a tempo Continuo
A= [0, 1;
    -c/a, -b/a];
B= [0, 0;
    (L1)/(a), -(L2)/(a)];
Aa = [0 1 0; zeros(2,1) A];
Bb = [0 0;B];
%  B= [0;
%      (L1+L2)/a]; % considerando apenas uma entrada: forca f aplicada + no motor 1 e - no motor 2
C = [1, 0 0];
D = 0;
sysC = ss(Aa,Bb,C,D);


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

