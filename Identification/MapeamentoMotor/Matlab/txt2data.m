% Universidade Federal de Uberlândia
%   Faculdade de Engenharia Mecânica
%    Curso de Engenharia Mecatrônica 
%   
%   Criando IDdata a partir dos resultados experimentais
%   
%   Aluno: Gabriel Costa e Silva
% =================================

close all
clear all
clc
% Constantes
rpm_2_rads=((2*pi)/60);
Kg_2_N=(9.81);
Ts=0.012;

txtfile = dlmread('M1_E1.txt');
ForcaPPM = iddata((txtfile(:,3)*Kg_2_N),txtfile(:,2),Ts,'ExperimentName','Força por PPM');
ForcaPPM.OutputName = {'Force'};
ForcaPPM.OutputUnit = {'Newtons'};
ForcaPPM.InputName = {'PPM'};
ForcaPPM.InputUnit = {'milliseconds'};

figure
plot(txtfile(:,3)*9.81)


M1_E1data=iddata(ForcaPPM);
save('M1_E1.data','M1_E1data')

txtfile = dlmread('M1_E2.txt');
ForcaPPM = iddata((txtfile(:,3)*Kg_2_N),txtfile(:,2),Ts,'ExperimentName','Força por PPM');
ForcaPPM.OutputName = {'Force'};
ForcaPPM.OutputUnit = {'Newtons'};
ForcaPPM.InputName = {'PPM'};
ForcaPPM.InputUnit = {'milliseconds'};

M1_E2data=iddata(ForcaPPM);
save('M1_E2.data','M1_E2data')

txtfile = dlmread('M1_E3.txt');
ForcaPPM = iddata((txtfile(:,3)*Kg_2_N),txtfile(:,2),Ts,'ExperimentName','Força por PPM');
ForcaPPM.OutputName = {'Force'};
ForcaPPM.OutputUnit = {'Newtons'};
ForcaPPM.InputName = {'PPM'};
ForcaPPM.InputUnit = {'milliseconds'};

M1_E3data=iddata(ForcaPPM);
save('M1_E3.data','M1_E3data')

txtfile = dlmread('M2_E1.txt');
ForcaPPM = iddata((txtfile(:,3)*Kg_2_N),txtfile(:,2),Ts,'ExperimentName','Força por PPM');
ForcaPPM.OutputName = {'Force'};
ForcaPPM.OutputUnit = {'Newtons'};
ForcaPPM.InputName = {'PPM'};
ForcaPPM.InputUnit = {'milliseconds'};

M2_E1data=iddata(ForcaPPM);
save('M2_E1.data','M2_E1data')


txtfile = dlmread('M2_E2.txt');
ForcaPPM = iddata((txtfile(:,3)*Kg_2_N),txtfile(:,2),Ts,'ExperimentName','Força por PPM');
ForcaPPM.OutputName = {'Force'};
ForcaPPM.OutputUnit = {'Newtons'};
ForcaPPM.InputName = {'PPM'};
ForcaPPM.InputUnit = {'milliseconds'};

M2_E2data=iddata(ForcaPPM);
save('M2_E2.data','M2_E2data')


txtfile = dlmread('M2_E3.txt');
ForcaPPM = iddata((txtfile(:,3)*Kg_2_N),txtfile(:,2),Ts,'ExperimentName','Força por PPM');
ForcaPPM.OutputName = {'Force'};
ForcaPPM.OutputUnit = {'Newtons'};
ForcaPPM.InputName = {'PPM'};
ForcaPPM.InputUnit = {'milliseconds'};

M2_E3data=iddata(ForcaPPM);
save('M2_E3.data','M2_E3data')



ForcaTempo = iddata((txtfile(:,3)*Kg_2_N),txtfile(:,1),Ts,'ExperimentName','Força por tempo');
ForcaTempo.OutputName = {'Force'};
ForcaTempo.OutputUnit = {'Newtons'};
ForcaTempo.InputName = {'Time'};
ForcaTempo.InputUnit = {'seconds'};


PPMTempo = iddata(txtfile(:,2),txtfile(:,1),Ts,'ExperimentName','PPM por tempo');
PPMTempo.OutputName = {'PPM'};
PPMTempo.OutputUnit = {'milliseconds'};
PPMTempo.InputName = {'Time'};
PPMTempo.InputUnit = {'seconds'};

ForcaPPM = iddata((txtfile(:,3)*Kg_2_N),txtfile(:,2),Ts,'ExperimentName','Força por PPM');
ForcaPPM.OutputName = {'Force'};
ForcaPPM.OutputUnit = {'Newtons'};
ForcaPPM.InputName = {'PPM'};
ForcaPPM.InputUnit = {'milliseconds'};


figure
plot(ForcaPPM.u,movmean(ForcaPPM.y,20))
title("Mapeamento Estático");
xlabel("PPM [milliseconds]")
ylabel("Força [N]")

figure
subplot(2,1,1)
plot(ForcaTempo.u,movmean(ForcaTempo.y,20))
title("Mapeamento Estático");
xlabel("Tempo [s]")
ylabel("Força [N]")
subplot(2,1,2)
plot(PPMTempo.u,PPMTempo.y)
xlabel("Tempo [s]")
ylabel("PPM [millisseconds]")

