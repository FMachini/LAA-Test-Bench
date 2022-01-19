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
Ts=0.025;

txtfile = dlmread('dado2_35porcento.txt');
data2_35 = iddata(txtfile(:,4),txtfile(:,1),Ts,'ExperimentName','COMPORTAMENTO Do TILT');
data2_35.OutputName = {'Angle'};
data2_35.OutputUnit = {'degrees'};
data2_35.InputName = {'Time'};
data2_35.InputUnit = {'seconds'};

save('data2_35.mat','data2_35')

txtfile = dlmread('dado2_14porcento.txt');
data2_14 = iddata(txtfile(:,4),txtfile(:,1),Ts,'ExperimentName','COMPORTAMENTO Do TILT');
data2_14.OutputName = {'Angle'};
data2_14.OutputUnit = {'degrees'};
data2_14.InputName = {'Time'};
data2_14.InputUnit = {'seconds'};

save('data2_14.mat','data2_14')

figure
plot(data2_35.u,movmean(data2_35.y,20))
hold on
plot(data2_14.u,movmean(data2_14.y,15))

txtfile = dlmread('dado1_din_35.txt');
data1_35 = iddata(txtfile(:,4),txtfile(:,1),Ts,'ExperimentName','COMPORTAMENTO Do TILT');
data1_35.OutputName = {'Angle'};
data1_35.OutputUnit = {'degrees'};
data1_35.InputName = {'Time'};
data1_35.InputUnit = {'seconds'};

save('data1_35.mat','data1_35')

% txtfile = dlmread('tilt114porcento.txt');
% data1_14 = iddata(txtfile(:,4),txtfile(:,1),Ts,'ExperimentName','COMPORTAMENTO Do TILT');
% data1_14.OutputName = {'Angle'};
% data1_14.OutputUnit = {'degrees'};
% data1_14.InputName = {'Time'};
% data1_14.InputUnit = {'seconds'};
% 
% save('data1_14.mat','data1_14')



figure
plot(data1_35.u,movmean(data1_35.y,20))
% hold on
% plot(data1_14.u,movmean(data1_14.y,15))

