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


figure
txtfile = dlmread('MOTORPRETO.txt');

plot(txtfile(1000:5000,3)*Kg_2_N,'r')
hold on

txtfile = dlmread('M2_E1.txt');

plot(txtfile(500:2850,3)*Kg_2_N,'k')

txtfile = dlmread('M2_E2.txt');

plot(txtfile(500:2850,3)*Kg_2_N,'k')

txtfile = dlmread('M2_E3.txt');

plot(txtfile(500:2850,3)*Kg_2_N,'k')
grid on
yticks(0:0.25:5)
legend('Dois motores ligados','Somente o outro motor')
xlabel('N amostras')
ylabel('Forças [N]')
print('MotorCW_comparativo','-dpdf','-bestfit')

% txtfile = dlmread('dados.txt');
% 
% plot(txtfile(3200:8000,3)*Kg_2_N+0.1962,'r')
% hold on
% 
% txtfile = dlmread('M2_E1.txt');
% 
% plot(txtfile(1500:5000,3)*Kg_2_N,'k')
% 
% txtfile = dlmread('M2_E2.txt');
% 
% plot(txtfile(500:4000,3)*Kg_2_N,'k')
% 
% txtfile = dlmread('M2_E3.txt');
% 
% plot(txtfile(500:3800,3)*Kg_2_N,'k')
% grid on
% yticks(0:0.25:5)
% legend('Dois motores ligados','Somente o outro motor')

%figure;plot(m2sozim(:,2),'k');hold on;plot(m1sozim(:,2),'r');plot(MCCWjunto(:,2)+0.1962,'b');legend('Somente um motor até 75 por cento PPM','Somente outro motor até 75 por cento PPM','Dois motores ligados até 60 por cento PPM');xlabel('N amostra');ylabel('Força[N]');grid on;print('comparativo','-dpdf','-bestfit')