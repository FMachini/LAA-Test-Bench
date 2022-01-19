%% Mapeamento do tilt rotor
clear all
close all

global Ts; 
global numValues;
global numPWM;

Ts = 0.01;
numValues=250;
numPWM=15;

filename1='ensaiotilt1_01.txt';
filename2='ensaiotilt1_02.txt';
filename3='ensaiotilt1_03.txt';

dado1 = readDataTilt(filename1);
dado2 = readDataTilt(filename2);
dado3 = readDataTilt(filename3);

dado_final=[dado1; dado2; dado3];


offset=  62.51; % Para qual valor de angulo de thetapot temos o angulo zero do tilt
dado_final=dado_final-offset*ones(size(dado_final));
f=[dado_final(:,1);dado_final(:,2);dado_final(:,3);dado_final(:,4);dado_final(:,5);dado_final(:,6);dado_final(:,7);dado_final(:,8);dado_final(:,9);dado_final(:,10);dado_final(:,11);dado_final(:,12);dado_final(:,13);dado_final(:,14);dado_final(:,15)];
pwm = [1650000*ones(size(dado_final,1),1);1700000*ones( size(dado_final,1),1);1750000*ones( size(dado_final,1),1);1800000*ones( size(dado_final,1),1);1850000*ones( size(dado_final,1),1);1900000*ones( size(dado_final,1),1);1950000*ones( size(dado_final,1),1);2000000*ones( size(dado_final,1),1);2050000*ones( size(dado_final,1),1);2100000*ones( size(dado_final,1),1);2150000*ones( size(dado_final,1),1);2200000*ones( size(dado_final,1),1);2250000*ones( size(dado_final,1),1);2300000*ones( size(dado_final,1),1);2350000*ones( size(dado_final,1),1)];


%%%%% Figuras Importantes

figure

errorbar([1650000:50000:2350000],mean(dado_final),std(dado_final),'*')
title("Mapeamento Estatico do Tilt2");
xlabel("$PWM [nseconds]$",'Interpreter','latex');
ylabel("$\alpha_1 [graus]$",'Interpreter','latex');
hold on

%%Regress�o Linear sobre os dados
format long
coefs = polyfit(pwm,f,2);
%coefs=round(coefs,6);
pwm1=linspace(1650000,2350000,1000);
plot(pwm1,polyval(coefs,pwm1));

leg1=legend('Dados experimentais','Curva Ajustada - \alpha_1[PWM]=0.0000224995092*PWM²-0.1508455823443*PWM+229.8191731868106')

set(leg1,'Interpreter','latex')
grid on
print('Tilt1_grau2','-dpdf','-bestfit')


figure

errorbar([1650000:50000:2350000],mean(dado_final),std(dado_final),'*')
title("Mapeamento Estatico do Tilt2");
xlabel("$PWM [nseconds]$",'Interpreter','latex');
ylabel("$\alpha_1 [graus]$",'Interpreter','latex');
hold on

coefs1 = polyfit(pwm,f,1);
%coefs1=round(coefs1,6);
pwm1=linspace(1650000,2350000,1000);
plot(pwm1,polyval(coefs1,pwm1));

leg1=legend('Dados experimentais','Curva Ajustada - alpha_1[PWM]=-0.0608475457143*PWM+140.8711136507924')

set(leg1,'Interpreter','latex')
grid on
print('Tilt1_grau1','-dpdf','-bestfit')



%%%%% Figuras do angulo do potenciometro
% figure
% errorbar([1650:50:2350],mean(dado_final),std(dado_final),'*')
% title("Mapeamento Estatico do Tilt2");
% xlabel("$PWM [milliseconds]$",'Interpreter','latex');
% ylabel("$\theta_{pot} [graus]$",'Interpreter','latex');
% hold on
% 
% pwm = [1650*ones(size(dado_final,1),1);1700*ones( size(dado_final,1),1);1750*ones( size(dado_final,1),1);1800*ones( size(dado_final,1),1);1850*ones( size(dado_final,1),1);1900*ones( size(dado_final,1),1);1950*ones( size(dado_final,1),1);2000*ones( size(dado_final,1),1);2050*ones( size(dado_final,1),1);2100*ones( size(dado_final,1),1);2150*ones( size(dado_final,1),1);2200*ones( size(dado_final,1),1);2250*ones( size(dado_final,1),1);2300*ones( size(dado_final,1),1);2350*ones( size(dado_final,1),1)];
% f=[dado_final(:,1);dado_final(:,2);dado_final(:,3);dado_final(:,4);dado_final(:,5);dado_final(:,6);dado_final(:,7);dado_final(:,8);dado_final(:,9);dado_final(:,10);dado_final(:,11);dado_final(:,12);dado_final(:,13);dado_final(:,14);dado_final(:,15)];
% 
% %%Regress�o Linear sobre os dados
% format long
% coefs = polyfit(pwm,f,2);
% %coefs=round(coefs,6);
% pwm1=linspace(1650,2350,1000);
% plot(pwm1,polyval(coefs,pwm1));
% 
% leg1=legend('Dados experimentais')
% set(leg1,'Interpreter','latex')
% grid on
