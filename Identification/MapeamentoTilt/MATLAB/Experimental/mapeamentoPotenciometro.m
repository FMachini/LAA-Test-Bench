%% Mapeamento do tilt rotor
clear all
close all


% Mapeamento est�tico



% Resultado dos ensaios
% ADC com alimenta��o 5 V
% Tilt do bra�o maior marcado como m1
% potenciometro

y1 = 50:5:140;
x1 = [ 11 30 49 69 88 108 126 146 165 184 202 224 242 263 282 302 320 340 357];

coefs=polyfit(x1,y1,1);

f= polyval(coefs,x1) ;

plot(x1,y1,'r*');
hold on
plot(x1,f,'b');


%Tilt 1 
% 0 graus no tilt vale 61,22 no pot
% 41 graus no tilt vale 98,71 no pot

% A=[61.50 1; 98.71 1];
% B=[0;41];
% 
% coefs1=inv(A)*B
% 
% figure
% 
% Ts=0.02;
% txtfile = dlmread('ensaiotilt2_01.txt');
% data1 = iddata([txtfile(:,3) txtfile(:,4)],[txtfile(:,1) txtfile(:,2)],Ts,'ExperimentName','COMPORTAMENTO Do TILT');
% data1.OutputName = {'CAD','Angle'};
% data1.OutputUnit = {'0-1024','degrees'};
% data1.InputName = {'Time','PWM'};
% data1.InputUnit = {'seconds','milliseconds'};
% 
% 
% coefs=polyfit(data1.y(:,1),data1.y(:,2),1);
% 
% f= polyval(coefs,data1.y(:,1)) ;
% 
% plot(data1.y(:,1),data1.y(:,2),'r*');
% hold on
% plot(data1.y(:,1),f,'b');
% 
% xlabel('CAD [0-1024]');
% ylabel('theta_tilt [graus]');
% 
% figure
% 
% 
% coefs=polyfit(data1.u(:,2),data1.y(:,2),1);
% 
% f= polyval(coefs,data1.u(:,2)) ;
% 
% plot(data1.u(:,2),data1.y(:,2),'r*');
% hold on
% plot(data1.u(:,2),f,'b');
% xlabel('PWM [milliseconds]');
% ylabel('theta_tilt [graus]');

