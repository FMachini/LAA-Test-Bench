clc
clear all
close all
format long
%(0.148+0.052)*9.81

ensaio1=importdata('ensaio07_5.txt'); %[theta1,theta2,theta4,dottheta1,dottheta2,dottheta4,forca1,forca2]

ensaio2=importdata('ensaio08_5.txt'); %[theta1,theta2,theta4,dottheta1,dottheta2,dottheta4,forca1,forca2]

ensaio3=importdata('ensaio05_5.txt'); %[theta1,theta2,theta4,dottheta1,dottheta2,dottheta4,forca1,forca2]

ensaio4=importdata('ensaio06_5.txt'); %[theta1,theta2,theta4,dottheta1,dottheta2,dottheta4,forca1,forca2]

ensaio1= ensaio1(:,2);
ensaio2= ensaio2(:,2);
ensaio3= ensaio3(:,2);
ensaio4= ensaio4(:,2);

Ts=0.02; % 20 milisegundos

figure
%sgtitle('Resultados Experimentais','fontsize',14,'interpreter','latex')
subplot(4,1,1);
plot([0:size(ensaio1)-1]*Ts,ensaio1,'Color',[0.8500 0.3250 0.0980])
ylabel('$\theta_2[rad]$','fontsize',14,'interpreter','latex')
xlabel('$Tempo [s]$','fontsize',14,'interpreter','latex')
grid on
hold on

subplot(4,1,2);
plot([0:size(ensaio2)-1]*Ts,ensaio2,'Color',[0.4660 0.6740 0.1880])
ylabel('$\theta_2[rad]$','fontsize',14,'interpreter','latex')
xlabel('$Tempo [s]$','fontsize',14,'interpreter','latex')
grid on
hold on

subplot(4,1,3);
plot([0:size(ensaio3)-1]*Ts,ensaio3,'Color',[0.3 0.05 0.5410])
ylabel('$\theta_2[rad]$','fontsize',14,'interpreter','latex')
xlabel('$Tempo [s]$','fontsize',14,'interpreter','latex')
grid on
hold on

subplot(4,1,4);
plot([0:size(ensaio4)-1]*Ts,ensaio4,'r')
ylabel('$\theta_2[rad]$','fontsize',14,'interpreter','latex')
xlabel('$Tempo [s]$','fontsize',14,'interpreter','latex')
grid on
hold on

%Análise dos dados


%sobresinal
%sobresinal = (valor maximo - thetainf) / thetainf

valormax1=mean(ensaio1(31:37));
valormax2=mean(ensaio2(47:52));
valormax3=mean(ensaio3(54:58));
valormax4=mean(ensaio4(51:55));

valormax=[valormax1 valormax2 valormax3 valormax4];

thetainf1=mean(ensaio1(450:500));
thetainf2=mean(ensaio2(450:500));
thetainf3=mean(ensaio3(450:500));
thetainf4=mean(ensaio4(450:500));

thetainf=[thetainf1 thetainf2 thetainf3 thetainf4];

sobresinal_ens1=  (valormax1 - thetainf1)/thetainf1;
sobresinal_ens2=  (valormax2 - thetainf2)/thetainf2;
sobresinal_ens3=  (valormax3 - thetainf3)/thetainf3;
sobresinal_ens4=  (valormax4 - thetainf4)/thetainf4;

sobresinal = (valormax-thetainf)./thetainf;

qsi = sqrt(log(sobresinal).^2)./sqrt(pi^2+(log(sobresinal)).^2);

%tempo de pico

tp1 = Ts*34;
tp2 = Ts*49.5;
tp3 = Ts*56;
tp4 = Ts*53; 

tp=[tp1 tp2 tp3 tp4];

wn = pi./(tp.* sqrt(1- (qsi.^2)));

% tamanho do erro inicial =  valor de referencia - valor inicial do ensaio 
% valor inicial do ensaio são a média dos 8 primeiros valores
ref=5*pi/180; % 5 graus em rad

valorinicial_ens1=mean(ensaio1(1:2));
valorinicial_ens2=mean(ensaio2(1:2));
valorinicial_ens3=mean(ensaio3(1:2));
valorinicial_ens4=mean(ensaio4(1:2));


erro_ens1=  ref-valorinicial_ens1;
erro_ens2=  ref-valorinicial_ens2;
erro_ens3=  ref-valorinicial_ens3;
erro_ens4=  ref-valorinicial_ens4;

Amplitude=[erro_ens1 erro_ens2 erro_ens3 erro_ens4];

K = thetainf./Amplitude;
%deveria ser 1 mas tem nao linearidades
K=1.13*ones(1,4);


subplot(4,1,1);
H = tf(K(1)*(wn(1)^2),[1, 2*qsi(1)*wn(1), wn(1)^2]);
%opt = stepDataOptions('InputOffset',valorinicial_ens1,'StepAmplitude',Amplitude(1));
%step(H,opt,9)
[y,t]=step(H,9);
y=y.*Amplitude(1)+valorinicial_ens1;
plot(t,y,'b');
title('')
legend('Res. Exp','Res. Ident.')

subplot(4,1,2);
H = tf(K(2)*(wn(2)^2),[1, 2*qsi(2)*wn(2), wn(2)^2]);
%opt = stepDataOptions('InputOffset',valorinicial_ens2,'StepAmplitude',Amplitude(2));
%step(H,opt,9)
[y,t]=step(H,9);
y=y.*Amplitude(2)+valorinicial_ens2;
plot(t,y,'b');
title('')
legend('Res. Exp','Res. Ident.')

subplot(4,1,3);
H = tf(K(3)*(wn(3)^2),[1, 2*qsi(3)*wn(3), wn(3)^2]);
%opt = stepDataOptions('InputOffset',valorinicial_ens3,'StepAmplitude',Amplitude(3));
%step(H,opt,9)
[y,t]=step(H,9);
y=y.*Amplitude(3)+valorinicial_ens3;
plot(t,y,'b');
title('')
legend('Res. Exp','Res. Ident.')

subplot(4,1,4);
H = tf(K(4)*(wn(4)^2),[1, 2*qsi(4)*wn(4), wn(4)^2]);
%opt = stepDataOptions('InputOffset',valorinicial_ens4,'StepAmplitude',Amplitude(4));
%step(H,opt,9)
[y,t]=step(H,9);
y=y.*Amplitude(4)+valorinicial_ens4;
plot(t,y,'b');
title('')
legend('Res. Exp','Res. Ident.')

l2=0.60;
Kp=3;

%%Calculo dos parâmetros
% 1/J1 b 1/J2  
%A=[2*Kp*l2 0 0; 0 1 0; 0 0 1];
%B=[wn.^2;2.*qsi.*wn;K.*wn.^2];


J2= 2*Kp*l2./(K.*wn.^2)

b2=1./(2.*qsi.*wn.*J2)

J22=(2*Kp*l2)./wn.^2
