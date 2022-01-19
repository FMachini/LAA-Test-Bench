clear all
close all

%(0.148+0.052)*9.81

ensaio1=importdata('ensaio01_m10.txt'); %[theta1,theta2,theta4,dottheta1,dottheta2,dottheta4,forca1,forca2]

theta4 = ensaio1(100:500,3)
theta4_dot = ensaio1(100:500,6)

Ts=0.02;

figure

subplot(2,1,1);
plot([0:size(theta4)-1]*Ts,theta4*-1,'Color','[0 0.4470 0.7410]')
ylabel('$\theta_4[rad]$','fontsize',14,'interpreter','latex')
xlabel('$Tempo [s]$','fontsize',14,'interpreter','latex')
grid on
hold on

subplot(2,1,2);
plot([0:size(theta4_dot)-1]*Ts,theta4_dot *-1,'Color','[0 0.4470 0.7410]')
ylabel('$\dot{\theta_4}[rad/s]$','fontsize',14,'interpreter','latex')
xlabel('$Tempo [s]$','fontsize',14,'interpreter','latex')
grid on
hold on


%Análise dos dados


%sobresinal
%sobresinal = (valor maximo - thetainf) / thetainf

valormax1=mean(theta4(153:162));
thetainf1=mean(theta4(390:400));
sobresinal_ens1=  (valormax1 - thetainf1)/thetainf1;
sobresinal=[sobresinal_ens1];
qsi = sqrt(log(sobresinal).^2)./sqrt(pi^2+(log(sobresinal_ens1)).^2);

%tempo de pico

tp1 = Ts*34;
tp=[tp1];


wn= pi./(tp* sqrt(1- (qsi.^2)));

% tamanho do erro inicial =  valor de referencia - valor inicial do ensaio 
% valor inicial do ensaio são a média dos 8 primeiros valores
ref=-10*pi/180; % -10 graus em rad

valorinicial_ens1=mean(theta4(1:10));
erro_ens1=  ref-valorinicial_ens1;
Amplitude=[erro_ens1];

K = thetainf1/Amplitude;

subplot(2,1,1);
H = tf(K(1)*(wn(1)^2),[1, 2*qsi(1)*wn(1), wn(1)^2]);
%opt = stepDataOptions('InputOffset',valorinicial_ens1,'StepAmplitude',Amplitude(1));
%step(H,opt,9)
[y,t]=step(H,9);
y=y.*Amplitude(1)+valorinicial_ens1;
plot(t,y,'b');
title('')
legend('Res. Exp','Res. Ident.')

%%sgtitle('Resultados Experimentais','fontsize',14,'interpreter','latex')

%print('ResultadosExpTheta4','-dpdf','-bestfit')

%[num,den] = ord2(wn,z) 
% transfar function & response of a 2nd order system
% qsi is tha damping factor
% wn is natural frequency
%num=[K*wn^2];
%den=[1 2*qsi*wn wn^2];
%G=tf(num,den)                  %% get transfar function
%subplot(2,2,1),step(G)      %% step response response of system


