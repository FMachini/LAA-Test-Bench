clear all 
close all
clc

Ts=0.02;
deg2rad=pi/180;

txtfile = dlmread('ensaio1.txt');
data = iddata(txtfile(:,1:6),txtfile(:,7:10),Ts,'ExperimentName','Ensaio Controle LQRi 124 com tilt');
data.OutputName = {'Theta1','Theta2','Theta4','Theta1_dot','Theta2_dot','Theta4_dot'};
data.OutputUnit = {'rad','rad','rad','rad/s','rad/s','rad/s',};
data.InputName = {'F1','F2','alpha1','alpha2'};
data.InputUnit = {'N','N','rad','rad'};



figure

subplot(4,3,[1 2 3]);

plot([1:size(data.y,1)],data.y(:,1),'b');
xticks([0:100:1500])
xlabel('k');
ylabel('$\theta_1 [graus]$ ','Interpreter','latex');
grid on

subplot(4,3,[4 5 6]);

plot([1:size(data.y,1)],data.y(:,2),'k');
xticks([0:100:1500])
xlabel('k');
ylabel('$\theta_2 [graus]$ ','Interpreter','latex');
grid on

subplot(4,3,[7 8 9]);

plot([1:size(data.y,1)],data.y(:,3),'r');
xticks([0:100:1500])
xlabel('k');
ylabel('$\theta_4 [graus]$ ','Interpreter','latex');
grid on

subplot(4,3,10);

plot([1:size(data.y,1)],data.y(:,4),'b');
xticks([0:250:1500])
xlabel('k');
ylabel('$\dot{\theta}_1 [graus]$ ','Interpreter','latex');
grid on

subplot(4,3,11);

plot([1:size(data.y,1)],data.y(:,5),'k');
xticks([0:250:1500])
xlabel('k');
ylabel('$\dot{\theta}_2 [graus]$ ','Interpreter','latex');
grid on

subplot(4,3,12);

plot([1:size(data.y,1)],data.y(:,6),'r');
xticks([0:250:1500])
xlabel('k');
ylabel('$\dot{\theta}_4 [graus]$ ','Interpreter','latex');
grid on


figure

subplot(2,2,1);

plot([1:size(data.u,1)],data.u(:,1),'k');
xticks([0:250:1500])
xlabel('k');
ylabel('$F_1 [N]$ ','Interpreter','latex');
grid on


hold on
plot((0:size(data.u,1)-1),ones(size(data.u,1))*2.9,'b--');
plot((0:size(data.u,1)-1),ones(size(data.u,1))*1.3,'r--');
legend('F_1','sat. inf','sat. sup')


subplot(2,2,2);

plot([1:size(data.u,1)],data.u(:,2),'k');
xticks([0:250:1500])
xlabel('k');
ylabel('$F_2 [N]$ ','Interpreter','latex');
grid on

hold on
plot((0:size(data.u,1)-1),ones(size(data.u,1))*2.9,'b--');
plot((0:size(data.u,1)-1),ones(size(data.u,1))*1.3,'r--');
legend('F_2','sat. inf','sat. sup')


subplot(2,2,3);

plot([0:size(data.u,1)-1],data.u(:,3),'k');
xticks([0:250:1500])
xlabel('k');
ylabel('$\alpha_1 [rad]$ ','Interpreter','latex');
grid on

hold on
plot((0:size(data.u,1)-1),ones(size(data.u,1))*35*deg2rad,'b--');
plot((0:size(data.u,1)-1),ones(size(data.u,1))*0*deg2rad,'r--');
legend('alpha_1','sat. inf','sat. sup')

subplot(2,2,4);

plot([0:size(data.u,1)-1],data.u(:,4),'k');
xticks([0:250:1500])
xlabel('k');
ylabel('$\alpha_2 [rad]$ ','Interpreter','latex');
grid on

hold on
plot((0:size(data.u,1)-1),ones(size(data.u,1))*35*deg2rad,'b--');
plot((0:size(data.u,1)-1),ones(size(data.u,1))*0*deg2rad,'r--');
legend('alpha_2','sat. inf','sat. sup')

