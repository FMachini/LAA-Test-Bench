clear all; close all; clc;

load 'NonLinOut.mat';
kend = size(x,2)-1;
Ts = 0.02;
plotData

clear x uctr;

data = load('Theta124LQT_Notilt.txt');

xp = data(:,[1 2 3 4 5 6]);
%xAux(:,[1 2 3]) = xAux(:,[1 2 3])*deg2rad;
xp(:,[4 5 6]) = xp(:,[4 5 6])*180/pi;
xp = xp';
up = data(:,[7 8]);
up = up';

up(:,[3 4]) = up(:,[3 4]);

%time = data(:,11);

figure(1)
subplot(2,2,1)
hold on
plot([1:kend].*Ts,xp(3,1:end), 'LineWidth',2)

%text('Units','normalized','String','(a)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

%ylabel('$\theta_1$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
%xlabel('$Time~(s)$','Interpreter','latex','FontSize',26,'FontName','Times');

%set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
%xlim([0 kend*Ts])

subplot(2,2,3)
plot([1:kend].*Ts,xp(6,1:end), 'LineWidth',2)

% text('Units','normalized','String','(c)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\theta_2$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
% xlabel('Time~(s)','Interpreter','latex','FontSize',26,'FontName','Times');
% 
% set(gca,'FontSize',26,'FontName','Times'), grid on
% 
% % ylim([-15 5])
% xlim([0 kend*Ts])

subplot(2,2,2)
plot([1:kend].*Ts,xp(1,1:end), 'LineWidth',2)

% text('Units','normalized','String','(b)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\theta_4$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
% xlabel('Time~(s)','Interpreter','latex','FontSize',26,'FontName','Times');
% 
% set(gca,'FontSize',26,'FontName','Times'), grid on
% %set(gcf, 'Color', 'None')
% 
% % ylim([-5 20])
% xlim([0 kend*Ts])

subplot(2,2,4)
plot([1:kend].*Ts,xp(4,1:end), 'LineWidth',2)

% text('Units','normalized','String','(d)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\dot{\theta}_1$~(deg/s)','Interpreter','latex','FontSize',26,'FontName','Times');
% xlabel('Time~(s)','Interpreter','latex','FontSize',26,'FontName','Times');
% 
% set(gca,'FontSize',26,'FontName','Times'), grid on
% %set(gcf, 'Color', 'None')
% 
% % ylim([-5 20])
% xlim([0 kend*Ts])
% 

%% Figure 2 - system states - Position X,Y
figure(2)
subplot(1,2,1)
hold on
plot([1:kend].*Ts, xp(2,1:end), 'LineWidth',2)

% text('Units','normalized','String','(a)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\dot{\theta}_2~(deg/s)$','Interpreter','latex','FontSize',26,'FontName','Times');
% xlabel('Time~(s)','Interpreter','latex','FontSize',26,'FontName','Times');
% 
% set(gca,'FontSize',26,'FontName','Times'), grid on
% 
% % ylim([-15 5])
% xlim([0 kend*Ts])

subplot(1,2,2)
hold on
plot([1:kend].*Ts, xp(4,1:end), 'LineWidth',2)

% text('Units','normalized','String','(c)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\dot{\theta}_4~(deg/s)$','Interpreter','latex','FontSize',26,'FontName','Times');
% xlabel('Time~(s)','Interpreter','latex','FontSize',26,'FontName','Times');
% 
% set(gca,'FontSize',26,'FontName','Times'), grid on
% 
% % ylim([-15 5])
% xlim([0 kend*Ts])

% subplot(2,2,2)
% plot([1:kend].*Ts, xp(10,1:end-1), 'LineWidth',2)
% 
% text('Units','normalized','String','(b)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');
% 
% ylabel('$X$~(m)','Interpreter','latex','FontSize',26,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
% 
% set(gca,'FontSize',26,'FontName','Times'), grid on
% 
% % ylim([-5 20])
% xlim([0 kend*Ts])
% 
% subplot(2,2,4)
% plot([1:kend].*Ts, xp(11,1:end-1), 'LineWidth',2)
% 
% text('Units','normalized','String','(d)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');
% 
% ylabel('$Y$~(m)','Interpreter','latex','FontSize',26,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
% 
% set(gca,'FontSize',26,'FontName','Times'), grid on
% 
% % ylim([-5 20])
% xlim([0 kend*Ts])




%% Figure 4 - controles

figure(3)
subplot(2,2,1)
plot([1:kend].*Ts, up(1,:)-1.7, 'LineWidth',2)

% text('Units','normalized','String','(a)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\Omega_1$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
% 
% set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,3)
plot([1:kend].*Ts, up(2,:)-1.7, 'LineWidth',2)

% text('Units','normalized','String','(c)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\Omega_2$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
% 
% set(gca,'FontSize',26,'FontName','Times'), grid on
% 
% % ylim([-15 5])
% xlim([0 kend*Ts])

% subplot(2,2,2)
% plot([1:kend].*Ts, up(3,:)*180/pi, 'LineWidth',2)
% 
% % text('Units','normalized','String','(b)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');
% % 
% % ylabel('$\Omega_3$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
% % xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
% % 
% % set(gca,'FontSize',26,'FontName','Times'), grid on
% % 
% % % ylim([-5 20])
% % xlim([0 kend*Ts])
% 
% subplot(2,2,4)
% plot([1:kend].*Ts, up(4,:)*180/pi, 'LineWidth',2)
% 
% % text('Units','normalized','String','(d)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');
% % 
% % ylabel('$\Omega_4$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
% % xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
% % 
% % set(gca,'FontSize',26,'FontName','Times'), grid on
% % 
% % ylim([-5 20])
% xlim([0 kend*Ts])
% 
% 
