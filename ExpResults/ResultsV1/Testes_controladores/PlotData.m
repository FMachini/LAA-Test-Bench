close all
clear all; clc;
ftsize = 20;
Ts = 0.02;
%% Figure 1 - system states - ATTITUDE
for i = 1:2
    arq = {'Teste_controle_no_control2.txt', 'Teste_controle_LQR.txt'};
    %leg = 'Constrained';
    leg = {'No Control','LQR Control','With tilt dynamics'};
    A = textread(arq{i});
    xp = A(:,1:6);
    up = A(:,7:8);
    kend = size(A,1);
figure (2)
subplot(1,2,1)
hold on
plot([1:kend].*Ts,xp(:,1), 'LineWidth',2)

%text('Units','normalized','String','(a)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\theta_4$~(deg/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(1,2,2)
hold on
plot([1:kend].*Ts,xp(:,2), 'LineWidth',2)

%text('Units','normalized','String','(c)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\dot{\theta}_4$~(deg/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-15 5])
% xlim([0 kend*Ts])
% 
% subplot(2,2,2)
% hold on
% plot([1:kend].*Ts,xp(3,1:end-1).*180/pi, 'LineWidth',2)
% 
% text('Units','normalized','String','(b)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\theta_1$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
% set(gca,'FontSize',ftsize,'FontName','Times'), grid on
% 
% % ylim([-5 20])
% xlim([0 kend*Ts])
% 
% subplot(2,2,4)
% hold on
% plot([1:kend].*Ts,xp(4,1:end-1).*180/pi, 'LineWidth',2)
% 
% text('Units','normalized','String','(d)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\theta^{dot}_1$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
% set(gca,'FontSize',ftsize,'FontName','Times'), grid on
% 
% % ylim([-5 20])
% xlim([0 kend*Ts])
% 
% 
% %% Figure 2 - system states - Position X,Y
% figure(3)
% subplot(2,3,1)
% hold on
% plot([1:kend].*Ts, xp(5,1:end-1)*180/pi, 'LineWidth',2)
% 
% text('Units','normalized','String','(a)', 'Position',[0.85 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\theta_2$~(m/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
% set(gca,'FontSize',ftsize,'FontName','Times'), grid on
% 
% %ylim([-15 5])
% xlim([0 kend*Ts])
% 
% subplot(2,3,2)
% hold on
% plot([1:kend].*Ts, xp(6,1:end-1)*180/pi, 'LineWidth',2)
% 
% text('Units','normalized','String','(b)', 'Position',[0.85 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\theta^{dot}_2$~(m/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
% set(gca,'FontSize',ftsize,'FontName','Times'), grid on
% 
% % ylim([-15 5])
% 
% 
% %% Figure 3 - system states - Position Z
% 
% 
% %% Figure 4 - controles
% 
% figure(4)
% subplot(2,2,1)
% hold on
% plot([1:kend].*Ts, up(1,:), 'LineWidth',2)
% 
% text('Units','normalized','String','(a)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');
% 
% ylabel('$f_1$~(N)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
% set(gca,'FontSize',ftsize,'FontName','Times'), grid on
% 
% % ylim([-15 5])
% xlim([0 kend*Ts])
% 
% subplot(2,2,3)
% hold on
% plot([1:kend].*Ts, up(2,:), 'LineWidth',2)
% 
% text('Units','normalized','String','(c)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');
% 
% ylabel('$F_2$~(N)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
% set(gca,'FontSize',ftsize,'FontName','Times'), grid on
% 
% % ylim([-15 5])
% xlim([0 kend*Ts])
% 
% subplot(2,2,2)
% hold on
% plot([1:kend].*Ts, up(3,:)*180/pi, 'LineWidth',2)
% 
% text('Units','normalized','String','(b)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\alpha_1$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
% set(gca,'FontSize',ftsize,'FontName','Times'), grid on
% 
% % ylim([-5 20])
% xlim([0 kend*Ts])
% 
% subplot(2,2,4)
% hold on
% plot([1:kend].*Ts, up(4,:)*180/pi, 'LineWidth',2)
% 
% text('Units','normalized','String','(d)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\alpha_2$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
% set(gca,'FontSize',ftsize,'FontName','Times'), grid on


end