%close all
clear all;
%% Figure 1 - system states - ATTITUDE
for i = 1:2
    arq = {'Tilted_case.mat','No_Tilted_case2.mat'};
    %leg = 'Constrained';
    leg = {'Tilt','No Tilt','Constraint bounds'};
    load(arq{i});
figure (1)
subplot(2,2,1)
hold on
plot([1:kend].*Ts,xp(1,1:end-1).*180/pi, 'LineWidth',2)

text('Units','normalized','String','(a)', 'Position',[0.9 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$p$~(deg/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',26,'FontName','Times');
legend(leg,'FontSize',12,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,3)
hold on
plot([1:kend].*Ts,xp(2,1:end-1).*180/pi, 'LineWidth',2)

text('Units','normalized','String','(c)', 'Position',[0.9 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$q$~(deg/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,2)
hold on
plot([1:kend].*Ts,xp(3,1:end-1).*180/pi, 'LineWidth',2)

text('Units','normalized','String','(b)', 'Position',[0.9 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\phi$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

subplot(2,2,4)
hold on
plot([1:kend].*Ts,xp(4,1:end-1).*180/pi, 'LineWidth',2)

text('Units','normalized','String','(d)', 'Position',[0.9 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\theta$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])


%% Figure 2 - system states - Position X,Y
figure(6)
subplot(2,3,1)
hold on
plot([1:kend].*Ts, xp(5,1:end-1), 'LineWidth',2)

text('Units','normalized','String','(a)', 'Position',[0.85 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$u$~(m/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,3,2)
hold on
plot([1:kend].*Ts, xp(6,1:end-1), 'LineWidth',2)

text('Units','normalized','String','(b)', 'Position',[0.85 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$v$~(m/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,3,4)
hold on
plot([1:kend].*Ts, xp(8,1:end-1), 'LineWidth',2)

text('Units','normalized','String','(d)', 'Position',[0.85 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$x_E$~(m)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

subplot(2,3,5)
hold on
plot([1:kend].*Ts, xp(9,1:end-1), 'LineWidth',2)

text('Units','normalized','String','(e)', 'Position',[0.85 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$y_E$~(m)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

%% Figure 3 - system states - Position Z
%figure(3)
subplot(2,3,3)
hold on
plot([1:kend].*Ts, xp(7,1:end-1), 'LineWidth',2)

text('Units','normalized','String','(c)', 'Position',[0.85 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$w$~(m/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,3,6)
hold on
plot([1:kend].*Ts, xp(10,1:end-1), 'LineWidth',2)

text('Units','normalized','String','(f)', 'Position',[0.85 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$z_E$~(m)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])


%% Figure 4 - controles

figure(4)
subplot(2,2,1)
hold on
plot([1:kend].*Ts, up(1,:), 'LineWidth',2)

text('Units','normalized','String','(a)', 'Position',[0.9 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\Omega_1$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,3)
hold on
plot([1:kend].*Ts, up(2,:), 'LineWidth',2)

text('Units','normalized','String','(c)', 'Position',[0.9 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\Omega_2$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,2)
hold on
plot([1:kend].*Ts, up(3,:), 'LineWidth',2)

text('Units','normalized','String','(b)', 'Position',[0.9 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\Omega_3$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

subplot(2,2,4)
hold on
plot([1:kend].*Ts, up(4,:), 'LineWidth',2)

text('Units','normalized','String','(d)', 'Position',[0.9 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\Omega_4$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])


%% Figure 5 - controles tilt

% figure(5)
% subplot(2,2,1)
% hold on
% plot([1:kend].*Ts, up(5,:)*180/pi, 'LineWidth',2)
% 
% text('Units','normalized','String','(a)', 'Position',[0.9 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\alpha_1$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
% legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
% set(gca,'FontSize',26,'FontName','Times'), grid on
% 
% % ylim([-15 5])
% xlim([0 kend*Ts])
% 
% subplot(2,2,3)
% hold on
% plot([1:kend].*Ts, up(6,:)*180/pi, 'LineWidth',2)
% 
% text('Units','normalized','String','(c)', 'Position',[0.9 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\alpha_2$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
% legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
% set(gca,'FontSize',26,'FontName','Times'), grid on
% 
% % ylim([-15 5])
% xlim([0 kend*Ts])
% 
% subplot(2,2,2)
% hold on
% plot([1:kend].*Ts, up(7,:)*180/pi, 'LineWidth',2)
% 
% text('Units','normalized','String','(b)', 'Position',[0.9 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\alpha_3$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
% legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
% set(gca,'FontSize',26,'FontName','Times'), grid on
% 
% % ylim([-5 20])
% xlim([0 kend*Ts])
% 
% subplot(2,2,4)
% hold on
% plot([1:kend].*Ts, up(8,:)*180/pi, 'LineWidth',2)
% 
% text('Units','normalized','String','(d)', 'Position',[0.9 0.1 0], 'FontSize',26,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\alpha_4$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');
% legend(leg,'FontSize',14,'Interpreter','latex','FontName','Times')
% set(gca,'FontSize',26,'FontName','Times'), grid on
% 
% % ylim([-5 20])
% xlim([0 kend*Ts])
end
%legend({'No tilt', 'Tilt', 'Constraint bounds'},'FontSize',12,'Interpreter','latex','FontName','Times')

%%Plot boundaries

% figure(6)
% subplot(2,3,4)
% hold on
% plot([1:kend].*Ts, 1*ones(size([1:kend])),'--','Color','k','LineWidth',2)
% legend(leg,'FontSize',14,'Interpreter','latex','FontName','Times')
% 
% subplot(2,3,5)
% hold on
% plot([1:kend].*Ts, 1*ones(size([1:kend])),'--','Color','k','LineWidth',2)
% legend(leg,'FontSize',14,'Interpreter','latex','FontName','Times')
% 
% subplot(2,3,6)
% hold on
% plot([1:kend].*Ts, 1*ones(size([1:kend])),'--','Color','k','LineWidth',2)
% legend(leg,'FontSize',14,'Interpreter','latex','FontName','Times')
% 
% 
% figure(5)
% subplot(2,2,1)
% hold on
% plot([1:kend].*Ts, 25*ones(size([1:kend])),'--','Color','k','LineWidth',2)
% plot([1:kend].*Ts, -25*ones(size([1:kend])),'--','Color','k','LineWidth',2)
% legend(leg,'FontSize',14,'Interpreter','latex','FontName','Times')
% 
% subplot(2,2,2)
% hold on
% plot([1:kend].*Ts, 25*ones(size([1:kend])),'--','Color','k','LineWidth',2)
% plot([1:kend].*Ts, -25*ones(size([1:kend])),'--','Color','k','LineWidth',2)
% legend(leg,'FontSize',14,'Interpreter','latex','FontName','Times')
% 
% subplot(2,2,3)
% hold on
% plot([1:kend].*Ts, 25*ones(size([1:kend])),'--','Color','k','LineWidth',2)
% plot([1:kend].*Ts, -25*ones(size([1:kend])),'--','Color','k','LineWidth',2)
% legend(leg,'FontSize',14,'Interpreter','latex','FontName','Times')
% 
% subplot(2,2,4)
% hold on
% plot([1:kend].*Ts, 25*ones(size([1:kend])),'--','Color','k','LineWidth',2)
% plot([1:kend].*Ts, -25*ones(size([1:kend])),'--','Color','k','LineWidth',2)
% legend(leg,'FontSize',14,'Interpreter','latex','FontName','Times')
