close all
clear all;
clc;
ftsize = 20;
%% Figure 1 - system states - ATTITUDE
for i = [25]
    %arq = {'Results/MPC_Data_.txt','Results/MPC_Data_5.txt','Results/MPC_Data_6.txt'};
    %leg = 'Constrained';
    leg = strcat('Exp',num2str(i));
    A = textread(strcat('Results\MPC_Data_V',num2str(i),'.txt'));
    kend = size(A,1);
    Ts = 1;
figure (1)
subplot(2,2,1)
hold on
plot([1:kend].*Ts,A(:,3), 'LineWidth',2)

text('Units','normalized','String','(a)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\theta_4$~(deg/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,3)
hold on
plot([1:kend].*Ts,A(:,6), 'LineWidth',2)

text('Units','normalized','String','(c)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\theta^{dot}_4$~(deg/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,2)
hold on
plot([1:kend].*Ts,A(:,1), 'LineWidth',2)

text('Units','normalized','String','(b)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\theta_1$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

subplot(2,2,4)
hold on
plot([1:kend].*Ts,A(:,4), 'LineWidth',2)

text('Units','normalized','String','(d)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\theta^{dot}_1$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])


%% Figure 2 - system states - Position X,Y
figure(6)
subplot(2,3,1)
hold on
plot([1:kend].*Ts, A(:,2), 'LineWidth',2)

text('Units','normalized','String','(a)', 'Position',[0.85 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\theta_2$~(m/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

%ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,3,2)
hold on
plot([1:kend].*Ts, A(:,5), 'LineWidth',2)

text('Units','normalized','String','(b)', 'Position',[0.85 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\theta^{dot}_2$~(m/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-15 5])


%% Figure 3 - system states - Position Z


%% Figure 4 - controles

figure(4)
subplot(2,2,1)
hold on
plot([1:kend].*Ts, A(:,7), 'LineWidth',2)

text('Units','normalized','String','(a)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$f_1$~(N)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,3)
hold on
plot([1:kend].*Ts, A(:,8), 'LineWidth',2)

text('Units','normalized','String','(c)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$F_2$~(N)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,2)
hold on
plot([1:kend].*Ts, A(:,9), 'LineWidth',2)

text('Units','normalized','String','(b)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\alpha_1$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

subplot(2,2,4)
hold on
plot([1:kend].*Ts, A(:,10), 'LineWidth',2)

text('Units','normalized','String','(d)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\alpha_2$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on


end