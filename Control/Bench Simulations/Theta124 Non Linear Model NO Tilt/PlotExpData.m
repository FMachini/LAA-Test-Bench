clear all;

%C:\Users\felip\OneDrive\Área de Trabalho\Machini\Doutorado\Bancada\MPC Aviation\18_03_21
close all
Ts= 0.02;
ftsize = 20;
rad2deg = 180/pi;
utrim = 1.7;
zZero = 18.7;

for i = 1:2
    
    arq = {'Theta124LQT_Notilt_Sim.txt','Theta124LQT_Notilt.txt'};
    leg = {'Simulation','Experimental Data'};
    %leg = {'Exp 1','Exp 2','Exp 3'};
    A = textread(arq{i});
    if i == 2 
        A(:,1:3) = A(:,1:3)*pi/180;
        utrim = 0;
        zZero = 0;
    end
    A(:,1:6) = A(:,1:6)*rad2deg;
    
    kend = size(A,1);
figure (1)

% Theta1 position
subplot(3,2,1)
hold on
plot([1:kend].*Ts,A(:,1), 'LineWidth',2)

text('Units','normalized','String','(a)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\theta_1$~(deg/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(3,2,2)
hold on
plot([1:kend].*Ts,A(:,4), 'LineWidth',2)

text('Units','normalized','String','(c)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\dot{\theta}_1$~(deg/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

% Theta4 position
subplot(3,2,5)
hold on
plot([1:kend].*Ts,A(:,3), 'LineWidth',2)

text('Units','normalized','String','(b)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\theta_4$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

subplot(3,2,6)
hold on
plot([1:kend].*Ts,A(:,6), 'LineWidth',2)

text('Units','normalized','String','(d)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\dot{\theta}_4$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])


% Theta2 position
% figure(6)
subplot(3,2,3)
hold on
plot([1:kend].*Ts, A(:,2)+zZero, 'LineWidth',2)

text('Units','normalized','String','(a)', 'Position',[0.85 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\theta_2$~(m/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

%ylim([-15 5])
xlim([0 kend*Ts])

subplot(3,2,4)
hold on
plot([1:kend].*Ts, A(:,5), 'LineWidth',2)

text('Units','normalized','String','(b)', 'Position',[0.85 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\dot{\theta}_2$~(m/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

%% Figure 3 - system states - Position Z


%% Figure 4 - controles

figure(4)
subplot(1,2,1)
hold on
plot([1:kend].*Ts, A(:,7)+utrim, 'LineWidth',2)

text('Units','normalized','String','(a)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$f_1$~(N)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(1,2,2)
hold on
plot([1:kend].*Ts, A(:,8)+utrim, 'LineWidth',2)

text('Units','normalized','String','(c)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$F_2$~(N)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
set(gca,'FontSize',ftsize,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

% subplot(2,2,2)
% hold on
% plot([1:kend].*Ts, A(:,9), 'LineWidth',2)
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
% plot([1:kend].*Ts, A(:,10), 'LineWidth',2)
% 
% text('Units','normalized','String','(d)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');
% 
% ylabel('$\alpha_2$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% xlabel('Time (s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
% set(gca,'FontSize',ftsize,'FontName','Times'), grid on


end