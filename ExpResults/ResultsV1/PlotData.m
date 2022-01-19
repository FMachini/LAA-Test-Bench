clear all;close all;clc;
ftsize = 20;
A = textread('Teste_motor_ligado.txt');

kend = size(A,1);

figure (1)
subplot(2,2,1)
hold on
plot(A(:,1), 'LineWidth',2)

%text('Units','normalized','String','(a)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\phi$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('k','Interpreter','latex','FontSize',ftsize,'FontName','Times');
%legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
%set(gca,'FontSize',ftsize,'FontName','Times'), grid on


subplot(2,2,3)
hold on
plot(A(:,2), 'LineWidth',2)
hold on
plot(A(:,5),'r', 'LineWidth',2)
%text('Units','normalized','String','(c)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$p$~(deg/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('k','Interpreter','latex','FontSize',ftsize,'FontName','Times');
%legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
%set(gca,'FontSize',ftsize,'FontName','Times'), grid on


subplot(2,2,2)
hold on
plot(A(:,3), 'LineWidth',2)

%text('Units','normalized','String','(b)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\theta$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('k','Interpreter','latex','FontSize',ftsize,'FontName','Times');
%legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
%set(gca,'FontSize',ftsize,'FontName','Times'), grid on

subplot(2,2,4)
hold on
plot(A(:,4), 'LineWidth',2)

%text('Units','normalized','String','(d)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$q$~(deg/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('k','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
% set(gca,'FontSize',ftsize,'FontName','Times'), grid on

figure (2)
subplot(2,2,1)
hold on
plot(A(:,5), 'LineWidth',2)

%text('Units','normalized','String','(a)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\theta_1$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('k','Interpreter','latex','FontSize',ftsize,'FontName','Times');
%legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
%set(gca,'FontSize',ftsize,'FontName','Times'), grid on


subplot(2,2,3)
hold on
plot(A(:,6), 'LineWidth',2)

%text('Units','normalized','String','(c)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$dot{theta}_1$~(deg/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('k','Interpreter','latex','FontSize',ftsize,'FontName','Times');
%legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
%set(gca,'FontSize',ftsize,'FontName','Times'), grid on


subplot(2,2,2)
hold on
plot(A(:,7), 'LineWidth',2)

%text('Units','normalized','String','(b)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$\theta_2$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('k','Interpreter','latex','FontSize',ftsize,'FontName','Times');
%legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
%set(gca,'FontSize',ftsize,'FontName','Times'), grid on

subplot(2,2,4)
hold on
plot(A(:,8), 'LineWidth',2)

%text('Units','normalized','String','(d)', 'Position',[0.9 0.1 0], 'FontSize',ftsize,'FontName','Times','Interpreter','latex');

ylabel('$dot{theta}_2$~(deg/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('k','Interpreter','latex','FontSize',ftsize,'FontName','Times');
% legend(leg,'FontSize',16,'Interpreter','latex','FontName','Times')
% set(gca,'FontSize',ftsize,'FontName','Times'), grid on

