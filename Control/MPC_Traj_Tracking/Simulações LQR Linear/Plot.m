close all
%% Figure 1 - system states - ATTITUDE

figure
subplot(2,2,1)
plot([1:kend].*Ts,xp(1,1:end-1).*180/pi, 'LineWidth',2)

text('Units','normalized','String','(a)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$P$~(deg/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,3)
plot([1:kend].*Ts,xp(2,1:end-1).*180/pi, 'LineWidth',2)

text('Units','normalized','String','(c)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$Q$~(deg/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,2)
plot([1:kend].*Ts,xp(3,1:end-1).*180/pi, 'LineWidth',2)

text('Units','normalized','String','(b)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\phi$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

subplot(2,2,4)
plot([1:kend].*Ts,xp(4,1:end-1).*180/pi, 'LineWidth',2)

text('Units','normalized','String','(d)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\theta$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])




%% Figure 4 - controles

figure
subplot(2,2,1)
plot([1:kend].*Ts, ulqr(1,:), 'LineWidth',2)

text('Units','normalized','String','(a)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\Omega_1$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,3)
plot([1:kend].*Ts, ulqr(2,:), 'LineWidth',2)

text('Units','normalized','String','(c)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\Omega_2$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,2)
plot([1:kend].*Ts, ulqr(3,:), 'LineWidth',2)

text('Units','normalized','String','(b)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\Omega_3$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

subplot(2,2,4)
plot([1:kend].*Ts, ulqr(4,:), 'LineWidth',2)

text('Units','normalized','String','(d)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\Omega_4$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

