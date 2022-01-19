close all
% xplanta MPC = [P Q phi theta u v w X Y Z]
%% Figure 1 - system states - ATTITUDE

figure
subplot(2,2,1)
plot([1:kend].*Ts,xp_lin(1,1:end-1).*180/pi, 'LineWidth',2)

text('Units','normalized','String','(a)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$P$~(deg/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,3)
plot([1:kend].*Ts,xp_lin(2,1:end-1).*180/pi, 'LineWidth',2)

text('Units','normalized','String','(c)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$Q$~(deg/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,2)
plot([1:kend].*Ts,xp_lin(3,1:end-1).*180/pi, 'LineWidth',2)

text('Units','normalized','String','(b)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\phi$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

subplot(2,2,4)
plot([1:kend].*Ts,xp_lin(4,1:end-1).*180/pi, 'LineWidth',2)

text('Units','normalized','String','(d)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\theta$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])


%% Figure 2 - system states - Position X,Y
figure
subplot(2,2,1)
plot([1:kend].*Ts, xp_lin(5,1:end-1), 'LineWidth',2)

text('Units','normalized','String','(a)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$U$~(m/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,3)
plot([1:kend].*Ts, xp_lin(6,1:end-1), 'LineWidth',2)

text('Units','normalized','String','(c)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$V$~(m/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,2)
plot([1:kend].*Ts, xp_lin(8,1:end-1), 'LineWidth',2)

text('Units','normalized','String','(b)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$X$~(m)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

subplot(2,2,4)
plot([1:kend].*Ts, xp_lin(9,1:end-1), 'LineWidth',2)

text('Units','normalized','String','(d)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$Y$~(m)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

%% Figure 3 - system states - Position Z
figure
subplot(1,2,1)
plot([1:kend].*Ts, xp_lin(7,1:end-1), 'LineWidth',2), hold on


text('Units','normalized','String','(a)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$W$~(m/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(1,2,2)
plot([1:kend].*Ts, xp_lin(10,1:end-1), 'LineWidth',2)

text('Units','normalized','String','(b)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$Z$~(m)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])


%% Figure 4 - controles

figure
subplot(2,2,1)
plot([1:kend].*Ts, up(1,:), 'LineWidth',2)

text('Units','normalized','String','(a)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\Omega_1$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,3)
plot([1:kend].*Ts, up(2,:), 'LineWidth',2)

text('Units','normalized','String','(c)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\Omega_2$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,2)
plot([1:kend].*Ts, up(3,:), 'LineWidth',2)

text('Units','normalized','String','(b)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\Omega_3$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

subplot(2,2,4)
plot([1:kend].*Ts, up(4,:), 'LineWidth',2)

text('Units','normalized','String','(d)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\Omega_4$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])


%% Figure 5 - controles tilt

figure
subplot(2,2,1)
plot([1:kend].*Ts, up(5,:)*180/pi, 'LineWidth',2)

text('Units','normalized','String','(a)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\alpha_1$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,3)
plot([1:kend].*Ts, up(6,:)*180/pi, 'LineWidth',2)

text('Units','normalized','String','(c)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\alpha_2$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-15 5])
xlim([0 kend*Ts])

subplot(2,2,2)
plot([1:kend].*Ts, up(7,:)*180/pi, 'LineWidth',2)

text('Units','normalized','String','(b)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\alpha_3$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

subplot(2,2,4)
plot([1:kend].*Ts, up(8,:)*180/pi, 'LineWidth',2)

text('Units','normalized','String','(d)', 'Position',[0.027 0.93 0], 'FontSize',26,'FontName','Times','Interpreter','latex');

ylabel('$\alpha_4$~(deg)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on

% ylim([-5 20])
xlim([0 kend*Ts])

%% epsilon
figure
plot([1:kend].*Ts, umpc(5,:), 'LineWidth',2)


ylabel('$\varepsilon$~(rad/s)','Interpreter','latex','FontSize',26,'FontName','Times');
xlabel('Time (s)','Interpreter','latex','FontSize',26,'FontName','Times');

set(gca,'FontSize',26,'FontName','Times'), grid on


xlim([0 kend*Ts])