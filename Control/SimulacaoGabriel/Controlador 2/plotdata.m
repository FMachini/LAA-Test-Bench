function [] = plotdata(u,y, ym,x,k)
figure('units','normalized','outerposition',[0 0 1 1])
%grafico da saida yk
subplot(2,2,1);
plot((0:k-1),y');
hold on
plot((0:k-1),ym','r--');
title('Saida y(k)');
xlabel('k');
ylabel('\theta(k)');
leg1 = legend('$\theta_4 [graus]$ - real', '$\theta_4 [graus]$ - medido');
set(leg1,'Interpreter','latex');
hold on
grid on
%grafico do estado xk
subplot(2,2,3);
plot((0:k),x(1,:)',(0:k),x(4,:)');
title('Estados x(k)');
xlabel('k');
ylabel('x(k)');
leg2 = legend('Estado 1 - $\theta_4 [rad]$','Estado 4 - $\dot{\theta}_4[rad/s]$');
set(leg2,'Interpreter','latex');
hold on
grid on

subplot(2,2,4);
plot((0:k),x(2,:)',(0:k),x(3,:)',(0:k),x(5,:)',(0:k),x(6,:)');
title('Estados x(k)');
xlabel('k');
ylabel('x(k)');
leg3 = legend('Estado 2 - $F_1[N]$','Estado 3 - $F_2[N]$','Estado 5 - $\dot{F}_1[N/s]$','Estado 6 - $\dot{F}_2[N/s]$');
set(leg3,'Interpreter','latex');
hold on
grid on
%grafico do controle uk
subplot(2,2,2);
plot((0:k-1),u');
title('Acao de controle w(k)');
xlabel('k');
ylabel('w (PWM)');
leg4 = legend('PWM-1','PWM-2');
set(leg4,'Interpreter','latex');
hold on
grid on
end

