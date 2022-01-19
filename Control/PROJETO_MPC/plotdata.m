function [] = plotdata(w,y,ym,x,u,du,k)
figure('units','normalized','outerposition',[0 0 1 1])
%grafico da saida yk
subplot(3,2,1);
plot((0:k-1),y');
hold on
plot((0:k-1),ym','r');
ylabel('y');
leg1 = legend('$\theta_4 [graus]$ - real', '$\theta_4 [graus]$ - medido');
set(leg1,'Interpreter','latex');
hold on
grid on

%grafico do controle wk - PWM
subplot(3,2,2);
plot((0:k-1),w');
ylabel('w - PWM');
leg4 = legend('PWM-1','PWM-2');
set(leg4,'Interpreter','latex');
hold on
grid on

%grafico do estado xk
subplot(3,2,3);
plot((0:k),x(1,:)',(0:k),x(4,:)');
ylabel('x');
leg2 = legend('Estado 1 - $\theta_4 [rad]$','Estado 4 - $\dot{\theta}_4[rad/s]$');
set(leg2,'Interpreter','latex');
hold on
grid on

subplot(3,2,4);
plot((0:k),x(2,:)',(0:k),x(3,:)',(0:k),x(5,:)',(0:k),x(6,:)');
ylabel('x');
leg3 = legend('Estado 2 - $F_1[N]$','Estado 3 - $F_2[N]$','Estado 5 - $\dot{F}_1[N/s]$','Estado 6 - $\dot{F}_2[N/s]$');
set(leg3,'Interpreter','latex');
hold on
grid on

%grafico do controle uk - forca
subplot(3,2,5);
plot((0:k-1),u',(0:k),x(2,:)',(0:k),x(3,:)');
ylabel('u - Forca');
xlabel('k');
leg4 = legend('F1','F2','x2','x3');
set(leg4,'Interpreter','latex');
hold on
grid on

%grafico do incrememto de controle du
subplot(3,2,6);
plot((0:k-1),du');
xlabel('k');
ylabel('du - Forca/s');
leg4 = legend('du-1','du-2');
set(leg4,'Interpreter','latex');
hold on
grid on
end

