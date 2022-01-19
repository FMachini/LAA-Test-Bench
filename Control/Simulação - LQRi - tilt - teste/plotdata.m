%Gabriel Renato Oliveira Alves 
%LQRi - Controle de atitude (theta4)

function [] = plotdata(w,y,ym,u,x,k)
figure('units','normalized','outerposition',[0 0 1 1])
%grafico da saida yk
subplot(2,3,[1,2]);
plot((0:k-1),y');
hold on
plot((0:k-1),ym');
xlabel('k');
ylabel('\theta(k)');
leg1 = legend('$\theta_1 [graus]$ - real','$\theta_2 [graus]$ - real', '$\theta_4 [graus]$ - real','$\theta_1 [graus]$ - medido','$\theta_2 [graus]$ - medido', '$\theta_4 [graus]$ - medido');
set(leg1,'Interpreter','latex');
hold on
grid on
%grafico do controle wk
subplot(2,3,4);
plot((0:k-1),w');
xlabel('k');
ylabel('w (PWM)');
leg4 = legend('PWM-F1','PWM-F2','PWM-T1','PWM-T2');
set(leg4,'Interpreter','latex');
hold on
grid on

%grafico do controle uk
subplot(2,3,6);
plot((0:k),x(6,:)*180/pi',(0:k),x(7,:)*180/pi');
xlabel('k');
ylabel('alpha(k) [deg]');
leg3 = legend('x6 - $\alpha_1 [deg]$','x7 - $\alpha_2 [deg]$');
set(leg3,'Interpreter','latex');
hold on
grid on

subplot(2,3,5);
plot((0:k-1),u([1,2],:)',(0:k),x(4,:)',(0:k),x(5,:)');
xlabel('k');
ylabel('u(k)');
leg3 = legend('u1','u2','x4 - $F_1[N]$','x5 - $F_2[N]$');
set(leg3,'Interpreter','latex');
hold on
grid on

%grafico do estado xk
subplot(2,3,3);
plot((0:k),x(1,:)',(0:k),x(2,:)',(0:k),x(3,:)',(0:k),x(8,:)',(0:k),x(9,:)',(0:k),x(10,:)');
xlabel('k');
ylabel('x(k)');
leg2 = legend('x1 - $\theta_1 [rad]$','x2 - $\theta_2 [rad]$','x3 - $\theta_4 [rad]$','x8 - $\dot{\theta}_1[rad/s]$','x9 - $\dot{\theta}_2[rad/s]$','x10 - $\dot{\theta}_4[rad/s]$');
set(leg2,'Interpreter','latex');
hold on
grid on



end

