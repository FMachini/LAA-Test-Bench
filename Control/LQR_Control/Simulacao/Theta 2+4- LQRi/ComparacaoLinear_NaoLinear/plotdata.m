%LQRi - Controle de atitude e altura(theta24)

function [] = plotdata(w,y,u,x,xref,ylin,ulin,xlin,k)
figure

subplot(2,2,1);
plot((0:k-1),y(1,:)','b');
xlabel('k');
ylabel('$\theta_2 [graus]$ ','Interpreter','latex');
grid on
hold on
plot((0:k-1),ylin(1,:)','m');
plot((0:k-1),xref(1,:)','k--');
legend('\theta_2','\theta_2_{lin}','referencia')



subplot(2,2,2);
plot((0:k-1),y(2,:)','k');
xlabel('k');
ylabel('$\theta_4 [graus]$','Interpreter','latex');
grid on
hold on
plot((0:k-1),ylin(2,:)','m');
plot((0:k-1),xref(2,:)','k--');
legend('\theta_4','\theta_4_{lin}','referencia')


subplot(2,2,3);
plot((0:k),x(3,:),'b');
xlabel('k');
ylabel('$\dot{\theta}_2[graus/s]$','Interpreter','latex');
grid on
hold on
plot((0:k),xlin(3,:)','m');
legend('n linear','linear');

subplot(2,2,4);
plot((0:k),x(4,:),'k');
xlabel('k');
ylabel('$\dot{\theta}_4[graus/s]$','Interpreter','latex');
grid on
hold on
plot((0:k),xlin(4,:)','m');
legend('n linear','linear');


print('Theta124_States','-dpdf','-bestfit');

figure


subplot(3,2,[1 2]);
plot((0:k),x(3,:),'b');
xlabel('k');
ylabel('$ F_1 [N]$','Interpreter','latex');
grid on
hold on
plot((0:k-1),u(1,:),'k--');
plot((0:k-1),ones(k)*1.3,'r--');
plot((0:k-1),ones(k)*2.9,'r--');
plot((0:k-1),ulin(1,:),'m');
legend('Força aplicada','Força requerida','Flinear','sat. inf','sat. sup');
hold off


subplot(3,2,[3 4]);
plot((0:k),x(4,:),'b');
xlabel('k');
ylabel('$ F_2 [N]$','Interpreter','latex');
grid on
hold on
plot((0:k-1),u(2,:),'k--');
plot((0:k-1),ulin(2,:),'m');
plot((0:k-1),ones(k)*1.3,'r--');
plot((0:k-1),ones(k)*2.9,'r--');
grid on
legend('Força aplicada','Força requerida','Flinear','sat. inf','sat. sup');
hold off


subplot(3,2,5);
plot((0:k-1),w(1,:),'b');
xlabel('k');
ylabel('$ PWM_1 [N]$','Interpreter','latex');
grid on
hold on
plot((0:k),ones(k+1)*40,'r');
plot((0:k),ones(k+1)*60,'k');
legend('PWM aplicado','sat inf.','sat sup.');
hold off

subplot(3,2,6);
plot((0:k-1),w(2,:),'b');
xlabel('k');
ylabel('$ PWM_2 [N]$','Interpreter','latex');
grid on
hold on
plot((0:k),ones(k+1)*40,'r');
plot((0:k),ones(k+1)*60,'k');
legend('PWM aplicado','sat inf.','sat sup.');
hold off

print('Theta124_forces','-dpdf','-bestfit');

end

