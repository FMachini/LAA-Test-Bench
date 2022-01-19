%LQRi - Controle de atitude e altura(theta24)

function [] = plotdata(w,y,u,x,xref,ylin,ulin,xlin,k)

data=readExpData('ensaio1.txt');

figure
subplot(2,3,1);
plot((0:k-1),y(1,:)','b');
xlabel('k');
ylabel('$\theta_1 [graus]$ ','Interpreter','latex');
grid on
hold on
plot((0:k-1),ylin(1,:)','m');

plot([1:size(data.y,1)],data.y(:,1),'r');
xticks([0:350:1500])

plot((0:k-1),xref(1,:)','k--');

legend('\theta_1','\theta_1_{lin}','\theta_1_{exp]','referencia')
hold off



legend('\theta_1','\theta_1_{lin}','referencia')

subplot(2,3,2);
plot((0:k-1),y(2,:)','b');
xlabel('k');
ylabel('$\theta_2 [graus]$','Interpreter','latex');
grid on
hold on
plot((0:k-1),ylin(2,:)','m');

plot([1:size(data.y,1)],data.y(:,2),'r');
xticks([0:350:1500])

plot((0:k-1),xref(2,:)','k--');

legend('\theta_2','\theta_2_{lin}','\theta_2_{exp]','referencia')
hold off


subplot(2,3,3);
plot((0:k-1),y(3,:)','b');
xlabel('k');
ylabel('$\theta_4 [graus]$','Interpreter','latex');
grid on
hold on
plot((0:k-1),ylin(3,:)','m');


plot([1:size(data.y,1)],data.y(:,3),'r');
xticks([0:350:1500])

plot((0:k-1),xref(3,:)','k--');

legend('\theta_4','\theta_4_{lin}','\theta_4_{exp]','referencia')
hold off

subplot(2,3,4);
plot((0:k),x(6,:),'b');
xlabel('k');
ylabel('$\dot{\theta}_1[graus/s]$','Interpreter','latex');
grid on
hold on
plot((0:k),xlin(4,:)','m');

plot([1:size(data.y,1)],data.y(:,4),'r');
xticks([0:350:1500])

legend('n linear','linear','exp');
hold off


subplot(2,3,5);
plot((0:k),x(7,:),'r');
xlabel('k');
ylabel('$\dot{\theta}_2[graus/s]$','Interpreter','latex');
grid on
hold on
plot((0:k),xlin(5,:)','m');

plot([1:size(data.y,1)],data.y(:,5),'r');
xticks([0:350:1500])

legend('n linear','linear','exp');
hold off



subplot(2,3,6);
plot((0:k),x(8,:),'g');
xlabel('k');
ylabel('$\dot{\theta}_4[graus/s]$','Interpreter','latex');
grid on
hold on
plot((0:k),xlin(4,:)','m');

plot([1:size(data.y,1)],data.y(:,6),'r');
xticks([0:350:1500])

legend('n linear','linear','exp');
hold off



print('Theta124_States','-dpdf','-bestfit');

figure

subplot(3,2,[1 2]);
plot((0:k),x(4,:),'b');
xlabel('k');
ylabel('$ F_1 [N]$','Interpreter','latex');
grid on

hold on
plot((0:k-1),ulin(1,:),'m');
plot((0:k-1),u(1,:),'k--');

plot([0:size(data.u,1)-1],data.u(:,1),'m');
xticks([0:350:1500])

plot((0:k-1),ones(k)*1.3,'r--');
plot((0:k-1),ones(k)*2.9,'r--');

legend('n linear','linear','F requerida','Fexp','sat. inf','sat. sup');
hold off


subplot(3,2,[3 4]);
plot((0:k),x(5,:),'b');
xlabel('k');
ylabel('$ F_2 [N]$','Interpreter','latex');
grid on
hold on
plot((0:k-1),ulin(2,:),'m');
plot((0:k-1),u(2,:),'k--');


plot([0:size(data.u,1)-1],data.u(:,2),'m');
xticks([0:350:1500])

plot((0:k-1),ones(k)*1.3,'r--');
plot((0:k-1),ones(k)*2.9,'r--');

legend('n linear','linear','F requerida','Fexp','sat. inf','sat. sup');
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

subplot(3,2,6);
plot((0:k-1),w(2,:),'b');
xlabel('k');
ylabel('$ PWM_2 [N]$','Interpreter','latex');
grid on
hold on
plot((0:k),ones(k+1)*40,'r');
plot((0:k),ones(k+1)*60,'k');
legend('PWM aplicado','sat inf.','sat sup.');

print('Theta124_forces','-dpdf','-bestfit');

end

