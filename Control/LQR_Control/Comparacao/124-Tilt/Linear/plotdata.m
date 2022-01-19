%LQRi - Controle de atitude e altura(theta24)

function [] = plotdata(y,u,x,referencias,k)

data=readExpData('ensaio1.txt');


deg2rad= pi/180;

figure

subplot(4,3,[1 2 3]);
plot((0:k-1),y(1,:)','b');
xlabel('k');
ylabel('$\theta_1 [graus]$ ','Interpreter','latex');
grid on
hold on

plot([1:size(data.y,1)],data.y(:,1),'r');
xticks([0:100:1500])
xlabel('k');
plot((0:k-1),referencias(1,:)','k--');
legend('\theta_1','\theta_1_{exp]','referencia')
hold off

subplot(4,3,[4 5 6]);
plot((0:k-1),y(2,:)','k');
xlabel('k');
ylabel('$\theta_2 [graus]$ ','Interpreter','latex');
grid on
hold on
plot([1:size(data.y,1)],data.y(:,2),'r');
xticks([0:100:1500])
xlabel('k');
plot((0:k-1),referencias(2,:)','k--');
legend('\theta_2','\theta_2_{exp]','referencia')
hold off


subplot(4,3,[7 8 9]);
plot((0:k-1),y(3,:)','r');
xlabel('k');
ylabel('$\theta_4 [graus]$','Interpreter','latex');
grid on
hold on
plot([1:size(data.y,1)],data.y(:,3),'r');
xticks([0:100:1500])
xlabel('k');
plot((0:k-1),referencias(3,:)','k--');
legend('\theta_4','\theta_4_{exp]','referencia')
hold off

subplot(4,3,10);
plot((0:k),x(4,:),'b');
xlabel('k');
ylabel('$\dot{\theta}_1[graus/s]$','Interpreter','latex');
grid on
hold on
plot([1:size(data.y,1)],data.y(:,4),'r');
xticks([0:250:1500])


legend('\dot{\theta}_1','\dot{\theta}_1_{exp]')
hold off


subplot(4,3,11);
plot((0:k),x(5,:),'k');
xlabel('k');
ylabel('$\dot{\theta}_2[graus/s]$','Interpreter','latex');
grid on
hold on
plot([1:size(data.y,1)],data.y(:,5),'r');
xticks([0:250:1500])

legend('\dot{\theta}_2','\dot{\theta}_2_{exp]')
hold off

subplot(4,3,12);
plot((0:k),x(6,:),'r');
xlabel('k');
ylabel('$\dot{\theta}_4[graus/s]$','Interpreter','latex');
grid on
hold on
plot([1:size(data.y,1)],data.y(:,6),'r');
xticks([0:250:1500])

legend('\dot{\theta}_4','\dot{\theta}_4_{exp]')
hold off


figure

subplot(4,1,1);
plot((0:k-1),u(1,:),'r');
xlabel('k');
ylabel('$ F_1 [N]$','Interpreter','latex');
grid on
hold on

plot([0:size(data.u,1)-1],data.u(:,2),'m');
xticks([0:250:1500])

plot((0:k-1),ones(k)*1.3,'b--');
plot((0:k-1),ones(k)*2.9,'k--');

legend('F_1','F_1_{exp}','sat. inf','sat. sup')
hold off

subplot(4,1,2);
plot((0:k-1),u(2,:),'r');
xlabel('k');
ylabel('$ F_2 [N]$','Interpreter','latex');
grid on
hold on

plot([0:size(data.u,1)-1],data.u(:,2),'m');
xticks([0:250:1500])

plot((0:k-1),ones(k)*1.3,'b--');
plot((0:k-1),ones(k)*2.9,'k--');

legend('F_2','F_2_{exp}','sat. inf','sat. sup')
hold off


subplot(4,1,3);
plot((0:k-1),u(3,:),'r');
xlabel('k');
ylabel('$ alpha_1 [rad]$','Interpreter','latex');
grid on
hold on

plot([0:size(data.u,1)-1],data.u(:,3),'m');
xticks([0:250:1500])

plot((0:k-1),ones(k)*35*deg2rad,'b--');
plot((0:k-1),ones(k)*0*deg2rad,'k--');

legend('alpha_1','alpha_1_{exp}','sat. inf','sat. sup')
hold off


subplot(4,1,4);
plot((0:k-1),u(4,:),'r');
xlabel('k');
ylabel('$ alpha_2 [rad]$','Interpreter','latex');
grid on
hold on

plot([0:size(data.u,1)-1],data.u(:,4),'m');
xticks([0:250:1500])

plot((0:k-1),ones(k)*35*deg2rad,'b--');
plot((0:k-1),ones(k)*0*deg2rad,'k--');

legend('alpha_2','alpha_2_{exp}','sat. inf','sat. sup')
hold off

end

