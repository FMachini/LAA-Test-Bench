%LQRi - Controle de atitude e altura(theta24)

function [] = plotdata(y,u,x,xref,k)
figure

subplot(2,2,1);
plot((0:k-1),y(1,:)','b');
xlabel('k');
ylabel('$\theta_2 [graus]$ ','Interpreter','latex');
grid on
hold on
plot((0:k-1),xref(1,:)','k--');
legend('\theta_2','referência');
hold off


subplot(2,2,2);
plot((0:k-1),y(2,:)','k');
xlabel('k');
ylabel('$\theta_4 [graus]$','Interpreter','latex');
grid on
hold on
plot((0:k-1),xref(2,:)','k--');
legend('\theta_4','referência')
hold off


subplot(2,2,3);
plot((0:k),x(3,:),'b');
xlabel('k');
ylabel('$\dot{\theta}_2[graus/s]$','Interpreter','latex');
grid on



subplot(2,2,4);
plot((0:k),x(4,:),'k');
xlabel('k');
ylabel('$\dot{\theta}_4[graus/s]$','Interpreter','latex');
grid on



figure

subplot(2,1,1);
plot((0:k-1),u(1,:),'r');
xlabel('k');
ylabel('$ F_1 [N]$','Interpreter','latex');
grid on
hold on
plot((0:k-1),ones(k)*1.3,'b--');
plot((0:k-1),ones(k)*2.9,'k--');
legend('F_1','sat. inf','sat. sup')
hold off

subplot(2,1,2);
plot((0:k-1),u(2,:),'r');
xlabel('k');
ylabel('$ F_2 [N]$','Interpreter','latex');
grid on
hold on
plot((0:k-1),ones(k)*1.3,'b--');
plot((0:k-1),ones(k)*2.9,'k--');
legend('F_2','sat. inf','sat. sup')
hold off

end

