clear;

global J;
global b;
global c;


t0=0;
tf=1*pi;
t = t0:0.02*pi:tf;

J=0.01;
b=0.0259;
c=0.6376;

y2_init = 0.0; % rad/s
y1_init = 4.0; %[rad]

y0 = [y1_init; y2_init];

[t,y]  = ode45(@theta4_din,[t0 tf],y0);
 

hFig = figure(1);

subplot(2,1,1);

plot(t,y(:,1),'r-');
grid on
ylabel('$\dot{\theta_4} [rad]$','Interpreter','latex','FontSize',15)

subplot(2,1,2);

plot(t,y(:,2),'b-');
grid on
xlabel('Tempo [s]');
ylabel('$\dot{\theta_4} [rad/s]$','Interpreter','latex','FontSize',15)

%hFig = figure(1);

%subplot(1,5,[1 2 3]);

%plot(t,y(:,1),'r-',t,y(:,2),'b-');

% xlabel('time');
% 
% xlim([0 t(end)]);
% 
% ylim([-3 3]);
% 
% xtick = 0:pi:10*pi;
% 
% set(gca,'xtick',xtick);
% 
% set(gca,'xticklabel',{'0','pi','2pi','3pi','4pi','5pi','6pi','7pi','8pi','9pi','10pi'});
% 
% legend('y2(t)','y1(t)');
% 
% grid on;
% 
%  
% 
% subplot(1,5,[4 5]);
% 
% plot(y(:,1),y(:,2),'r-','LineWidth',2);  
% 
% xlabel('y(2)');ylabel('y(1)');
% 
% axis([-3 3 -3 3]);
% 
% pbaspect([1 1 1]);
% 
% tStr = sprintf("y1(0)=%0.2f,y2(0)=%0.2f \nk=%0.2f,m=%0.2f,c=%0.2f",y1_init,y2_init,k,m,c);
% 
% title(tStr);
% 
%  
% 
% set(hFig,'Position',[300 300 780 300]);

