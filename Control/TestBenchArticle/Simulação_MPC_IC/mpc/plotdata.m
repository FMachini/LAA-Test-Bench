%% Gabriel Renato Oliveira Alves 
%% MPC_explicit

function [] = plotdata(k,kref,xAux,chiKK,yref,u,up,ulqr,umpc,du,time,x)
rad2deg = 180/pi;
deg2rad = pi/180;

tamanhoFonte = 14;

ref = ones(k,3) .* yref';

ref(1:kref(1),1) = 0;
ref(1:kref(2),2) = 0;
ref(1:kref(3),3) = 0;

xAux = xAux';
up = up';
chiKK = chiKK';
u = u';
ulqr = ulqr';
umpc = umpc';
du = du';
time = time';
k = (0:k-1) * 0.02;

%% grafico da posicao
figure('units','normalized','outerposition',[0 0 1 1])
subplot(3,2,1);
plot(k,ref(:,1),'m-.','LineWidth',2);
hold on
plot(k,chiKK(:,1)+chiKK(:,7),'b-','LineWidth',2);
hold on
plot(k,xAux(:,1),'r--','LineWidth',2);
hold on
plot(k,x(1,1:end-1),'r','LineWidth',2);
ylabel('Posição Angular (rad)');
xlabel('Tempo (s)');
leg3 = legend('$\theta_{1,ref}$',...
              '$\theta_{1,e}$',...
              '$\theta_1$');
set(leg3,'Interpreter','latex');
leg3.FontSize = tamanhoFonte;
t = title('(a.1)');
t.FontWeight = 'normal';
grid on

subplot(3,2,3);
plot(k,ref(:,2),'m-.','LineWidth',2);
hold on
plot(k,chiKK(:,2)+chiKK(:,8),'b-','LineWidth',2);
hold on
plot(k,xAux(:,2),'r--','LineWidth',2);
ylabel('Posição Angular (rad)');
xlabel('Tempo (s)');
leg3 = legend('$\theta_{2,ref}$',...
              '$\theta_{2,e}$',...
              '$\theta_2$');
set(leg3,'Interpreter','latex');
leg3.FontSize = tamanhoFonte;
t = title('(a.2)');
t.FontWeight = 'normal';
grid on

subplot(3,2,5);
plot(k,ref(:,3),'m-.','LineWidth',2);
hold on
plot(k,chiKK(:,3)+chiKK(:,9),'b-','LineWidth',2);
hold on
plot(k,xAux(:,3),'r--','LineWidth',2);
ylabel('Posição Angular (rad)');
xlabel('Tempo (s)');
%ylim([-0.0325 0.0025]);
leg3 = legend('$\theta_{4,ref}$',...
              '$\theta_{4,e}$',...
              '$\theta_4$');
set(leg3,'Interpreter','latex');
leg3.FontSize = tamanhoFonte;
t = title('(a.3)');
t.FontWeight = 'normal';
grid on

% axes('position',[.17 .17 .12 .12]);
% box on
% kini = 1;
% kend = 200;
% plot(k(kini:kend),chiKK(kini:kend,3)+chiKK(kini:kend,9),'bo-','LineWidth',2);
% hold on
% plot(k(kini:kend),xAux(kini:kend,3),'r--','LineWidth',2);

%% grafico da velocidade
subplot(3,2,2);
plot(k,xAux(:,4),'r--','LineWidth',2);
hold on
plot(k,chiKK(:,4),'b-','LineWidth',2);
hold on
ylabel('Velocidade Angular(rad/s)');
xlabel('Tempo (s)');
leg3 = legend('$\dot{\theta}_1$','$\dot{\theta}_{1,e}$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(b.1)');
t.FontWeight = 'normal';
grid on

subplot(3,2,4);
plot(k,xAux(:,5),'r--','LineWidth',2);
hold on
plot(k,chiKK(:,5),'b-','LineWidth',2);
hold on
ylabel('Velocidade Angular(rad/s)');
xlabel('Tempo (s)');
%ylim([-0.0475 0.02]);
leg3 = legend('$\dot{\theta}_2$','$\dot{\theta}_{2,e}$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(b.2)');
t.FontWeight = 'normal';
grid on

subplot(3,2,6);
plot(k,xAux(:,6),'r--','LineWidth',2);
hold on
plot(k,chiKK(:,6),'b-','LineWidth',2);
hold on
ylabel('Velocidade Angular(rad/s)');
xlabel('Tempo (s)');
%ylim([-0.1 0.02]);
leg3 = legend('$\dot{\theta}_4$','$\dot{\theta}_{4,e}$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(b.3)');
t.FontWeight = 'normal';
grid on

% axes('position',[.77 .17 .12 .12]);
% box on
% kini = 1;
% kend = 100;
% plot(k(kini:kend),chiKK(kini:kend,6),'bo-','LineWidth',2);
% hold on
% plot(k(kini:kend),xAux(kini:kend,6),'r--','LineWidth',2);

%% grafico do controle uk - forca
figure('units','normalized','outerposition',[0 0 1 1])
subplot(2,2,1);
plot(k,ulqr(:,1),'b',k,ulqr(:,2),'r','LineWidth',2);
hold on
plot(k,umpc(:,1),'g--',k,umpc(:,2),'m--','LineWidth',2);
ylabel('Força (N)');
xlabel('Tempo (s)');
leg3 = legend('$u_{lqr,1}$','$u_{lqr,2}$',...
              '$u_{mpc,1}$','$u_{mpc,2}$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(a)');
t.FontWeight = 'normal';
grid on

% axes('position',[.12 .77 .12 .12]);
% box on
% kini = 1;
% kend = 25;
% plot(k(kini:kend),ulqr(kini:kend,1),'b',k(kini:kend),ulqr(kini:kend,2),'r','LineWidth',2);
% hold on
% plot(k(kini:kend),umpc(kini:kend,1),'g--',k(kini:kend),umpc(kini:kend,2),'m--','LineWidth',2);

%% grafico do controle uk - forca
subplot(2,2,2);
plot(k,up(:,1),'b',k,up(:,2),'r','LineWidth',2);
ylabel('Força (N)');
xlabel('Tempo (s)');
leg3 = legend('$u_{1}+\bar{F}_1$','$u_{2}+\bar{F}_2$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(b)');
t.FontWeight = 'normal';
grid on

%% grafico do controle uk - tilt
subplot(2,2,3);
plot(k,ulqr(:,3),'b',k,ulqr(:,4),'r','LineWidth',2);
hold on
plot(k,umpc(:,3),'g--',k,umpc(:,4),'m--','LineWidth',2);
ylabel('Posição Angular (rad)');
xlabel('Tempo (s)');
leg3 = legend('$u_{lqr,3}$','$u_{lqr,4}$',...
              '$u_{mpc,3}$','$u_{mpc,4}$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(c)');
t.FontWeight = 'normal';
grid on

%% grafico do controle uk - tilt
subplot(2,2,4);
plot(k,up(:,3),'b',k,up(:,4),'r','LineWidth',2);
yleg = ylabel('Posição Angular (rad)');
xleg = xlabel('Tempo (s)');
leg3 = legend('$u_3$','$u_4$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(d)');
t.FontWeight = 'normal';
grid on
% axes('position',[.72 .12 .08 .08]);
% box on
% kini = 300;
% kend = 600;
% plot(k(kini:kend),up((kini:kend),3),'b',k(kini:kend),up((kini:kend),4),'r','LineWidth',2);
% ylim([-0.001 0.004])

%% grafico do incremento de controle duk - forca
% figure('units','normalized','outerposition',[0 0 1 1])
% subplot(2,1,1);
% plot(k,du(:,1),'b',k,du(:,2),'r','LineWidth',2);
% ylabel('Força (N)');
% xlabel('Tempo (s)');
% leg3 = legend('$\Delta u_{1}$','$\Delta u_{2}$');
% leg3.FontSize = tamanhoFonte;
% set(leg3,'Interpreter','latex');
% set(gca,'FontSize',14)
% t = title('(a)');
% t.FontWeight = 'normal';
% grid on

% axes('position',[.12 .72 .08 .08]);
% box on
% kini = 1;
% kend = 20;
% plot(k(kini:kend),du((kini:kend),1),'b',k(kini:kend),du((kini:kend),2),'r','LineWidth',2);
%ylim([-0.001 0.004])

%% grafico do incremento de controle duk - tilt
% subplot(2,1,2);
% plot(k,du(:,3),'b',k,du(:,4),'r','LineWidth',2);
% ylabel('Posição Angular (rad)');
% xlabel('Tempo (s)');
% leg3 = legend('$\Delta u_{3}$','$\Delta u_{4}$');
% leg3.FontSize = tamanhoFonte;
% set(leg3,'Interpreter','latex');
% set(gca,'FontSize',14)
% t = title('(b)');
% t.FontWeight = 'normal';
% grid on

%% grafico da estimativa de perturbação
figure('units','normalized','outerposition',[0 0 1 1])
plot(k,chiKK(:,7),'b',k,chiKK(:,8),'r',k,chiKK(:,9),'g','LineWidth',2);
ylabel('Posição Angular (rad)');
xlabel('Tempo (s)');
leg3 = legend('$d_1$','$d_2$', '$d_3$');
leg3.FontSize = 18;
set(gca,'FontSize',20)
set(leg3,'Interpreter','latex');
hold on
grid on

%% grafico das referencias
% figure('units','normalized','outerposition',[0 0 1 1])
% plot(k,ref(:,1),'b-.',k,ref(:,2),'r-.',k,ref(:,3),'g-.','LineWidth',2);
% ylabel('Posição Angular (rad)');
% xlabel('Tempo (s)');
% leg3 = legend('$\theta_{1,ref}$','$$\theta_{2,ref}$$', '$\theta_{4,ref}$');
% leg3.FontSize = tamanhoFonte;
% set(leg3,'Interpreter','latex');
% hold on
% grid on
end

