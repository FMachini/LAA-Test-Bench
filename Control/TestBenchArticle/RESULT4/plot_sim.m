close all;
clear all;
clc;

rad2deg = 180/pi;
deg2rad = pi/180;
tamanhoFonte = 14;
ftsize = 22;
%**************************************************************************
param_sim = load('estados4.txt');
x_sim = param_sim(:,[1:6])*180/pi;
u_sim = param_sim(:,[7:10]);
chi_sim = param_sim(:,[11:19]);
umpc_sim = param_sim(:,[20:23]);
ulqr_sim = param_sim(:,[24:27]);
%**************************************************************************
param_sys = load('./matrizes/parametros_sistema.txt');
Ts = param_sys(1);
k = param_sys(2);
u0 = param_sys([4,5,6,7])';
yref = param_sys([8,9,10]);
%**************************************************************************
A = load('./matrizes/A.txt');
B = load('./matrizes/B.txt');
C = load('./matrizes/C.txt');
K_lqr = load('./matrizes/K_mpc.txt');
LL_ob = load('./matrizes/LL_mpc.txt');
mu = load('./matrizes/mu.txt');
rho = load('./matrizes/rho.txt');
horizontes = load('./matrizes/horizontes.txt');
N = horizontes(1);
M = horizontes(2);
%**************************************************************************
n = size(A,1);
p = size(B,2);
q = size(C,1);
Ip = eye(p);

Ak = A - B*K_lqr;
%Observador
Ab = [Ak, zeros(n,q);zeros(q,n),eye(q)];
Bb = [B;zeros(q,p)];
Cb = [C, eye(q)];
%**************************************************************************
data = load('data.txt');

xAux = data(:,[1 2 3 4 5 6]);
%xAux(:,[1 2 3]) = xAux(:,[1 2 3])*deg2rad;
xAux(:,[4 5 6]) = xAux(:,[4 5 6])*180/pi;

up = data(:,[7 8 9 10]);
up(:,[3 4]) = up(:,[3 4]) * deg2rad;

time = data(:,11);

ref = ones(k,3);
ref(1:500,1) = ref(1:500,1) * 0;
%ref(1:400,2) = ref(1:400,2) * 0;
% ref(501:end,1) = yref(1) * deg2rad;
% ref(:,2) = yref(2) * deg2rad;
% ref(:,3) = yref(3) * deg2rad;

ref(501:end,1) = yref(1);
ref(:,2) = yref(2);
ref(:,3) = yref(3);

u = up - repmat(u0',k,1);

ulqr = -xAux * deg2rad * K_lqr';
umpc = u - ulqr;

chiKK = zeros(n+q,k);
chiKm1Km1 = zeros(n+q,1);
uMpckm1 = zeros(p,1);
ym = xAux(:,[1 2 3])';

%Estimativa do estado chi
for i = 1:k
    chiKKm1 = Ab * chiKm1Km1 + Bb * uMpckm1;
    yKKm1 = Cb * chiKKm1;
    chiKK(:,i) = chiKKm1 + LL_ob * (ym(:,i) - yKKm1);
    chiKm1Km1 = chiKK(:,i);
    uMpckm1 = umpc(i,:)';
end
chiKK = chiKK';
chiKK(:,[4 5]) = chiKK(:,[4 5]);

ukm1 = zeros(k,4);
for i = 2:k
    ukm1(i,:) = u(i-1,:);
end
%
du = u - ukm1;
k = (0:k-1) * 0.02;
%**************************************************************************
figure('units','normalized','outerposition',[0 0 1 1])
%grafico da posicao
subplot(3,2,1);
plot(k,ref(:,1),'m-.','LineWidth',2);
hold on
plot(k,x_sim(:,1),'b-','LineWidth',2);
hold on
plot(k,xAux(:,1),'r--','LineWidth',2);
ylabel('$\theta_1$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
%ylim([-0.05 0.5]);
leg3 = legend('$\theta_{1,ref}$',...
              '$\theta_{1,s}$',...
              '$\theta_1$');
set(leg3,'Interpreter','latex');
leg3.FontSize = tamanhoFonte;
t = title('(a.1)');
t.FontWeight = 'normal';
set(gca,'FontSize',ftsize,'FontName','Times')
ylim([-1 25]);
grid on

subplot(3,2,3);
plot(k,ref(:,2),'m-.','LineWidth',2);
hold on
plot(k,x_sim(:,2),'b-','LineWidth',2);
hold on
plot(k,xAux(:,2),'r--','LineWidth',2);
ylabel('$\theta_2$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
leg3 = legend('$\theta_{2,ref}$',...
              '$\theta_{2,s}$',...
              '$\theta_2$');
set(leg3,'Interpreter','latex');
leg3.FontSize = tamanhoFonte;
t = title('(a.2)');
t.FontWeight = 'normal';
ylim([-5 15]);
set(gca,'FontSize',ftsize,'FontName','Times')
grid on

subplot(3,2,5);
plot(k,ref(:,3),'m-.','LineWidth',2);
hold on
plot(k,x_sim(:,3),'b-','LineWidth',2);
hold on
plot(k,xAux(:,3),'r--','LineWidth',2);
ylabel('$\theta_4$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
ylim([-8 8]);
leg3 = legend('$\theta_{4,ref}$',...
              '$\theta_{4,s}$',...
              '$\theta_4$');
set(leg3,'Interpreter','latex');
leg3.FontSize = tamanhoFonte;
t = title('(a.3)');
t.FontWeight = 'normal';
set(gca,'FontSize',ftsize,'FontName','Times')
grid on

% axes('position',[.17 .17 .12 .12]);
% box on
% kini = 1;
% kend = 200;
% plot(k(kini:kend),chiKK(kini:kend,3)+chiKK(kini:kend,9),'bo-','LineWidth',2);
% hold on
% plot(k(kini:kend),xAux(kini:kend,3),'r--','LineWidth',2);

%grafico da velocidade
subplot(3,2,2);
plot(k,xAux(:,4),'r--','LineWidth',2);
hold on
plot(k,x_sim(:,4),'b-','LineWidth',2);
hold on
ylabel('$\dot{\theta}_1$~(deg/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
leg3 = legend('$\dot{\theta}_1$','$\dot{\theta}_{1,s}$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(b.1)');
t.FontWeight = 'normal';
set(gca,'FontSize',ftsize,'FontName','Times')
ylim([-7 12]);
grid on

subplot(3,2,4);
plot(k,xAux(:,5),'r--','LineWidth',2);
hold on
plot(k,x_sim(:,5),'b-','LineWidth',2);
hold on
ylabel('$\dot{\theta}_2$~(deg/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
leg3 = legend('$\dot{\theta}_2$','$\dot{\theta}_{2,s}$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(b.2)');
t.FontWeight = 'normal';
set(gca,'FontSize',ftsize,'FontName','Times')
ylim([-7 10]);
grid on

subplot(3,2,6);
plot(k,xAux(:,6),'r--','LineWidth',2);
hold on
plot(k,x_sim(:,6),'b-','LineWidth',2);
hold on
ylabel('$\dot{\theta}_4$~(deg/s)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
ylim([-8 8]);
leg3 = legend('$\dot{\theta}_4$','$\dot{\theta}_{4,s}$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(b.3)');
t.FontWeight = 'normal';
set(gca,'FontSize',ftsize,'FontName','Times')
grid on

% axes('position',[.77 .17 .12 .12]);
% box on
% kini = 1;
% kend = 100;
% plot(k(kini:kend),chiKK(kini:kend,6),'bo-','LineWidth',2);
% hold on
% plot(k(kini:kend),xAux(kini:kend,6),'r--','LineWidth',2);

%************************************************************************
figure('units','normalized','outerposition',[0 0 1 1])
%grafico do controle uk - Motor 1
subplot(2,2,1);
plot(k,ulqr(:,1),'b',k,ulqr_sim(:,1),'r','LineWidth',2);
hold on
plot(k,umpc(:,1),'g--',k,umpc_sim(:,1),'m--','LineWidth',2);
ylabel('$f_1$~(N)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
leg3 = legend('$u_{lqr}$','$u_{lqr,s}$',...
              '$u_{mpc}$','$u_{mpc,s}$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(a)');
t.FontWeight = 'normal';
set(gca,'FontSize',ftsize,'FontName','Times')
grid on

%grafico de força total - Motor 1
subplot(2,2,2);
plot(k,up(:,1),'b',k,u_sim(:,1),'r','LineWidth',2);
hold on
%plot(k,u_sim(:,1),'g--',k,u_sim(:,2),'m--','LineWidth',2);
ylabel('Thrust Force~(N)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
leg3 = legend('$u_{1}+\bar{F}_1$','$u_{1,s}+\bar{F}_1$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(b)');
t.FontWeight = 'normal';
set(gca,'FontSize',ftsize,'FontName','Times')
grid on

%grafico do controle uk - Motor 2
subplot(2,2,3);
plot(k,ulqr(:,2),'b',k,ulqr_sim(:,2),'r','LineWidth',2);
hold on
plot(k,umpc(:,2),'g--',k,umpc_sim(:,2),'m--','LineWidth',2);
ylabel('$f_2$~(N)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
leg3 = legend('$u_{lqr}$','$u_{lqr,s}$',...
              '$u_{mpc}$','$u_{mpc,s}$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(c)');
t.FontWeight = 'normal';
set(gca,'FontSize',ftsize,'FontName','Times')
grid on

%grafico força total - Motor 2
subplot(2,2,4);
plot(k,up(:,2),'b',k,u_sim(:,2),'r','LineWidth',2);
hold on
%plot(k,u_sim(:,1),'g--',k,u_sim(:,2),'m--','LineWidth',2);
ylabel('Thrust Force~(N)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
leg3 = legend('$u_{2}+\bar{F}_2$','$u_{2,s}+\bar{F}_2$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(d)');
t.FontWeight = 'normal';
set(gca,'FontSize',ftsize,'FontName','Times')
grid on

%************************************************************************
figure('units','normalized','outerposition',[0 0 1 1])
%grafico do controle uk - Tilt 1
subplot(2,2,1);
plot(k,ulqr(:,3)*rad2deg,'b',k,ulqr_sim(:,3)*rad2deg,'r','LineWidth',2);
hold on
plot(k,umpc(:,3)*rad2deg,'g--',k,umpc_sim(:,3)*rad2deg,'m--','LineWidth',2);
ylabel('$\alpha_1$~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
leg3 = legend('$u_{lqr}$','$u_{lqr,s}$',...
              '$u_{mpc}$','$u_{mpc,s}$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(a)');
t.FontWeight = 'normal';
set(gca,'FontSize',ftsize,'FontName','Times')
grid on

%grafico de tilt total - Tilt 1
subplot(2,2,2);
plot(k,up(:,3)*rad2deg,'b',k,u_sim(:,3)*rad2deg,'r','LineWidth',2);
hold on
%plot(k,u_sim(:,1),'g--',k,u_sim(:,2),'m--','LineWidth',2);
ylabel('Tilt Angle~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
leg3 = legend('$\alpha_1$','$\alpha_{1,s}$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(b)');
t.FontWeight = 'normal';
set(gca,'FontSize',ftsize,'FontName','Times')
grid on

%grafico do controle uk - Tilt 2
subplot(2,2,3);
plot(k,ulqr(:,4)*rad2deg,'b',k,ulqr_sim(:,4)*rad2deg,'r','LineWidth',2);
hold on
plot(k,umpc(:,4)*rad2deg,'g--',k,umpc_sim(:,4)*rad2deg,'m--','LineWidth',2);
ylabel('$\alpha_2$~(N)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
leg3 = legend('$u_{lqr}$','$u_{lqr,s}$',...
              '$u_{mpc}$','$u_{mpc,s}$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(c)');
t.FontWeight = 'normal';
set(gca,'FontSize',ftsize,'FontName','Times')
grid on

%grafico tilt total - Tilt 2
subplot(2,2,4);
plot(k,up(:,4)*rad2deg,'b',k,u_sim(:,4)*rad2deg,'r','LineWidth',2);
hold on
%plot(k,u_sim(:,1),'g--',k,u_sim(:,2),'m--','LineWidth',2);
ylabel('Tilt Angle~(deg)','Interpreter','latex','FontSize',ftsize,'FontName','Times');
xlabel('Time $(s)$','Interpreter','latex','FontSize',ftsize,'FontName','Times');
leg3 = legend('$\alpha_2$','$\alpha_{2,s}$');
leg3.FontSize = tamanhoFonte;
set(leg3,'Interpreter','latex');
t = title('(d)');
t.FontWeight = 'normal';
set(gca,'FontSize',ftsize,'FontName','Times')
grid on

% axes('position',[.72 .12 .08 .08]);
% box on
% kini = 300;
% kend = 600;
% plot(k(kini:kend),up((kini:kend),3),'b',k(kini:kend),up((kini:kend),4),'r','LineWidth',2);
% ylim([-0.001 0.004])

% figure('units','normalized','outerposition',[0 0 1 1])
% %grafico do incremento de controle duk - forca
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

% %grafico do incremento de controle duk - tilt
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

%grafico da estimativa de perturbação
% figure('units','normalized','outerposition',[0 0 1 1])
% plot(k,chiKK(:,7),'b',k,chiKK(:,8),'r',k,chiKK(:,9),'g','LineWidth',2);
% hold on
% plot(k,chi_sim(:,7),'b--',k,chi_sim(:,8),'r--',k,chi_sim(:,9),'g--','LineWidth',2);
% ylabel('Posição Angular (rad)');
% xlabel('Tempo (s)');
% leg3 = legend('$d_1$','$d_2$', '$d_3$','$d_{1,s}$','$d_{2,s}$', '$d_{3,s}$');
% leg3.FontSize = 18;
% set(gca,'FontSize',20)
% set(leg3,'Interpreter','latex');
% hold on
% grid on

%grafico das referencias
% figure('units','normalized','outerposition',[0 0 1 1])
% plot(k,ref(:,1),'b-.',k,ref(:,2),'r-.',k,ref(:,3),'g-.','LineWidth',2);
% ylabel('Posição Angular (rad)');
% xlabel('Tempo (s)');
% ylim([-0.05 0.4]);
% leg3 = legend('$\theta_{1,ref}$','$$\theta_{2,ref}$$', '$\theta_{4,ref}$');
% leg3.FontSize = 18;
% set(gca,'FontSize',20)
% set(leg3,'Interpreter','latex');
% hold on
% grid on

%grafico do tempo de cpu
% figure('units','normalized','outerposition',[0 0 1 1])
% plot(k,time/1000,'b','LineWidth',2);
% ylabel('Tempo computacional (ms)');
% xlabel('Tempo (s)');
% %leg3 = legend('$Tempo$');
% leg3.FontSize = 18;
% set(gca,'FontSize',20)
% set(leg3,'Interpreter','latex');
% hold on
% grid on