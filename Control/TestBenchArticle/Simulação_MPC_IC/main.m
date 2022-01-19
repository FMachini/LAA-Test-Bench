%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Title: 3 DOF Test Bench Model Using Robotic Notation 
%%% Author: Felipe Machini M. Marques and Gabriel Renato Oliveira Alves
%%% Date: 21/06/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all;clc

%% Creating test bench robotic model

BenchModel; %create 3dof bench model
actuatorDynamics; %Create actators transfer functions

%% Robot Simulation
q = homeConfiguration(robot); 
ndof = size(q,2);

% Initializing variables
kend = 3000;
Ts = 0.02;

x = zeros(2*ndof,kend);
x(:,1) = [q zeros(size(q))];
u = zeros(4,kend);
uctr = zeros(4,kend);
utrim = [1.75 1.75 0 0]'; %equilibrium condition
xm1=0;xm2=0;xa1=[0,0];xa2=[0,0];
int_e = 0;
Cc = [1 0 0 0 0 0 0 0;
      0 -1 0 0 0 0 0 0;
      0 0 0 1 0 0 0 0];%get theta1 theta2 theta4 positions
  
Cc2 = [1  0 0 0 0  0 0 0;
       0 -1 0 0 0  0 0 0;
       0  0 0 1 0  0 0 0;
       0  0 0 0 1  0 0 0;
       0  0 0 0 0 -1 0 0;
       0  0 0 0 0  0 0 1]; %get theta1 theta2 theta4 positions and velocities
 xref = [20;10;0]*pi/180;
 
 % Computing Control Gain Matrices
 [Kp,Kx] = computeGain(Ts); %Compute LQR controller gains
 
%% MPC
addpath('./mpc')
%% Parametros planta
[A, B, C, D, Ts, u0, Acon, Bcon, Ccon, Dcon] = paramplanta_gab();
%% Auxiliares
p = size(B, 2);
q_size = size(C, 1);
n = size(A, 1);
rad2deg = 180/pi;
deg2rad = pi/180;
%% Duração simulação
kmax = kend;
%% Estado inicial
up0 = utrim;%[u0; 0; 0];
theta0 = [0, 0, 0]*deg2rad;
thetap0 = [0, 0, 0];
xini = [theta0, thetap0];
%% Referencia de saida
yref = [20; 10; 0]*deg2rad;
yref_aux = [0; 0; 0];
kref = [500 + 1; 0 + 1; 0 + 1];
%% -----------------------------------------------------------------------
%% PROJETO DO MPC
%% -----------------------------------------------------------------------
%% Parametros de projeto do Observador
pesoESTADOS = [1, 1, 1, 1, 1, 1];
pesoPERT = [1, 1, 1] * 16;
pesoSAIDA = [1, 1, 1];
Qo = diag([pesoESTADOS, pesoPERT]); % size = q+n
Ro = diag([pesoSAIDA]);

%% Parametros de projeto do LQR
pesoTHETA = [0.2, 42, 6.5];
pesoTHETAp =[0.1, 128, 1.8];
pesoForca = [5, 5];
pesoAlfa =  [1, 1];
Qr = diag([pesoTHETA, pesoTHETAp]);   % peso dos estados
Rr = diag([pesoForca, pesoAlfa]);       % peso do controle lqr

%% Parametros de projeto do MPC
N = 10;             % horizonte de prediçao
M = 8;              % horizonte de controle
pesoDForca = [15, 15];
pesoDAlfa =  [20, 20];
pesoTHETA = [0.8, 10, 0.1];
R = diag([pesoDForca, pesoDAlfa]);    % peso do incremento de controle
Q = diag([pesoTHETA]);               % peso da saida
r = repmat(yref_aux, N, 1);

%% Restrições
umax = [2.9-up0(1); 2.9-up0(2); 30*deg2rad; 30*deg2rad];   % limitantes superiores de forca/tilt
umin = [1.3-up0(1); 1.3-up0(2); 0; 0];     % limitantes inferior de forca/tilt
dumax = [0.8; 0.8; 0.13; 0.13];            % limitantes superiores da taxa de variacao de forca/tilt por periodo
dumin =-[0.8; 0.8; 0.13; 0.13];            % limitantes inferiores da taxa de variacao de forca/tilt por periodo
%% Atraso de transporte;
Td = 0.015;  % (s)
%% Matrizes dos modelos chi e csi
[LL, Ab, Bb, Cb, K, Atil, Btil, Ctil, Ak] = matrizes_modelos(A, B, C, Qr, Rr, Qo, Ro);

%% MPC classico
[Pdu,Pu,PI,Hdu,Hu,Hqp,Aqp,Gn,phi] = mpc_classico(Ak,B,C,Atil,Btil,Ctil,N,M,Q,R,K);
uMaxN = repmat(umax, N, 1);
uMinN = repmat(umin, N, 1);
duMaxN = repmat(dumax, N, 1);
duMinN = repmat(dumin, N, 1);
%% Definindo variaveis de estado e controle
% Estado x
x_mpc = zeros(n,kmax+1);
x_ruido = zeros(n,kmax+1);
chi = zeros(n+q_size,kmax);
% Entrada
umpclqr = zeros(p,kmax);
ulqr = zeros(p,kmax);
umpc = zeros(p,kmax);
up = zeros(p,kmax);
% Incremento da entrada
dumpc = zeros(p,kmax);
du = zeros(p,kmax);
% Saida real
y = zeros(q_size,kmax);
% Saida medida
ym = zeros(q_size,kmax);
% Timer
timer = zeros(1,kmax);
%% Condicoes Iniciais
x_mpc(:,1) = xini';
ukm1 = zeros(p,1);
umpckm1 = zeros(p,1);
chiKm1Km1 = zeros(n+q_size,1);
%% Ruido posicao angular - Range de +/-0.025 rad
%rng(0,'twister');
a = -0.025;
b = 0.025;
ruidoPa1 = (b-a).*rand(kmax,1) + a;
ruidoPa2 = (b-a).*rand(kmax,1) + a;
ruidoPa3 = (b-a).*rand(kmax,1) + a;
ruidoPa = [ruidoPa1';ruidoPa2';ruidoPa3'];
%% Ruido velocidade angular - Range de +/-0.14 rad/s
a = -0.14;
b = 0.14;
ruidoVa1 = (b-a).*rand(kmax,1) + a;
ruidoVa2 = (b-a).*rand(kmax,1) + a;
ruidoVa3 = (b-a).*rand(kmax,1) + a;
ruidoVa = [ruidoVa1';ruidoVa2';ruidoVa3'];

%% Configuraçao quadprog
options = optimset('Algorithm','interior-point-convex','Display','final');
TotalTime = 0;
tic
for  i = 1:kend
    k = i;
    % Calculating control action
%     e = xref - Cc*x(:,i);
%     int_e = int_e + e*Ts;
%     uctr(:,i+1) =  Kp*int_e - Kx*Cc2*x(:,i);
%     uctr([3,4],i+1) = max(uctr([3,4],i+1),umin([3,4]));
%     uctr([3,4],i+1) = min(uctr([3,4],i+1),umax([3,4]));
    %% Calculo da ação de controle do MPC
    % Configurando referência
    if (k == kref(1))
        yref_aux(1) = yref(1);
        r = repmat(yref_aux, N, 1);
    end
    if (k == kref(2))
        yref_aux(2) = yref(2);
        r = repmat(yref_aux, N, 1);
    end
    if (k == kref(3))
        yref_aux(3) = yref(3);
        r = repmat(yref_aux, N, 1);
    end
    % Estados com ruido
    x_ruido(:,k) = Cc2 * x(:,k); %+ [ruidoPa(:,k);ruidoVa(:,k)];
    % Saida medida
    ym(:,k) = x_ruido([1:q_size],k);
    % Estimativa do estado chi
    chiKKm1 = Ab * chiKm1Km1 + Bb * umpckm1;
    yKKm1 = Cb * chiKKm1;
    chi(:,k) = chiKKm1 + LL * (ym(:,k) - yKKm1);
    chiKm1Km1 = chi(:,k);
    % Estado artificial
    csi = [chi(:,k);umpckm1];
    % Calcular f
    f = phi * csi;
    fqp = 2 * Gn' * (f - r);
    fu = Pu *  x_ruido(:,k); %chi([1:n],k);
    fdu = Pdu *  x_ruido(:,k); %chi([1:n],k);
    fI = PI * ukm1;
    uMpckm1N = repmat(umpckm1,N,1);
    % Calcular bqp;
    bqpDU = [duMaxN - fdu + fI - Hdu*uMpckm1N;
             fdu - fI + Hdu*uMpckm1N - duMinN];
    bqpU = [uMaxN-Hu*uMpckm1N-fu;
            fu+Hu*uMpckm1N-uMinN];
    bqp = [bqpDU; bqpU];
    
    % Calcular o incremento no controle
    [dutil,fval,exitFlag] = quadprog(Hqp,fqp,Aqp,bqp,[],[],[],[],[],options);
    dumpc(:,k) = dutil(1:p);
    
    % Controle
    umpc(:,k) = umpckm1 + dumpc(:,k);
    ulqr(:,k) = -K *  x_ruido(:,k); %chi([1:n],k);
    umpclqr(:,k) = ulqr(:,k) + umpc(:,k);
    up(:,k) = umpclqr(:,k) + up0;
    
    timer(1,k) = toc;
    TotalTime = TotalTime + timer(1,k);
    tic;
    fprintf('Interação: %d\nPeriod Time: %0.2f seconds\nTotalTime: %0.2f seconds\n',k,timer(1,k),TotalTime);
    
    %x_mpc(:,k+1) = A * x_mpc(:,k) + B * umpclqr(:,k);
    
    du(:,k) = umpclqr(:,k) - ukm1; 
    umpckm1 = umpc(:,k);
    ukm1 = umpclqr(:,k);
    
    uctr(:,i+1) = umpclqr(:,k);
%% Actuator dynamics
     [f1,~,xm1] = lsim(motor1SYS,uctr(1,i+1)*ones(5,1),0.02*(0:0.25:1),xm1(end),'zoh');
     [f2,~,xm2] = lsim(motor2SYS,uctr(2,i+1)*ones(5,1),0.02*(0:0.25:1),xm2(end),'zoh');
     [alpha1,~,xa1] = lsim(alpha1SYS,uctr(3,i+1)*ones(5,1)*180/pi,0.02*(0:0.25:1),xa1(end,:),'zoh');
     [alpha2,~,xa2] = lsim(alpha2SYS,uctr(4,i+1)*ones(5,1)*180/pi,0.02*(0:0.25:1),xa2(end,:),'zoh');
     
    u(:,i+1) = [f1(end);f2(end);alpha1(end)*pi/180;alpha2(end)*pi/180]+ utrim;
    %u(:,i+1) = uctr(:,i+1);
    %utotal = uctr(:,i+1) ;
    % Estimating Bicopter forces and moments wrt body coordinate frame
    [fBCS] = Bicopter(u(:,i+1));
    % Projecting joint forces on Inertial Coordinate frame
    fext = externalForce(robot,'endeffector',fBCS,x(1:ndof,i)');
    %calculating friction torques at each joint
    tauFric = -diag([1.013 0.338 0.4 0.6925/0.0187])*x(ndof+1:end,i);
    % Calculating joint accelerations
    qddot = forwardDynamics(robot,x(1:ndof,i)',x(ndof+1:end,i)',tauFric',fext);
    qdot = x(ndof+1:end,i) + qddot'*Ts; 
    q = x(1:ndof,i) + qdot*Ts;
    x(:,i+1) = [q;qdot];
end


%% Plotting Data
x_ruido = x_ruido(:,[1:kmax]);
dlmwrite('estados4.txt',[x_ruido',up',umpc',ulqr'],'delimiter','\t');
plotdata(kmax,kref,x_ruido,chi,yref,umpclqr,up,ulqr,umpc,du,timer,x);
%plotData;

%% Simulation

% figure('rend','painters','pos',[10 10 1000 800])
% for i = 1:kend
%     show(robot,x(1:ndof,i+1)','PreservePlot',false);
%     axis([-1,1,-1,1,-0.5,1])
%     pause(0.02)    
% end


