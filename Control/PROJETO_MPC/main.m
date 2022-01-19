clear;
close all;
clc;
% Script para carregar os dados da planta
[A,B,C,D,Ts] = paramplanta();

% Auxiliares
p = size(B,2);
q = size(C,1);
n = size(A,1)+4;

% Parametros de projeto (7,1,7,2)
rho = 7*[1 1];  % peso do controle
mi = 2;         % peso da saida
N = 5;          % horizonte de prediçao 
M = 3;          % horizonte de controle

wLim = [10, -10];           % limitantes em PPM
uMax = 1.319/13.43*[10;10]; % limitantes superiores de forca
uMin = -uMax;               % limitantes inferior de forca
duMax = 2/9*0.8*[5.2; 5.2];     % limitantes superiores da taxa de variacao de forca por periodo
duMin = 2/9*0.8*[-4;-4];        % limitantes inferiores da taxa de variacao de forca por periodo
bqpDU = [repmat( duMax,M,1);
         repmat(-duMin,M,1)];

Td = 0.01;          % (s) atraso de transporte;

% Script para carregar os dados do controlador
[Hqp,Aqp,Gn,phi] = controlador(A,B,C,D,N,M,rho,mi);

% Definindo variaveis de estado e controle
kmax = 400;
% Estado x
x = zeros(n,kmax+1);
% Entrada
u = zeros(p,kmax);
w = zeros(p,kmax);
% Incremento da entrada
du = zeros(p,kmax);
% Saida real
y = zeros(q,kmax);
% Saida medida
ym = zeros(q,kmax);

% Condicoes Iniciais
xini = [25*pi/180, zeros(1,n-1)];   
x(:,1) = xini';
ukm1 = zeros(p,1);
wkm1 = zeros(p,1);

% Referencia
r = 0;
r = repmat(r,N,1);

% Ruido posicao angular - Range de +/-0.025 rad
%rng(0,'twister');
a = -0.025;
b = 0.025;
ruidoPa = (b-a).*rand(kmax,1) + a;
% Ruido velocidade angular - Range de +/-0.14 rad/s
a = -0.14;
b = 0.14;
ruidoVa = (b-a).*rand(kmax,1) + a;

% Configuraçao quadprog
options = optimset('Algorithm','interior-point-convex','Display','final');
% Configuraçao ode45
options2 = odeset('Reltol',1e-6,'AbsTol',1e-6);
% Evolucao da dinamica da planta
for k = 1:kmax
    % Saida real
    y(:,k) = C * x([1,4],k);
    % Saida medida
    ym(:,k) = C * x([1,4],k) + ruidoPa(k);
    
    % Estado artificial
    csi = [(x([1,4],k)+[ruidoPa(k);ruidoVa(k)]);ukm1];

    % Calcular f
    f = phi*csi;
    fqp = 2*Gn'*(f - r);
    % Calcular bqp;
    bqp = [bqpDU;
           repmat((uMax - ukm1),M,1);     %uMAX
           repmat((ukm1 - uMin),M,1)];    %uMIN
    %ubA = repmat((uMax - ukm1),M,1);
    %lbA = repmat((uMin - ukm1),M,1);
    
    % Calcular o incremento no controle
    dutil = quadprog(Hqp,fqp,Aqp,bqp,[],[],[],[],[],options);
    %dutil = qpOASES(Hqp,fqp,Aqp,[],[],lbA,ubA);
    du(:,k) = dutil(1:p);
    % Atualizar o controle aplicado
    u(:,k) = ukm1 + du(:,k);
  
    % convertendo de u (forca) para w (PPM)
    w(:,k) = 13.43/1.319*u(:,k);
    
    % saturacao da entrada
    %w(:,k) = min(wLim(1),w(:,k)); % Limite superior
    %w(:,k) = max(wLim(2),w(:,k)); % Limite inferior
    
    xini = x(:,k);
    [t,dummy] = ode45(@(t,dummy) fnlin(t, dummy, wkm1),[0 Td], xini, options2); % Atraso de Td segundos
    xini = dummy(end,:)';
    [t,dummy] = ode45(@(t,dummy) fnlin(t, dummy, w(:,k)),[0 Ts-Td], xini, options2);
    x(:,k+1) = dummy(end,:)';
    
    ukm1 = u(:,k);
    wkm1 = w(:,k);
end

y = y*180/pi;%rad -> graus
ym = ym*180/pi;%rad -> graus
plotdata(w,y,ym,x,u,du,kmax);

