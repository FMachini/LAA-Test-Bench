%Gabriel Renato Oliveira Alves 
%LQRi - Controle de atitude (theta4)

clear;
clc;
% Script para carregar os dados da planta
[A,B,C,D,Ts] = paramplanta();

% Auxiliares
p = size(B,2);
q = size(C,1);
n = size(A,1)+4;

% Condição inicial e referencia
kmax = 1000;
x0 =   [ 0*pi/180,  0*pi/180, 0*pi/180, 0, 0, 0, 0, 0, 0, 0];
xref = [ 10*pi/180;0*pi/180; 0*pi/180];

% Parametros de projeto
pesoINT   = [ 1,  1,   8];
pesoTHETA = [16, 16, 0.5];
pesoTHETAp =[0.5, 4,  16];
Qr = diag([pesoINT,pesoTHETA,pesoTHETAp]); % size(Qr) = q+n
Rr = 1*diag([1, 1, 1, 1]);    % size(Rr) = p
Td = 0.01;          % (s) atraso de transporte;

% Auxiliares dinamica motor/tilt
PF1 = 0.7827/7.571; % PWM f1 para FORCA motor 1 
PF2 = 0.736/6.911;  % PWM f2 para FORCA motor 2
PT1 = 1/7.8413;     % PWM t1 para Tilt1
PT2 = 1/7.5815;     % PWM t2 para Tilt2
convAll = [PF1;PF2;PT1;PT2];

fLim = [10, -10];            % limitantes em PPM
tLim1 = [30*pi/180 * (1/PT1), 0];  % limitantes em PPM
tLim2 = [30*pi/180 * (1/PT2), 0];  % limitantes em PPM

% Script para carregar os dados do controlador
[Ki,K,Nx] = controlador(A,B,C,D,Qr,Rr,Ts);

[Kp,Kx] = computeGain();

% Definindo variaveis de estado e controle
% Estado x
x = zeros(n,kmax+1);
% Entrada
u = zeros(p,kmax);
u2 = zeros(p,kmax);
ctr = zeros(3,kmax);
w = zeros(p,kmax);
% Saida real
y = zeros(q,kmax);
% Saida medida
ym = zeros(q,kmax);

% Condicoes Iniciais
x(:,1) = x0';

xp0 = [zeros(1,n)];
fx0 = [ones(1,n)];
xi = zeros(q,1);
xikM1 = zeros(q,1);
wkm1 = zeros(p,1);

% Ruido posicao angular - Range de +/-0.025 rad
%rng(0,'twister');
a = -0.025;
b = 0.025;
ruidoPa1 = (b-a).*rand(kmax,1) + a;
ruidoPa2 = (b-a).*rand(kmax,1) + a;
ruidoPa3 = (b-a).*rand(kmax,1) + a;
ruidoPa = [ruidoPa1';ruidoPa2';ruidoPa3'];
% Ruido velocidade angular - Range de +/-0.14 rad/s
a = -0.14;
b = 0.14;
ruidoVa1 = (b-a).*rand(kmax,1) + a;
ruidoVa2 = (b-a).*rand(kmax,1) + a;
ruidoVa3 = (b-a).*rand(kmax,1) + a;
ruidoVa = [ruidoVa1';ruidoVa2';ruidoVa3'];


% Configuraçao ode45
options2 = odeset('Reltol',1e-6,'AbsTol',1e-6);
% Evolucao da dinamica da planta
for k = 1:kmax
    xaux = x([1:q,q+5:end],k);
    % Saida real
    y(:,k) = C * xaux;
    % Saida medida
    ym(:,k) = C * xaux + 0*ruidoPa(k);
    
    % Calculando integral do erro xi
    erro = ym(:,k) - xref;
    xikM1 = xi + Ts * erro;
    
    % Calculando forca f
    %u(:,k) = 0*(-[Ki, K] * [xi; (xaux)+[ruidoPa(:,k);ruidoVa(:,k)]] + K*Nx*xref);%
    ctr(:,k) = -Kp*xi - Kx*(xaux+0*[ruidoPa(:,k);ruidoVa(:,k)]);
    u(:,k) = estimateInput([ctr(1,k);ctr(2,k);ctr(3,k)],xaux);
    % Convertendo de u (forca) para w (PPM)
    w(:,k) = (1./convAll) .* u(:,k);
    
    % saturacao da entrada
    w([1,2],k) = min(fLim(1),w([1,2],k)); % Limite superior forca12
    w([1,2],k) = max(fLim(2),w([1,2],k)); % Limite inferior forca12
    
    w(3,k) = min(tLim1(1),w(3,k)); % Limite superior tilt1
    w(3,k) = max(tLim1(2),w(3,k)); % Limite inferior tilt1
    
    w(4,k) = min(tLim2(1),w(4,k)); % Limite superior tilt2
    w(4,k) = max(tLim2(2),w(4,k)); % Limite inferior tilt2
    
    x0 = x(:,k);
    [x0,xp0] = decic(@(t,x,dx) odemaple(t,x,dx,wkm1),0,x0,fx0,xp0,[]);
    [t,dummy] = ode15i(@(t,x,dx) odemaple(t, x, dx, wkm1),[0 Td], x0,xp0,options2); % Atraso de Td segundos
    
    x0 = dummy(end,:)';
    [x0,xp0] = decic(@(t,x,dx) odemaple(t,x,dx,w(:,k)),0,x0,fx0,xp0,[]);
    [t,dummy] = ode15i(@(t,x,dx) odemaple(t, x, dx, w(:,k)),[0 Ts-Td], x0,xp0,options2);
    x(:,k+1) = dummy(end,:)';
    

    xi = xikM1;
    wkm1 = w(:,k);
end
%data = [y',ym',w',u',x(:,1:end-1)'];
%dlmwrite('.txt',data,'delimiter','\t');

y = y*180/pi;%rad -> graus
ym = ym*180/pi;%rad -> graus
plotdata(w,y,ym,u,x,kmax);

