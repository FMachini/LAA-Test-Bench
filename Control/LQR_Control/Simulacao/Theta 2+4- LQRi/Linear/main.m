%LQRi - Controle de atitude e altura(theta24)

clear;
close all;
clc;

rad2deg=180/pi;
deg2rad=pi/180;

% Script para carregar os dados da planta
[A,B,C,D,Ts] = paramplanta();

% Auxiliares
p = size(B,2);
q = size(C,1);
n = size(A,1);

% Condicao inicial e referencia
kmax = 1500;
forcasiniciais=[1.7 1.7];
thetainitial    =[-18.7 -5]*deg2rad;
theta_dotinitial=[0 0]*deg2rad;

xini = [thetainitial theta_dotinitial];

xref0    = [thetainitial(1) 0]; % já esta em rad regular theta4  
xref100  = deg2rad*[0 0];       % regular theta2
xref500  = deg2rad*[10 0];      % ref theta2
xref1000 = deg2rad*[0 0];       % regular theta2

up0 = forcasiniciais';

% Parametros de projeto
PesoIntegral=[0.2 0.2];    %errotheta2 errotheta4
PesoTheta   =[8 4];        %theta2 theta4
PesoThetadot=[15 0.001];   %theta2_dot theta4_dot

Qr = diag([PesoIntegral PesoTheta PesoThetadot]);   % size(Qr) = q+n
Rr = 1*diag([1, 1]);                                % size(Rr) = p
Td = 0.01;                                          % (s) atraso de transporte;


% Script para carregar os dados do controlador
[Ki,K,Nx] = controlador(A,B,C,D,Qr,Rr,Ts);


% Definindo variaveis de estado e controle
% Estado x
x = zeros(n,kmax+1);
% Entrada
u = zeros(p,kmax);
w = zeros(p,kmax);
% Saida real
y = zeros(q,kmax);
% Saida medida
ym = zeros(q,kmax);

% Condicoes Iniciais
x(:,1) = xini';

xp0 = [zeros(1,n)];
fx0 = [ones(1,n)];
xi = zeros(q,1);
wkm1 = zeros(p,1);
xreferencias=zeros( size(thetainitial,2),kmax);

% Evolucao da dinamica da planta
for k = 1:kmax
    
    xreferencias(:,k)=xref0';
    
    if(k>=1000)
        xreferencias(:,k)=xref1000';
    elseif(k>=500)
        xreferencias(:,k)=xref500';
    elseif(k>=100)
        xreferencias(:,k)=xref100';
    end
    
    xaux = x(:,k);

    y(:,k) = C * xaux;

    % Calculando integral do erro xi
    erro = y(:,k) - xreferencias(:,k);
    xi = xi + Ts * erro;
    
    % Calculando forca f
    u(:,k) = -[Ki, K] * [xi; xaux] + K*Nx*xreferencias(:,k);
    
    
    u(:,k)=max(-0.4,u(:,k));
    u(:,k)=min(1.2,u(:,k));
    
    up = u(:,k) + up0;
     
    x0 = x(1:4,k);
    
    x(:,k+1)=A*x0+B*u(:,k);

end

y = y*180/pi;%rad -> graus
ym = ym*180/pi;%rad -> graus
u = u + up0;
plotdata(y,u,x,xreferencias*rad2deg,kmax);

