%LQRi - Controle de atitude e altura(theta24)

clear;
close all;
clc;

global Ts ;
Ts=0.02;

deg2rad = pi/180;
rad2deg = 180/pi;

% Script para carregar os dados da planta
[A,B,C,D,Ts] = paramplanta();

% Auxiliares
p = size(B,2);
q = size(C,1);
n = size(A,1);

% Condicao inicial e referencia
kmax = 1500;

forcasiniciais=[1.7 1.7];
angulosiniciais=[0 0];

thetainitial    =[0 0 -3]*deg2rad;
theta_dotinitial=[0 0 0]*deg2rad;

xini = [thetainitial theta_dotinitial];

xref0    = deg2rad*[0 18.7 0]; % já esta em rad regule theta4
xref100  = deg2rad*[0 18.7 0];       % regule theta2                                                              
xref500  = deg2rad*[-10 18.7 0];       % ref theta1
xref1000 = deg2rad*[-10 18.7 0];       % voltar para origem

up0 = [forcasiniciais';angulosiniciais'];

PesoInt=[0.5 0.5 0.5];             %err_theta1     err_theta2     err_theta4
PesoTheta=[0.1 1 0.05];            %theta1         theta2         theta4
PesoTheta_dot=[1.001 0.5 1];   %theta1_dot     theta2_dot     theta4_dot

% Parametros de projeto
Qr = diag([PesoInt, PesoTheta, PesoTheta_dot]); % size(Qr) = q+n
Rr = 1.0*diag([1, 1, 1, 1]);    % size(Rr) = p
Td = 0.15;          % (s) atraso de transporte;

% PesoInt=[0.5 0.5 0.5];             %err_theta1     err_theta2     err_theta4
% PesoTheta=[0.1 1 0.05];            %theta1         theta2         theta4
% PesoTheta_dot=[1.001 0.5 0.4901];   %theta1_dot     theta2_dot     theta4_dot

% Parametros de projeto
% PesoInt=[0.25 0.2 0.5];             %err_theta1     err_theta2     err_theta4
% PesoTheta=[0.01 1 0.05];            %theta1         theta2         theta4
% PesoTheta_dot=[1.001 0.5 0.4901];   %theta1_dot     theta2_dot     theta4_dot

% Qr = diag([PesoInt, PesoTheta, PesoTheta_dot]); % size(Qr) = q+n
% Rr = 1.0*diag([1, 1, 1, 1]);    % size(Rr) = p
% Td = 0.15;          % (s) atraso de transporte;

wLim = [60, 40];   % limitantes em PPM

% Auxiliares dinamica motor
Fcoef1 = [0.000458547428571   0.036316433142860  -0.944062080000104];
Fcoef2 = [0.000796758857143   0.009037906285718  -0.285379440000110];

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

% Configuracao ode45
options2 = odeset('Reltol',1e-6,'AbsTol',1e-6);
% Evolucao da dinamica da planta
for k = 1:kmax
    
    xreferencias(:,k)=xref0';
    
    if(k>=1000)
        xreferencias(:,k)=xref1000';
    elseif(k>=700)
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
    
    u(1:2,k)=max(-0.4,u(1:2,k));
    u(1:2,k)=min(1.2,u(1:2,k));
    
    u(3:4,k)=min(35*deg2rad,u(3:4,k));
    u(3:4,k)=max(0,u(3:4,k));
    
    up = u(:,k) + up0;
    
    x0 = x(1:n,k);
    
    x(:,k+1)=A*x0+B*u(:,k);

end

y = y*180/pi;   %rad -> graus
ym = ym*180/pi; %rad -> graus
u = u + up0;
plotdata(y,u,x,xreferencias*rad2deg,kmax);

% Escrevendo txt

K

fileID = fopen('Ganho_Estados.txt','w');
fprintf(fileID,'%5f %5f %5f %5f %5f %5f\n',K');
fclose(fileID);

Ki

fileID = fopen('Ganho_Integral.txt','w');
fprintf(fileID,'%5f %5f %5f\n',Ki');
fclose(fileID);