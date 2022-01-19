clear;
%close all;
clc;
%Script para carregar os dados da planta
[A,B,C,D,Ts] = paramplanta();

%Auxiliares
p = size(B,2);
q = size(C,1);
n = size(A,1)+4;

%Parametros de projeto
Qr = diag([0.1, 5, 1]);
Rr = diag([0.5 0.5]) ;

% limitantes em PPM
wLim = [10, -10];

%(s)atraso de transporte;
Td = 0.01; 

%Script para carregar os dados do controlador
Kc = controlador(A,B,C,D,Qr,Rr);
%Kc = acker(A, B, [0.95 0.96])
Kc=place(A,B,[0.90 0.93]);
%Definindo variaveis de estado e controle
kmax = 1000;
%Estado x
x = zeros(n,kmax+1);
%Entrada
u = zeros(p,kmax);            
w = zeros(p,kmax);  
%Saida real
y = zeros(q,kmax);
%Saida medida
ym = zeros(q,kmax);

%Condicoes Iniciais
xini = [9*pi/180 zeros(1,n-1)];   
x(:,1) = xini';
wkm1 = zeros(p,1);
% ukm2 = zeros(p,1);

%referencia
%Matriz M nao e quadrada portanto nao e inversivel
%Trataremos inicialmente com um regulardor

%ruido posicao angular - Range de +/-0.025 rad
% rng(0,'twister');
a = -0.025;
b = 0.025;
ruidoPa = (b-a).*rand(kmax,1) + a;
%ruido velocidade angular - Range de +/-0.14 rad/s
a = -0.14;
b = 0.14;
ruidoVa = (b-a).*rand(kmax,1) + a;

%Configura�ao ode45
options2 = odeset('Reltol',1e-6,'AbsTol',1e-6);
%Evolucao da dinamica da planta
for k = 1:kmax
    % calculando forca f
    u(:,k) = -Kc * (x([1,4],k) + [ruidoPa(k); ruidoVa(k)*1]); % o ruido faz efeito aqui
    
    % convertendo de u (forca) para w (PPM)
    w(:,k) = 13.43/1.319.*u(:,k);
    
    % saturacao da entrada
    w(:,k) = min(wLim(1), w(:,k)); % lim. superior
    w(:,k) = max(wLim(2), w(:,k)); % lim inferior
    
    % simulando
     xini = x(:,k);
%     [t,dummy] = ode45(@(t,dummy) fnlin(t, dummy, [wkm1; -wkm1]*1),[0 Td], xini, options2);
%     xini = dummy(end,:)';
%     [t,dummy] = ode45(@(t,dummy) fnlin(t, dummy, [w(:,k); -w(:,k)]*1),[0 Ts-Td], xini, options2);
%     x(:,k+1) = dummy(end,:)';
    [t,dummy] = ode45(@(t,dummy) fnlin(t, dummy, wkm1),[0 Td], xini, options2);
    xini = dummy(end,:)';
    [t,dummy] = ode45(@(t,dummy) fnlin(t, dummy, w(:,k)),[0 Ts-Td], xini, options2);
    x(:,k+1) = dummy(end,:)';
    
    %saida real
    y(:,k) = C * x([1,4],k);
    %saida medida
    ym(:,k) = C * x([1,4],k) + ruidoPa(k);
    
    wkm1 = w(:,k);
end

y = y.*180/pi;%rad -> graus
ym = ym.*180/pi;%rad -> graus
plotdata(w,y, ym, x,kmax);



