clear;clc
close all;
global L
%Script para carregar os dados da planta
[A,B,C,D,Ts] = paramplanta();
L = dlqr(A,C',0.1*eye(6),5*eye(2));
L = L';
%Auxiliares
p = size(B,2);
q = size(C,1);
n = size(A,1);

%Parametros de projeto
Qr = diag([1, 1, 1, 1, 1, 1]);
Rr = diag([1, 1]);
uLim = [10, -10];
Td = 2*Ts; %(s)atraso de transporte;

%Script para carregar os dados do controlador
Kc = controlador(A,B,C,D,Qr,Rr);
%Definindo variaveis de estado e controle
kmax = 1000;
%Estado x
x = zeros(n,kmax+1);
%Entrada
u = zeros(p,kmax);            
%Saida
y = zeros(q,kmax);

%Condicoes Iniciais
xini = [-pi/6 zeros(1,n-1)];   
x(:,1) = xini';
ukm1 = zeros(p,1);
ukm2 = zeros(p,1);

%referencia
%Matriz M nao e quadrada portanto nao e inversivel
%Trataremos inicialmente com um regulardor

%ruido posicao angular - Range de 0.05 rad
rng(0,'twister');
a = -0.025;
b = 0.025;
ruidoPa = (b-a).*rand(kmax,1) + a;
%ruido velocidade angular - Range de 0.4 rad/s
a = -0.14;
b = 0.14;
ruidoVa = (b-a).*rand(kmax,1) + a;

%filtro Butterworth
fs = 25;      %Sample frequency(Hz)
fn = fs/2;      %Nyquist frequency (midpoint of frequency range)
fc = 7;         %Cutoff frequency
[b,a] = butter(2, fc/fn);
x_fil = x;
x_fil4_km1 = 0;
x4km1 = 0;
x_hat = x;
y_hat = zeros(2,kmax+1);
y = y_hat;
%Configuraçao ode45
options2 = odeset('Reltol',1e-6,'AbsTol',1e-6);
%Evolucao da dinamica da planta
for k = 1:kmax
    u(:,k) = -Kc * x_hat(:,k);
    %y_hat(:,k) = C*x_fil(:,k);
    
    %saturacao da entrada
    u(:,k) = min(uLim(1),u(:,k));
    u(:,k) = max(uLim(2),u(:,k));
    
    xini = x_hat(:,k);
    [t,dummy] = ode45(@(t,dummy) fnlin(t, dummy, u(:,k)),[0 Ts], xini, options2);
    x(:,k+1) = dummy(end,:)';
    
    %ruido de medida
    x(1,k+1) = x(1,k+1)+ ruidoPa(k);
    x(4,k+1) = x(4,k+1)+ ruidoVa(k);
    
    %filtro
    x_fil(1:3,k+1) = x(1:3,k+1);
    x_fil(5:6,k+1) = x(5:6,k+1);
    x_fil(4,k+1) = -a(2)*x_fil(4,k)-a(3)*x_fil4_km1+b(1)*x(4,k+1)+b(2)*x(4,k)+b(3)*x4km1;
    x_fil4_km1 = x_fil(4,k);
    x4km1 = x(4,k);
    y(:,k) = [x_fil(1,k) x_fil(4,k)];
    %saida
    x_hat(:,k+1) = L*(y(:,k) - y_hat(k));%A*x_hat(:,k)
    y_hat(:,k) = C * x_hat(:,k);
    
    ukm2 = ukm1;
    ukm1 = u(:,k);
end

y = y*180/pi;%rad -> graus
plotdata(u,y,x_fil,kmax);

