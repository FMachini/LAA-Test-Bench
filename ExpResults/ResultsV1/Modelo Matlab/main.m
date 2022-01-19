clear;
close all;
clc;
%Script para carregar os dados da planta
[A,B,C,D,Ts] = paramplanta();

%Auxiliares
p = 6;%size(B,2);
q = 1;%size(C,1);
n = 6;%size(A,1);

%Parametros de projeto
Qr = 1e2*diag([1, 10]);
Rr = 200*diag([1, 1]);
uLim = 100*[10, -10];
Td = 0.01*Ts; %(s)atraso de transporte;

%Script para carregar os dados do controlador
Kc = controlador(A,B,C,D,Qr,Rr);
%Definindo variaveis de estado e controle
kmax = 50;
%Estado x
x = zeros(n,kmax+1);
%Entrada
u = zeros(2,kmax);            
%Saida
y = zeros(q,kmax);

%Condicoes Iniciais
%xini = [-pi/6 zeros(1,n-1)];
xini = [-10*pi/180 zeros(1,n-1)];
x(:,1) = xini';
ukm1 = zeros(p,1);
ukm2 = zeros(p,1);

%referencia
%Matriz M nao e quadrada portanto nao e inversivel
%Trataremos inicialmente com um regulardor

%ruido posicao angular - Range de 0.05 rad
rng(0,'twister');
a = -0.025*0;
b = 0.025*0;
ruidoPa = (b-a).*rand(kmax,1) + a;
%ruido velocidade angular - Range de 0.4 rad/s
a = -0.08*0;
b = 0.08*0;
ruidoVa = (b-a).*rand(kmax,1) + a;

%filtro Butterworth
fs = 25;      %Sample frequency(Hz)
fn = fs/2;      %Nyquist frequency (midpoint of frequency range)
fc = 7;         %Cutoff frequency
[b,a] = butter(2, fc/fn);
x_fil = x;
x_fil4_km1 = 0;
x4km1 = 0;

%Configuraçao ode45
options2 = odeset('Reltol',1e-6,'AbsTol',1e-6);
%Evolucao da dinamica da planta
for k = 1:kmax
    u(:,k) = -Kc * [x_fil(1,k);x_fil(4,k)];
  
    %saturacao da entrada
    u(:,k) = min(uLim(1),u(:,k));
    u(:,k) = max(uLim(2),u(:,k));
    
    xini = x_fil(:,k);
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
    
    %saida
    y(:,k) = [1, 0, 0, 0, 0, 0]* x_fil(:,k);
    
    ukm2 = ukm1;
    ukm1 = u(:,k);
end

y = y*180/pi;%rad -> graus
plotdata(u,y,x_fil,kmax);

