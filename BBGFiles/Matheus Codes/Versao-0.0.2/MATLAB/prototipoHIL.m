% Estrutura do hil
% 1 - setar parametros iniciais
% 2 - calcular o estado da planta
% 3 - enviar o estado da planta para o controlador
% 4 - receber sinal de controle (velocidade dos rotores) do controlador
% 5 - repitir processos 2,3 e 4 por n vezes
% 6 - fim

clc; clear all;

% Setar parametros iniciais
nIteracoes = 1000;              % numero de iterações
%XX = zeros(1, 12);              % estados a serem enviados para BBG
XX = zeros(1, 12);              % estados a serem recebidos pela BBG
omega = zeros(1,4);             % velocidade dos rotores
dt = 0.02;
t_int_inicial = 0;
t_int_final = dt;             % 
%dt = 0.02;
%condicoes_iniciais = zeros(size(XX));

%K1 = load("K1.txt");
%Kx = load("Kx.txt");

%testando matrizes de ganho que machini disse que funcionaram
load matrices.mat;

CoefRef = [0 0; 0 6; 0 0];

K1Vet = serialMatriz(K1);
KxVet = serialMatriz(Kx);
CoefRefVet = serialMatriz(CoefRef);

%posso dizer que o primeiro elemento é a velocidade do primeiro motor
%entao 
%trim(1) = -
%trim(2) = -
%trim(3) = +
%trim(4) = +
%?

%pelo 'testando' vi que nao da. Aparentemente nao converge. Entao tem que 
%pegar os valores de trim e realocar eles conforme desejado dentro do
%codigo em c

%original
trim = [198.9429; -198.9429; 198.9429; -198.9429];

%testando
%trim = [-198.9429; -198.9429; 198.9429; 198.9429];

trimVet = serialMatriz(trim);

% configuracao de rede - comunicacao
IP_BeagleBone = '10.0.33.236';
IP_Local = '127.0.0.1';
PORT = 8080;                     % definda de forma arbritaria

DATA_TYPE = 'double'; % melhorar legibilidade do código
NUMBER_OF_DATA_2_RCV = 4; % melhorar legibilidade do código
                            % 4 pq são as velocidades dos quatro rotores do
                            % drone
                            
%criando socket - Create a TCP/IP Connection
t_obj = tcpclient(IP_BeagleBone, PORT)
%t_obj = tcpclient(IP_Local, PORT)

% Iniciar hardware in the loop

%% Muticopter properties

param.n = 4; % number of eletric motors
param.m = 0.468; % Multirrotor mass
param.J = [0.0049,0,0;
           0,0.0049,0;
           0,0,0.0088];%inertia matrix
param.kd =4.8e-3; % drag coefficient
param.k = 2.9e-5; %thrust coefficient
param.l = ones(1,param.n)*0.225; %multirrotor arm length
param.gamma = [0,90,180,270]*pi/180; % multirrotor arm angle wrt x axis
param.JJm = ones(1,param.n)*3.357e-5; % propeller moment of inertia
param.b = 1.1e-6; %propeller drag coefficient
param.zcg = ones(1,param.n)*0; % Center of gravity z position BCS
param.g = 9.81;



omega = trim;

write(t_obj, K1Vet);
write(t_obj, KxVet);
write(t_obj, CoefRefVet);
write(t_obj, dt);
write(t_obj, trimVet);
%write(t_obj, XX);

xx_ = zeros(nIteracoes, 12);
omega_ = zeros(nIteracoes, 4);

tempo_loop = 0;
tempo_exec_inicial = tic
for i = 1:nIteracoes
    tStart = tic;        

    %receber da bbg
    XX = read(t_obj, 12, DATA_TYPE);
    %manter historico para plotar posteriormente
    xx_(i, :) = XX;
    
    %cálculo do estado da planta
    
    % Enviar dado para BBG (estado da planta)
    %DATA_2_SEND = XX; % melhorar legibilidade do código
    %write(t_obj, DATA_2_SEND);

    % Receber dado da BBG (variavel de saida de controle)
    omega = read(t_obj, NUMBER_OF_DATA_2_RCV, DATA_TYPE);
    omega_(i, :) = omega;
    
    %condicoes_iniciais = XX;
    t_int_inicial = t_int_final;
    t_int_final = t_int_final + dt;
    
    tempo_loop=toc(tStart);
    vetor_tempo_loop(i) = tempo_loop;
    pause(dt-tempo_loop);
    
end

tempo_exec = toc(tempo_exec_inicial);
% limpando recursos utilizados

clear t_obj %liberando recursos relacionado ao objeto TCP/Ip

%%somente para debug
plot(xx_(:,11))
