%% Estrutura simplificada do hil 'atuante'
% 
% # Setar parametros iniciais
% # Receber o estado da planta (da beagle bone)
% # Receber sinal de controle (velocidade dos rotores) do controlador
% # Repetir processos 2 e 3 por 'numero de iteracoes' de vezes
% # Fim
%% Lista de parametros principais
% 
% * XX: estado da aeronave -- coordenadas, orietacao, velocidades lineares e
% angulares
% * omega: velocidade dos rotores
% 

%% Limpar dados anterior
clc; clear all;

%% Setar parametros iniciais
nIteracoes = 5000; % numero de iteracoes
XX = zeros(1, 12); % estados a serem recebidos pela BBG
omega = zeros(1,4); % velocidade dos rotores
dt = 0.02; % tempo de amostragem em segundos

%K1 = load("K1.txt");
%Kx = load("Kx.txt");

%carregando matrizes de ganho (K1 e Kx)
load matrices.mat;

%Coeficientes para o polinomio de referencia. Unidades em metros
CoefRef = [0 0; 0 0; 0 0.1];

%matriz de trimagem. <<Notacao: M3, M1, M4, M2>>
trim = [+377.8826; -377.8826; +377.8826; -377.8826];

%Transformar matrizes num vetor para comunicacao entre bbg e MATLAB
K1Vet = serialMatriz(K1);
KxVet = serialMatriz(Kx);
CoefRefVet = serialMatriz(CoefRef);
trimVet = serialMatriz(trim);

%% Configuracao de rede - comunicacao
IP_BeagleBone = '10.0.33.236';
IP_Local = '127.0.0.1';
PORT = 8080; % definda de forma arbritaria

DATA_TYPE = 'double'; % melhorar legibilidade do codigo
NUMBER_OF_DATA_2_RCV = 4; % melhorar legibilidade do codigo
                          % 4 pq sao as velocidades dos quatro rotores do
                          % drone
                            
%criando socket - Create a TCP/IP Connection
t_obj = tcpclient(IP_BeagleBone, PORT)
%t_obj = tcpclient(IP_Local, PORT)

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

%% Enviar parametros iniciais para a Beagle Bone
%Enviar dados que representam a matriz K1
write(t_obj, K1Vet);
%Enviar dados que representam a matriz Kx
write(t_obj, KxVet);
%Enviar dados que representam a matriz de coeficientes do polinomio de
%referencia
write(t_obj, CoefRefVet);
%Enviar o valor do tempo de amostragem
write(t_obj, dt);
%Enviar dados que representam a matriz de trimagem 'trim'
write(t_obj, trimVet);
%Enviar dados que representam a matriz K1
write(t_obj, nIteracoes);

%% Armazenar dados para analise
%preallocando memoria para variaveis para melhorar eficiencia do codigo
xx_ = zeros(nIteracoes, 12); %matriz para armazenar estados passados como um historico
omega_ = zeros(nIteracoes, 4); %matriz para armazenar velocidades passadas como um historico
vetor_tempo_loop = zeros(nIteracoes, 1);

%% Armazenar e calcular dados para analise de tempo de processamento
%Variavel utilizada para obter o tempo gasto por cada iteracao 
tempo_loop = 0;

%Variavel para medir o tempo que a execucao de todo o loop demorou
tempo_exec_inicial = tic

%% Inicio da aplicacao de controle a atuacao
for i = 1:nIteracoes
    %Marca o tempo inicial da iteracao i para posteriormente ser calculado
    %o tempo gasto ate o final da mesma iteracao
    tStart = tic;        

    %receber o estado da aeronave da BBG
    XX = read(t_obj, 12, DATA_TYPE);
    %manter historico para plotar posteriormente
    xx_(i, :) = XX;

    % Receber dado da BBG (variavel de saida de controle)
    omega = read(t_obj, NUMBER_OF_DATA_2_RCV, DATA_TYPE);
    omega_(i, :) = omega;
    
    %Registra o tempo que pode ser condiferado como o final da iteracao
    tempo_loop = toc(tStart);
    
    %Variavel utilizada para guardar um historico do tempo gasto por cada
    %iteracao
    vetor_tempo_loop(i, :) = tempo_loop;
    
    %Forcar que o loop ocorra a cada 0.02 segundos, mesmo em casos que o
    %tempo de calculo e comunicacao com a BBG seja inferior a 0.02.
    pause(dt - tempo_loop);
    
end

%% Fim de execucao. Analise dos resultados obtidos

%Tempo gasto para executar todas as iteracoes
tempo_exec = toc(tempo_exec_inicial);

% limpando recursos utilizados na comunicao
clear t_obj %liberando recursos relacionado ao objeto TCP/Ip

%%somente para debug
plot(xx_(:,11))
