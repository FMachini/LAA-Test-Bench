clear all; close all

ip_bbg = '10.0.33.236';
ip = '127.0.0.1';
port = 8080;

%t = tcpclient('127.0.0.1', 8080);
%data = read(t, 1, 'double');
%clear t;


% t = tcpip('127.0.0.1', 8080);
% fopen(t);
% data = fscanf(t);
% fclose(t);
% delete(t);
% clear t


% echotcpip('on',4012);
% t = tcpip('localhost',4012);
% fopen(t);
% fwrite(t, 65:74);
% A = fread(t, 10);
% fclose(t);
% echotcpip('off');



% t=tcpclient(ip, port);
% 
% while 1
%     data=read(t,1,'double') % read tem que saber o numero de bytes que serao enviados 
%     data=data+1
%     write(t, data)
%     pause(1)
% % end
% 
% clear t

% data = 0;
% 
% t = tcpip(ip, port, 'NetworkRole', 'client');
% 
% fopen(t);
% data=fread(t, 1);
% %data=str2num(data);
% data=data+1;
% %data=num2str(data);
% fwrite(t, data)
% 
% fclose(t)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%t = tcpclient(ip, port)

%data = read(t)

%% testando receber vetores e matrizes

% %m
% data = read(t, 6, 'double')
% %m1
% data1 = read(t, 6, 'double')
% %m2
% data2 = read(t, 4, 'double')
% %m3
% data3 = read(t, 4, 'double')

% %i1 = 2008
% data = read(t, 1, 'double')
% %f1 = 3.14 - float === deu problema
% %data1 = read(t, 1, 'double')
% % st = "abc" - string === deu problema
% data2 = read(t, 1, 'uint8')

%% testando enviar matrizes para o servidor (bbg)

XX = [rand() rand() rand()];
XX1 = [rand()+1 rand()-1; rand()+5 rand()-5];
XX2 = [rand()+1; rand()-1; rand()+1; rand()-1];
XX3 = [rand() rand(); rand() rand(); rand() rand()];

%write(t, XX1) % problema: a função write aceita vetores e nao matrizes
%para tentar enviar xx1, tranformar-lo em vetor e depois enviar

%serializar matrix XX1 no vetor sXX1 (1x4)
sXX1 = serialMatriz(XX1);
sXX2 = serialMatriz(XX2);
sXX3 = serialMatriz(XX3);

% write(t, XX);
% write(t, sXX1);
% write(t, sXX2);
% write(t, sXX3);

%% testando o envio do vetor como sera feito no hil
% XX envia para bbg

%XX = [1 2 3 4 5 6 7 8 9 10 11 12];

%% 

% send_value = rand();
% write(t, send_value);
% data = read(t, 1, 'double')

K1 = load("K1.txt");
Kx = load("Kx.txt");
CoefRef = [0 1.0; 0 1.0; 0 1.0];
dt = 0.1798;
%trim = [1;2;3;4];
trim=[198.9429;-198.9429;198.9429;-198.9429];
XX = rand(1, 12);

t = tcpclient(ip, port)

K1Vet = serialMatriz(K1);
KxVet = serialMatriz(Kx);
CoefRefVet = serialMatriz(CoefRef);
trimVet = serialMatriz(trim);

write(t, K1Vet);
write(t, KxVet);
write(t, CoefRefVet);
write(t, dt);
write(t, trimVet);
write(t, XX);

for i = 1:1000
    [XX] = ode45(@(t,y)Hexa(), )
    write(t,XX);
    dataRecv(:,:,i) = read(t, 4, 'double')
end

clear t
