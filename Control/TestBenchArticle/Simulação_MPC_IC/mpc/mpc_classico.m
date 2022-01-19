%% Gabriel Renato Oliveira Alves 
%% MPC - classico

function [Pdu,Pu,PI,Hdu,Hu,Hqp,Aqp,Gn,phi] = mpc_classico(Ak,B,C,Atil,Btil,Ctil,N,M,Q,R,K)
%% Auxiliares
n = size(Ak,1);
p = size(B,2);
q = size(C,1);

%% Variaveis da equacao de predicao dos estados [x]
Px = Ak;
for i=1:N-1
    Px = [Px;Ak^(i+1)];
end
Hx = zeros(n*N,p*N);
for i = 1:N
    for j = 1:min(i,N)
        Hx(1+(i-1)*n:i*n , 1+(j-1)*p:j*p) = (Ak^(i-j))*B;
    end
end

%% Variaveis da equacao de predicao do ulqr
Plqr = [-K;zeros(p*(N-1),n)];
Hlqr = zeros(p*N,n*N);
for i = 2:N
    j = i-1;
    Hlqr((i-1)*p+1:i*p,(j-1)*n+1:j*n) = -K;
end

%% Variaveis da equacao de predicao do controle [u] em termos do uMpc
Pu = Plqr + Hlqr*Px;
Hu = Hlqr*Hx + eye(p*N);

%% Variaveis da equacao de predicao das variaveis controladas [y] em termos
% de duMpc
phi = Ctil*Atil;
for i=1:N-1
    phi = [phi;Ctil*Atil^(i+1)];
end

G = zeros(q*N,p*M);
for i = 1:N
    for j = 1:min(i,M)
        G(1+(i-1)*q:i*q , 1+(j-1)*p:j*p) = Ctil*(Atil^(i-j))*Btil;
    end
end

%% Variaveis da equacao de predicao do controle [u] em termos do duMpc
row = [1 zeros(1,p*M-1)];
col = [1;zeros(p-1,1)];
col = repmat(col,N,1);
T_NxM_Ip = toeplitz(col,row);

HT = Hu*T_NxM_Ip;

%% Variaveis da equacao de predicao do incremento de controle total [du] em
% termos do duMpc
HI = eye(p*N);
for i = 2:N
    j = i-1;
    HI((i-1)*p+1:i*p,(j-1)*p+1:j*p) = -eye(p);
end
PI = [eye(p);zeros(p*N-p,p)];

Pdu = HI*Pu;
Hdu = HI*Hu;
HIT = HI*HT;

%% Variaveis do problema de programacao quadratica
Qn = diag(repmat(diag(Q),N,1));
Rm = diag(repmat(diag(R),M,1));

Gn = Qn*G;
Hqp = 2*(G'*Qn*G+Rm);
 
%[HIT; -HIT];      % Restricao sobre o du
%[HT; -HT];        % Restricao sobre u
%[G; -G];          % Restricao sobre y
Aqp = [HIT; -HIT; HT; -HT];
end

