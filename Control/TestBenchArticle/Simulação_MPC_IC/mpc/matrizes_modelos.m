%% Gabriel Renato Oliveira Alves 
%% MPC - classico

function [LL,Ab,Bb,Cb,K,Atil,Btil,Ctil,Ak] = matrizes_modelos(A,B,C,Qr,Rr,Qo,Ro)
%% Auxiliares
n = size(A,1);
p = size(B,2);
q = size(C,1);
Ip = eye(p);

%% Ganho do LQR
K = dlqr(A, B, Qr, Rr);

%% Modelo com LQR
Ak = A - B*K;

%% Modelo chi
Ab = [Ak, zeros(n,q);zeros(q,n),eye(q)];
Bb = [B;zeros(q,p)];
Cb = [C, eye(q)];

% L = place(Ab',Cb',polos)';
% LL = inv(Ab) * L;
L = dlqr(Ab',Cb',Qo,Ro)';
LL = inv(Ab) * L;

%% Auxiliares
nb = size(Ab,1);
pb = size(Bb,2);
qb = size(Cb,1);
Ipb = eye(pb);

%% Modelo csi
Atil = [Ab Bb;zeros(pb,nb) Ipb];
Btil = [Bb;Ipb];
Ctil = [Cb zeros(qb,pb)];
end

