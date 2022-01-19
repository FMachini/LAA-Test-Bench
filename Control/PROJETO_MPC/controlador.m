function [Hqp,Aqp,Gn,phi] = controlador(A,B,C,D,N,M,rho,mi)
% Auxiliares
n = size(A,1);
p = size(B,2);
q = size(C,1);
Ip = eye(p);

Atil = [A B;zeros(p,n) Ip];
Btil = [B;Ip];
Ctil = [C zeros(q,p)];

G = zeros(q*N,p*M);
for i = 1:N
    for j = 1:min(i,M)
        G(1+(i-1)*q:i*q , 1+(j-1)*p:j*p) = Ctil*(Atil^(i-j))*Btil;
    end
end
Q = diag(mi);
R = diag(rho);

Qn = Q;
phi = Ctil*Atil;
for i=1:N-1
    phi = [phi;Ctil*Atil^(i+1)];
    Qn = blkdiag(Qn,Q);
end
Rm = R;
for i = 1:M-1
    Rm = blkdiag(Rm,R);
end
Gn = Qn*G;

Hqp = 2*(G'*Qn*G+Rm);
 
IpM = eye(p*M);
row = [1 zeros(1,p*M-1)];
col = [1;zeros(p-1,1)];
col = repmat(col,M,1);
TmIp = toeplitz(col,row);
%[IpM; -IpM];  % Restricao sobre o du
%[TmIp; -TmIp];    % Restricao sobre u
%[G; -G];          % Restricao sobre y
Aqp = [IpM; -IpM; TmIp; -TmIp];

end

