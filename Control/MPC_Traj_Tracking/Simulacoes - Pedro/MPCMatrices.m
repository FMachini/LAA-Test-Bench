function [Hqp, cqpaux, deltaUmax, deltaUmin, Phi, Sqp, Ymax, Ymin] = MPCMatrices(A, B, C, Q, R, N, M, deltaumax, deltaumin, ymax, ymin)
%auxiliary variables
nu = size(B,2);
nx = size(A,1);
ny = size(C,1);

row = [1 zeros(1, nu*M - 1)];
col = [1;zeros(nu-1,1)];
for i = 2:M
    col = [col;
           1;
           zeros(nu-1,1)];
end
TMIp = toeplitz(col,row);

% constraints
Ymax = repmat(ymax,N,1);
Ymin = repmat(ymin,N,1);
deltaUmax = repmat(deltaumax,M,1);
deltaUmin = repmat(deltaumin,M,1);

%Weighting matrices
Qb = Q;
for i = 2:N
    Qb = blkdiag(Qb, Q);%ok
end

Rb = R;
for i = 2:M
    Rb = blkdiag(Rb, R);%ok
end

%predictions
Atil = [A              B;
        zeros(nu, nx)  eye(nu, nu)];%ok
Btil = [B;
        eye(nu,nu)];%ok
Ctil = [C zeros(ny, nu)];%ok

Phi= Ctil*Atil;
for i = 2:N
    Phi = [Phi; 
           Ctil*Atil^i];%ok
end

%G
for i = 1:N
    for j = 1:M
        if i >= j
            G{i,j} = Ctil*Atil^(i-j)*Btil;
        else
            G{i,j} = zeros(ny, nu);
        end
    end
end
G = cell2mat(G);%ok

% Quadprog Matrices
Hqp = 2*(G'*Qb*G + Rb); %ok
Hqp = (Hqp+Hqp')/2;
cqpaux = 2*G'*Qb;

Sqp = [eye(nu*M, nu*M);
      -eye(nu*M, nu*M);
       TMIp;
      -TMIp;
       G;
      -G];%ok

