%LQRi - Controle de atitude (theta4)

function [Ki,K,Nx] = controlador(A,B,C,D,Qr,Rr,Ts)
    % Auxiliares
    p = size(B,2);
    q = size(C,1);
    n = size(A,1);
    
    %Projeto do regulador (K)
    %Utilizando o LQRi
    A = [eye(q), Ts*C;
         zeros(n,q), A];
    B = [zeros(q,p);
         B];
    
    Nx = [eye(q);
          zeros(n-q,q)];
    
    Kt = dlqr(A, B, Qr, Rr);
    
    Ki = Kt(:,1:q);
    K = Kt(:,q+1:end);
end

