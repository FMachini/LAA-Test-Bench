function [K] = controlador(A,B,C,D,Qr,Rr)
%Projeto do regulador (K)
    %Utilizando o LQR
    K = dlqr(A, B, Qr, Rr);
end

