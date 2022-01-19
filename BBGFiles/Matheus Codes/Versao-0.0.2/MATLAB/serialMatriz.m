function [serialVector] = serialMatriz(M)
% Serializa matriz (input) em vetor (1xn)
lenVetM_aux = size(M);
lenVetm = lenVetM_aux(1)*lenVetM_aux(2);
serialVector = [];
for i=1:lenVetm
    serialVector = [serialVector M(i)];
end