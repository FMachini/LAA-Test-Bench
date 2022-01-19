function [Hqp, cqpaux,deltaUmax, deltaUmin, Phi, Sqp, Ymax, Ymin, umax, umin, Klqr ] = loadMpcMatrices()
Hqp = dlmread('Matrices_26_03_21\Hqp.txt');
cqpaux = dlmread('Matrices_26_03_21\cqpaux.txt');
deltaUmax = dlmread('Matrices_26_03_21\deltaUmax.txt');
deltaUmin = dlmread('Matrices_26_03_21\deltaUmin.txt');
Phi = dlmread('Matrices_26_03_21\Phi.txt');
Sqp = dlmread('Matrices_26_03_21\Sqp.txt');
Ymax = dlmread('Matrices_26_03_21\Ymax.txt');
Ymin = dlmread('Matrices_26_03_21\Ymin.txt');
umax = dlmread('Matrices_26_03_21\umax.txt');
umin = dlmread('Matrices_26_03_21\umin.txt');
Klqr = dlmread('Matrices_26_03_21\Klqr.txt');
end

