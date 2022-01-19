clear all; close all; clc

[Ad,Bd] = LinearIdentifiedModel(0.02);

Cd = eye(4,7);
Cd(1,1) = 1; Cd(2,2)=1; Cd(3,3)=1;Cd(4,5)=1;

Qob = diag([1 1 1 5 1 5 10]);

Rob = diag([1 1 1 1]);

L = dlqr(Ad',Cd', Qob,Rob);
L = L';

kend = 200;
x = zeros(7,kend);

