clear all;clc;

SystemParameters;

At  = [Ad(1:4,1:4) Ad(1:4,7);
       Ad(7,1:4) Ad(7,7)];
%At  = [Ad(1:4,1:4)];
          
% Ct = zeros(3,5);
% Ct(1,1) = 1; Ct(2,2)=1; Ct(3,3)=1;
% Cx1 = zeros(3,3); Cx1(3,3)=1;
Ct = zeros(4,2); Ct(3,1)=1;Ct(4,2)=1;
 Cx = [eye(4,5) Ct];
Ax = [At zeros(5,2);zeros(2,5) eye(2)];
rank(obsv(Ax,Cx))