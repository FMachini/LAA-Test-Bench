%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% State Observer Design
% Design State Observer From Acquired Experimental Data
% Author : Felipe Machini
% Date: 07/05/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear all; close all; clc

% Get experimental data signal 
% A = [Theta1(deg) Theta2(deg) Theta4(deg) Theta1dot(rad/s) Theta2dot(rad/s)... 
% Theta4dot(rad/s) f1(N) f2(N) alpha1(deg) alpha2(deg)]
global A Ad Bd Cd Cc vel Axd Cxd Bxd;
A = textread('Results_26_03_2021.txt');

%% Model and control parameters
SystemParameters;

%% State Observer
Cd = zeros(3,6);
Cd(1,1) = 1;Cd(2,3)=1; Cd(3,5)=1;
% Qob = diag(1*[1 1 1 1 1 1 16 16 16]);
% 
% Rob = diag(1*[5 10 10]);

% Ax = [Ad zeros(6,3);
%        zeros(3,6) eye(3)];
% Bx = [Bd; zeros(3,4)];
%Cx1 = zeros(3,2); Cx1(1,1)=1;Cx1(2,2)=1;
% Cx = [Cd eye(3)];
 load 'OptWeights2.mat'
Qob = diag(out(1:9))
Rob = diag(out(10:end))
L = dlqr(Axd',Cxd', Qob,Rob);
% 
% try 
%     L = dlqr(Axd',Cx', Qob,Rob);
% 
% catch L
%     if (strcmp(L.identifier,'Control:design:lqr2'))
%         L = ones(3,9)*1e6;
%     end
% end
% L = inv(Ax)*L;
L = L';
% damp(Ad-L*Cd)
 damp(Axd-L*Cxd)

%% Test Observer 
Cc = [0 0 1*pi/180 0 0 0;
      1*pi/180 0 0 0 0 0;
      0 1*pi/180 0 0 0 0];


  xmpck = zeros(9,size(A,1)+1);
  up = zeros(4,size(A,1));
for  k = 1:size(A,1)
   up(:,k) = [A(k,7)-1.7 A(k,8)-1.7 A(k,9)*pi/180 A(k,10)*pi/180]';
   yout = Cc*A(k,1:6)';
   
   % State Observer prediction
   ye = Cxd*xmpck(:,k);
   xmpck(:,k+1) = Axd*xmpck(:,k) + L*(yout - ye) + Bxd*up(:,k);   
    
end

figure;
plot(xmpck(4,:)*180/pi)
vel = diff(A(:,3)/0.02);
velFiltered = movmean(vel,5);
hold on
plot(velFiltered)
plot(vel)
%vel = velFiltered;
plot(A(:,4)*180/pi)

% Optimization

%out = ga(@CostFun2,12,[],[],[],[],1e-4*ones(12,1),1e4*ones(12,1))

