clear all; close all;clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Model Predictive control implementation%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Multicopter Non-Linear Problem %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% MPC with terminal, input and state restrictions %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% Author: Felipe Machini M. Marques %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Ts = 0.01; %sample time
numit = 2000; % number of iterations

%% Configuração solver
options1 = optimset('Display','off','MaxIter',10000); %quadprog

%% Multicopter parameters

param.n = 4; % number of eletric motors
param.m = 0.468; % Multirrotor mass
param.J = [0.049,0,0;
           0,0.049,0;
           0,0,0.088];%inertia matrix
param.kd =4.8e-3; % drag coefficient
param.k = 2.9e-5; %thrust coefficient
param.l = ones(1,param.n)*0.225; %multirrotor arm length
param.gamma = [0,90,180,270]*pi/180; % multirrotor arm angle wrt x axis
param.JJm = ones(1,param.n)*3.357e-5; % propeller moment of inertia
param.b = 1.1e-6; %propeller drag coefficient
param.zcg = ones(1,param.n)*0; % Center of gravity z position BCS
param.g = 9.81;

hover = sqrt((param.m*param.g)/(param.k*param.n));
omg0 = hover*[1 -1 1 -1];
param.omg0=omg0;
tilt1 = 4;
param.tilt1 = tilt1;
tilt2 = 0;
param.tilt2 = tilt2;
num_inp = param.n+tilt1+tilt2;

numstates = 7;
%% Obtaining state space matrix x_dot = Ax +Bu

[A,B,C,D] = Matrices_new(param,omg0);

A1 = zeros(7,7);
A1(1,1)=-param.kd/param.m; A1(1,7)=param.g;
A1(2,2) = -param.kd/param.m; A1(2,6)=-param.g;
A1(3,3) = -param.kd/param.m;
A1(6,4)=1;
A1(7,5)=1;

B1 = [B(1:5,:);B(7:8,:)];
C1 = eye(size(A1));
D1 = zeros((size(A1,1)),size(B1,2));
%% Creating discrete state space model

[Ad,Bd,Cd,Dd] = c2dm(A1,B1,C1,D1,Ts);
 
 %% Calculating feedback LQR gain matrice

qv = 0.5e-1*[1,1,1];
qva =3e1*[1,1];
qa = 3e1*[1,1];
Q=diag([qv,qva,qa]);
rho=1;
rm = 5e-5;rt = 5e7;
R=rho*diag([rm,rm,rm,rm,rt,rt,rt,rt]);

Klqr = dlqr(Ad,Bd,Q,R); %calculating discrete LQR gain matrix

% Closed loop model
Acl = Ad-Bd*Klqr;eig(Acl)
Bcl = Bd;
Ccl = eye(size(Acl));
Dcl = zeros(size(Bcl));

umax = [60*[1,1,1,1] 0.5*[1,1,1,1]]';
umin = -umax;
xmax=[3,3,3,0.1,0.1,0.5,0.5]';
xmin = -xmax;

[Amf, Smas] = terminalconstraint(Ad, Bd, Klqr, Ts, 7, umax, umin, xmax, xmin);

%% Prediction model
ny = 3 ; % number of predictions
[Hmpc,Pmpc,L,M] = imgpc_predmat(Acl,Bcl,Ccl,Dcl,ny); %Prediction Model

i_cond = 10; % angular initial condition in deg
Qq = 100*eye(numstates*ny);
Hh = Hmpc'*Qq*Hmpc + 1*eye(ny*num_inp);
Hh=(Hh+Hh')/2;

%% Restrictions Parameters

% Restrictions
%Aqp = blkdiag(eye(ny*num_inp),-eye(ny*num_inp));
bqp_max = umax;
bqp_min= umin;
fbx_max = xmax;
fbx_min = xmin;
fu = -Klqr;
%Kappa = [[zeros(size(Klqr)),zeros(size(Klqr))]; [-Klqr,zeros(size(Klqr))]];

for jj = 1:ny-1
    fbx_max = [fbx_max;xmax];
    fbx_min = [fbx_min;xmin];
    bqp_max = [bqp_max;umax];
    bqp_min = [bqp_min;umin];
    fu = [fu;zeros(size(Klqr))];
end

Kappa = zeros(ny*num_inp,ny*numstates);
Kpa = -Klqr;
for jj = 1:ny-2
    Kpa =blkdiag(Kpa,-Klqr);
end
Kappa(num_inp+1:end,1:end-numstates) = Kpa;
    
bqp = [bqp_max;-bqp_min];
%Aqp = blkdiag((eye(ny*num_inp)+Kappa*Hmpc),-(eye(ny*num_inp)+Kappa*Hmpc));
Mat2 = Acl*Bd;
for n = 1:ny
    
Mm = Acl^(ny-n)*Bd; 
int = 1+(n-1)*size(Mat2,2):n*size(Mat2,2);
Mat(:,int) = Mm;
end
Aqpx = [Hmpc;-Hmpc];
Aqp = [eye(ny*num_inp)+Kappa*Hmpc ; -(eye(ny*num_inp)+Kappa*Hmpc) ; Smas.A*Mat; Aqpx];

%% Simulation

tt = zeros(numit);tt(1)=0;
x = zeros(numstates,numit);
icond = zeros(9,1);icond(7:8) = i_cond*[1;1]*pi/180;
ulqr = zeros(numit,num_inp);
Umpc = zeros(numit,num_inp);
ut = zeros(numit,num_inp);
flag  = zeros(numit,1);

for i = 2:numit
    tt(i+1) = tt(i)+Ts;
%Regulator control Law
    ulqr(i,:) = -Klqr*x(:,i);
        
    %% Predictive Control Law
    
    %f = 2*(Pmpc*x2(:,i))'*Qq*Hmpc;
    f = 2*x(:,i)'*Pmpc'*Qq*Hmpc;

    % Restrictions update
    bqp = [bqp_max-(fu+Kappa*Pmpc)*x(:,i);-(bqp_min-(fu+Kappa*Pmpc)*x(:,i)) ; Smas.b-Smas.A*Acl^ny*x(:,i); fbx_max - Pmpc*x(:,i); -fbx_min + Pmpc*x(:,i)];
    
    % Predictive input
    [umpc, Jcusto, flag(i)] = quadprog(2*Hh,f,Aqp,bqp, [], [], [] ,[] ,[Umpc(i-1,2:end) 0] ,options1);
   % [umpc, lambdadummy, how, flag(i), Jcusto] = mpt_solveQP(2*Hh,f,Aqp,bqp, [], [], [Umpc(i-1,2:end) 0], 5 ,options1);
    [aa,bb]=size(Klqr);
    Umpc(i,:) = umpc(1:aa);
    ut(i,:) = ulqr(i,:) + Umpc(i,:);     
%     State estimation 
    omg = ut(i,1:4);
    beta = zeros(1,4);
    alpha = ut(i,5:8);
    [time,XX] = ode45(@(t,Y) Hexa(Y,param,omg,alpha,beta,t),[tt(i) tt(i+1)],icond);
    icond=XX(end,:)';
    x(:,i+1) = [XX(end,1:5),XX(end,7:8)]';
end

%% Lqr Simulation
x2 = zeros(size(x));
icond = zeros(9,1);icond(7:8) = i_cond*[1;1]*pi/180;
u2 = zeros(numit,num_inp);

for i = 1:numit
    tt(i+1) = tt(i)+Ts;
%     Regulator control Law
    u2(i,:) = -Klqr*x2(:,i);     
%     State estimation 
    omg = u2(i,1:4);
    beta = zeros(1,4);
    alpha = u2(i,5:8);
    [time,XX] = ode45(@(t,Y) Hexa(Y,param,omg,alpha,beta,t),[tt(i) tt(i+1)],icond);
    icond=XX(end,:)';
    x2(:,i+1) = [XX(end,1:5),XX(end,7:8)]';
end


% %% Display results
t = 0:Ts:numit*Ts; %time vector
%
figure
subplot(3,1,1)
plot(t,x(1,:),t,x2(1,:),'-.')
title ('Linear velocity')
ylabel 'x_{1}'
subplot(3,1,2)
plot(t,x(2,:),t,x2(2,:),'-.')
ylabel 'x_{2}'
subplot(3,1,3)
plot(t,x(3,:),t,x2(3,:),'-.')
ylabel 'x_{3}'

figure
subplot(3,1,1)
plot(t,x(4,:),t,x2(4,:),'-.')
title ('Angular velocity')
ylabel 'x_{4}'
subplot(3,1,2)
plot(t,x(5,:),t,x2(5,:),'-.')
ylabel 'x_{5}'
subplot(3,1,3)
plot(t,zeros(size(t)),'-.')
ylabel 'x_{6}'

figure
subplot(3,1,1)
plot(t,x(6,:),t,x2(6,:),'-.')
title ('Euler Angles')
ylabel 'x_{7}'
subplot(3,1,2)
plot(t,x(7,:),t,x2(7,:),'-.')
ylabel 'x_{8}'
subplot(3,1,3)
plot(t,zeros(size(t)),'-.')
ylabel 'x_{9}'

figure
subplot(2,1,1)
plot(ut(:,1:4))
title ('Motors')
ylabel 'u_{mpc}'
subplot(2,1,2)
plot(t(1:end-1),u2(:,1:4))
ylabel 'u_{lqr}'

figure
subplot(2,1,1)
plot(ut(:,5:8))
title ('Tilt angles')
ylabel 'u_{mpc}'
subplot(2,1,2)
plot(t(1:end-1),u2(:,5:8))
ylabel 'u_{lqr}'