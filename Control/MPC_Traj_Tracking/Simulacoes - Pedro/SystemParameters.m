%% System Parameters
m = 0.468; % Multirrotor mass
J = [0.049 0      0;
     0     0.049  0;
     0     0      0.088];%inertia matrix
kd = 4.8e-3; % drag coefficient
l = 0.225; %multirrotor arm length
kp_m = 2.9e-5; %thrust coefficient
b = 1.1e-6; %propeller drag coefficient
JJm = 3.357e-5; % propeller moment of inertia
gamma = [0  90  180  270]*pi/180; % multirrotor arm angle wrt x axis
zcg = 0; % Center of gravity z position BCS
g = 9.81;

param.m = m;
param.J = J;
param.kd = kd;
param.kp_m = kp_m;
param.l_b = l;
param.gamma = gamma;
param.Jm = JJm*ones(1,4);
param.b = b;
param.g = g;

%% Entradas de trimagem
alpha_0 = zeros(4,1);

omg_0_vec = [sqrt((m*g)/(kp_m*4)); 
             -sqrt((m*g)/(kp_m*4));
             sqrt((m*g)/(kp_m*4));
             -sqrt((m*g)/(kp_m*4))];

%% State space matrices
A11c = [0, (JJm*omg_0_vec(1) + JJm*omg_0_vec(2) + JJm*omg_0_vec(3) + JJm*omg_0_vec(4))/J(1,1), 0, 0;
       (-JJm*omg_0_vec(1) - JJm*omg_0_vec(2) - JJm*omg_0_vec(3) - JJm*omg_0_vec(4))/J(2,2), 0, 0, 0;
       1, 0, 0, 0;
       0, 1, 0, 0];


A12c = zeros(4,6);
A21c = [0 0 0 g;
        0 0 -g 0;
        zeros(4,4)];

A22c = [-kd/m  zeros(1,5);
        0   -kd/m zeros(1,4);
        0   0  -kd/m zeros(1,3);
        eye(3,3) zeros(3,3)];

B11c = [0, 2*kp_m*l*omg_0_vec(2)/J(1,1), 0, -2*kp_m*l*omg_0_vec(4)/J(1,1);
       -2*kp_m*l*omg_0_vec(1)/J(2,2), 0, 2*kp_m*l*omg_0_vec(3)/J(2,2), 0;
        0, 0, 0, 0;
        0, 0, 0, 0];
    
B12c = [-b*omg_0_vec(1)^2/J(1,1), 0, b*omg_0_vec(3)^2/J(1,1), 0;
         0, b*omg_0_vec(2)^2/J(2,2), 0, -b*omg_0_vec(4)^2/J(2,2);
         0, 0, 0, 0;
         0, 0, 0, 0];

B21c = [0, 0, 0, 0;
        0, 0, 0, 0;
        2*kp_m*omg_0_vec(1)/m, 2*kp_m*omg_0_vec(2)/m, 2*kp_m*omg_0_vec(3)/m, 2*kp_m*omg_0_vec(4)/m;
        0, 0, 0, 0;
        0, 0, 0, 0;
        0, 0, 0, 0];

B22c = [kp_m*omg_0_vec(1)^2/m, 0, -kp_m*omg_0_vec(3)^2/m, 0;
       0, kp_m*omg_0_vec(2)^2/m, 0, -kp_m*omg_0_vec(4)^2/m;
       0, 0, 0, 0;
       0, 0, 0, 0;
       0, 0, 0, 0;
       0, 0, 0, 0];
    
%% Discrete LQR matrices
Ts = 0.04; %sampling time
[Alqrd, Blqrd] = c2dm(A11c, B11c, eye(4), zeros(4, 4), Ts, 'zoh');    

%% LQR for attitude
qva = 30*[1,1];
qa = 30*[1,1];
Qlqr = diag([qva,qa]); 
rm = 5e-5;
Rlqr = diag([rm,rm,rm,rm]);

Klqr = dlqr(Alqrd, Blqrd, Qlqr, Rlqr);

%% MPC continuous linear state space matrices
% x = [p q phi theta u v w xE yE zE]'
% u = [omg1 omg2 omg3 omg4 alpha1 alpha2 alpha3 alpha4]'
Apc = [A11c A12c;
       A21c A22c]; %p for plant
Bpc = [B11c B12c;
       B21c B22c];

Cmpc = [zeros(3,7) eye(3)];

[Apd, Bpd] = c2dm(Apc, Bpc, Cmpc, zeros(3, 8), Ts, 'zoh');    

A11d = Apd(1:4,  1:4);
A12d = Apd(1:4,  5:10);
A21d = Apd(5:10, 1:4);
A22d = Apd(5:10, 5:10);

B11d = Bpd(1:4,  1:4);
B12d = Bpd(1:4,  5:8);
B21d = Bpd(5:10, 1:4);
B22d = Bpd(5:10, 5:8);

%prediction matrices
Ampc = [A11d-B11d*Klqr  A12d;
        A21d-B21d*Klqr  A22d];

Bmpc = [B12d   B11d*[1;-1;1;-1];
        B22d   B21d*[1;-1;1;-1]];
%Weighting matrices
Qmpc = [30 0 0;
        0 30 0;
        0 0 0.3];
Rmpc = [100 0 0 0 0;
        0 100 0 0 0;
        0 0 100 0 0;
        0 0 0 100 0;
        0 0 0 0 0.01];

N = 30;
M = 25;

%constraints
deltaumax = [999999 999999 999999 999999 999999]';
deltaumin = -deltaumax;

umax = [25*pi/180 25*pi/180 25*pi/180 25*pi/180 200]';
umin = -umax;
ymax = 1*[1.0 1.0 1.0]';
ymin = -ymax;

