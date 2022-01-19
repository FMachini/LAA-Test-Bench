%% System Parameters
m = 0.468; % Multirrotor mass
J = [0.049 0      0;
     0     0.049  0;
     0     0      0.088];%inertia matrix
kd =4.8e-3; % drag coefficient
k = 2.9e-5; %thrust coefficient
l = 0.225; %multirrotor arm length
gamma = [0  90  180  270]*pi/180; % multirrotor arm angle wrt x axis
JJm =3.357e-5; % propeller moment of inertia
b = 1.1e-6; %propeller drag coefficient
zcg = 0; % Center of gravity z position BCS
g = 9.81;

omg0 = sqrt((m*g)/(k*4));%equilibium rotation
%% LQR continuous linear state space matrices
A11c = [0 0 0 0;
        0 0 0 0;
        1 0 0 0;
        0 1 0 0];

b1 = 2*l*omg0*k/J(1,1);
b2 = 2*l*omg0*k/J(2,2);
B11c = [0   b1 0  -b1;
       -b2  0  b2  0;
        0   0  0   0;
        0   0  0   0];

%% Discrete LQR matrices
Ts = 0.04; %sampling time
[Alqrd, Blqrd] = c2dm(A11c, B11c, eye(4), zeros(4, 4), Ts, 'zoh');    

%% LQR for attitude
qva = 3e1*[1,1];
qa = 3e1*[1,1];
Qlqr = diag([qva,qa]); 
rm = 5e-5;
Rlqr = diag([rm,rm,rm,rm]);

Klqr = dlqr(Alqrd, Blqrd, Qlqr, Rlqr);

%% MPC continuous linear state space matrices
A12c = zeros(4,6);
A21c = [0 0 0 g;
        0 0 -g 0;
        zeros(4,4)];

a1 = -kd/m;
A22c = [a1  zeros(1,5);
        0   a1 zeros(1,4);
        0   0  a1 zeros(1,3);
        eye(3,3) zeros(3,3)];

b3 = b*omg0^2/J(1,1);
b4 = b*omg0^2/J(2,2);
b5 = k*omg0^2/m;
b6 = 2*k*omg0/m;

B12c = [-b3  0   b3 0;
        0 -b4  0  b4;
        zeros(2,4) ];

B21c = [zeros(2,4);
        b6 b6 b6 b6;
        zeros(3,4)];

B22c = [b5  0  -b5 0;
        0  b5  0  -b5;
       zeros(4,4)];
   
Apc = [A11c A12c; A21c A22c]; %p for plant
Bpc = [B11c B12c; B21c B22c ];

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
Ampc = [A11d + B11d*(-Klqr)  A12d;
        A21d + B21d*(-Klqr)  A22d];

Bmpc = [B12d   B11d*ones(4,1);
        B22d   B21d*ones(4,1)];
%Weighting matrices
Qmpc = [1 0 0;
        0 1 0;
        0 0 0.01]*30;
Rmpc = [100 0 0 0 0;
        0 100 0 0 0;
        0 0 100 0 0;
        0 0 0 100 0;
        0 0 0 0 0.01];

N = 30;
M = 25;

%constraints
deltaumax = [9999 9999 9999 9999 9999]';
deltaumin = -deltaumax;

umax = 25e6*[pi/180 pi/180 pi/180 pi/180 9999]';
umin = -umax;
ymax = 1e6*[1 1 1]';
ymin = -ymax;
%caso interessante
% umin = [00*pi/180 0*pi/180 0*pi/180 0*pi/180 -9999]';
% ymax = 999*[1.1 1.1 1.1]';

