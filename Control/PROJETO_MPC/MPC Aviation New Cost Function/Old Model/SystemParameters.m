%% System Parameters
% Test Bench Properties
Ts = 0.02; %sampling time
Iz1 = 0.2876;Iz2 = 0.001; Iz3 = 0; Iz4 = 0.0135;
g = 9.81;

% Test Bench Properties
a = 0.0187;
b = 0.0495;
c = 0.6925;
Iyy2 = 0.289;
m2 = 0.154;
m3 = 0.115;
m4 = 0.600; % bicopter mass
l3 = 0.173;
l2 = 0.605;
L1 = 0.225;
L2 = 0.225;
Bar_mom_inertia = 1/12*((1.47*1.3)*0.85)*1.3^2; %Estimated from material properties
m_counter_weight = 1.7;
l4 = 0.33;
den2 = Iyy2 + 1/4 * l3^2* m2 + l2^2*(m2+m3+m4) + l4^2*m_counter_weight;
den1 = l2^2*(m2+m3+m4)+Iz1+Iz2+Iz3+Iz4 + l4^2*m_counter_weight;
trimForce = 1.7; % equilibrium trimming force

% Linear State Space Model
% x = [Theta1 Theta2 Theta4 Theta1_dot Theta2_dot Theta4_dot]
% expanded state vector (including error) z = [(x1ref-x1) (x2ref-x2) x_dot]
%a1 = (l4*g*m_counter_weight-l2*g*(m2+m3+m4))/den1;
%a1 = -3.4*l2/den1; %based on the trimming motor thrust
%a1 = -3.4*l2/25.7125;
a1 = -0.14;

%% Defining Matrices
A11c = [0 1; -c/a -b/a]; 
B11c = [0 0; L1/a -L2/a];
%% Discrete LQR matrices

[Alqrd, Blqrd] = c2dm(A11c, B11c, eye(2), zeros(2, 2), Ts, 'zoh');    

%% LQR for attitude
qa = 1.6;
qva = 0.3;
Qlqr = diag([qa,qva]); 
rm = 0.2;
Rlqr = diag([rm,rm]);

Klqr = dlqr(Alqrd, Blqrd, Qlqr, Rlqr);
damp(Alqrd-Blqrd*Klqr)
trimForce = 3.4; % equilibrium trimming force
%% MPC continuous linear state space matrices
A12c = zeros(2,4);
A21c = zeros(4,2);
A21c(2,1) =  a1; %A21c(2,4) = -1.58/den1;

A22c = zeros(4,4);A22c(2,2) = -0.35;
A22c(1,2) = 1; A22c(3,4) = 1;


B12c = zeros(2,2);% B12c(2,1) = -0.1303*L2*trimForce/a; B12c(2,2) = 0.1303*L2*trimForce/a;

B21c = [zeros(3,2);
        l2/den2 l2/den2];

B22c = zeros(4,2); %B22c(2,1) = l2*1.7/den1; B22c(2,2) = -l2*1.7/den1;
B22c(2,1) = 0.058; B22c(2,2) = -0.058;
%B22c(4,1) = -0.1303*l2*trimForce/den2; B22c(4,2) = -0.1303*l2*trimForce/den2;
   
Apc = [A11c A12c; A21c A22c]; %p for plant
Bpc = [B11c B12c; B21c B22c ];

Cmpc = zeros(4,6);Cmpc(1,3)=1; Cmpc(2,4) = 1;Cmpc(3,5) = 1;Cmpc(4,6) = 1;
%Cmpc = zeros(2,6);Cmpc(1,3)=1; Cmpc(2,5) = 1;
Dmpc = zeros(4,4);

[Apd, Bpd] = c2dm(Apc, Bpc, Cmpc, Dmpc, Ts, 'zoh');    

A11d = Apd(1:2,  1:2);
A12d = Apd(1:2,  3:end);
A21d = Apd(3:end, 1:2);
A22d = Apd(3:end, 3:end);

B11d = Bpd(1:2,  1:2);
B12d = Bpd(1:2,  3:end);
B21d = Bpd(3:end, 1:2);
B22d = Bpd(3:end, 3:end);

%prediction matrices
Ampc = [Alqrd + Blqrd*(-Klqr)  A12d;
        A21d + B21d*(-Klqr)  A22d];

Bmpc = [B12d   B11d*ones(2,1);
        B22d   B21d*ones(2,1)];
%Weighting matrices
Qmpc = [10 0 0 0;
        0 8 0 0;
        0 0 0.1 0;
        0 0 0 0.1];

% Qmpc = [2 0;
%         0 0.1];

Rmpc = [1 0 0
        0 1 0
        0 0 5];

N = 12;
M = 6;

%constraints
deltaumax = [10 10 9999]';
deltaumin = -deltaumax;

umax = [20*[pi/180 pi/180] 0.4]';
umin = [0 0 -0.4]';
%umin = -umax;
ymax = 9999*[1 1 1 1]'*pi/180;
%ymax = 9999*[1 1]'*pi/180;
ymin = -ymax;
%caso interessante
% umin = [00*pi/180 0*pi/180 0*pi/180 0*pi/180 -9999]';
% ymax = 999*[1.1 1.1 1.1]';

