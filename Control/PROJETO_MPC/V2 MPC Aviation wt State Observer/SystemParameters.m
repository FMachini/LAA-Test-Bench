%% System Parameters
% Test Bench Properties
Ts = 0.02; %sampling time

[Ad,Bd,Axd,Bxd,Cxd] = LinearIdentifiedModel(Ts);
Alqrd = Ad(1:2,1:2); Blqrd = Bd(1:2,1:2); B11d = Blqrd;
A12d = Ad(1:2,3:end); B12d = Bd(1:2,3:end);
A21d = Ad(3:end,1:2); B21d = Bd(3:end,1:2);
A22d = Ad(3:end,3:end); B22d = Bd(3:end,3:end);

% LQR for attitude
qa = 1.6;
qva = 0.3;
Qlqr = diag([qa,qva]); 
rm = 0.2;
Rlqr = diag([rm,rm]);

%Klqr = dlqr(Alqrd, Blqrd, Qlqr, Rlqr);
Klqr= [ 0.423440636836316 0.532131772661905;
        -0.419537958155796 -0.527227332360873];
%damp(Alqrd-Blqrd*Klqr)

%prediction matrices
Ampc = [Alqrd + Blqrd*(-Klqr)  A12d;
        A21d + B21d*(-Klqr)  A22d];

Bmpc = [B12d   B11d*ones(2,1);
        B22d   B21d*ones(2,1)];
    
Cmpc = zeros(4,6);Cmpc(1,3)=1; Cmpc(2,4) = 1;Cmpc(3,5) = 1;Cmpc(4,6) = 1;
%Cmpc = zeros(2,6);Cmpc(1,3)=1; Cmpc(2,5) = 1;
Dmpc = zeros(4,4);

%Weighting matrices
Qmpc = [0.5 0 0 0;
        0 0.2 0 0;
        0 0 0.5 0;
        0 0 0 0.5];

Rmpc = [50 0 0
        0 50 0
        0 0 1];

N = 12;
M = 5;

%constraints
deltaumax = [0.1 0.1 0.4]';
deltaumin = -deltaumax;

umax = [10*[pi/180 pi/180] 0.4]';
umin = [0 0 -0.4]';
%umin = -umax;
ymax = [9999 9999 9999 9999]'*pi/180;
%ymax = 9999*[1 1]'*pi/180;
ymin = -ymax;
%caso interessante
% umin = [00*pi/180 0*pi/180 0*pi/180 0*pi/180 -9999]';
% ymax = 999*[1.1 1.1 1.1]';

