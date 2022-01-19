% Creating motor and tilt actuators transfer functions in Laplace Domain

%% Motor Dynamics (in force / out force in N)
tfm1 = tf(9.072,[1 9.072]);
tfm2 = tf(8.8995,[1 8.8995]);

% State Space Model
% For motor1
Am1 = -9.072;
Bm1 = 9.072;
Cm1 = 1;
Dm1 = 0;
motor1SYS = ss(Am1,Bm1,Cm1,Dm1);

% For motor2
Am2 = -8.8995;
Bm2 = 8.8995;
Cm2 = 1;
Dm2 = 0;
motor2SYS = ss(Am2,Bm2,Cm2,Dm2);

% Tilt Dynamics ( in deg / out deg)
tfa1 = tf(28.697, [1 8.1457 28.697]);
tfa2 = tf(28.228, [1 8.033 28.228]);

% State Space Model

% For tilt Mechanism 1
At1 = [0 1 
      -28.697 -8.1457];
Bt1 = [0;
      28.697];
Ct1 = [1 0];
Dt1 = zeros(2,1);
alpha1SYS = ss(At1,Bt1,Ct1,[]);


% For tilt Mechanism 2
At2 = [0 1; 
      -28.228 -8.033];
Bt2 = [0; 
       28.228];
Ct2 = [1 0];

alpha2SYS = ss(At2,Bt2,Ct2,[]);