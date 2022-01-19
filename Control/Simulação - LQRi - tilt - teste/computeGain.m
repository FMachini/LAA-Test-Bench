function [Kp,Kx] = computeGain()

a = 0.0187;
b = 0.0495;
c = 0.6925;
Iyy2 = 0.289;
m1 = 2.761;
m2 = 0.154;
m3 = 0.115;
m4 = 0.600; % bicopter mass
l3 = 0.173;
l2 = 0.595;
L1 = 0.215;
L2 = 0.225;
Iz1 = 0.2876;Iz2 = 0.001; Iz3 = 0; Iz4 = 0.0135;
g = 9.81;

den1 = l2^2*(m1+m3+m4)+Iz1+Iz2+Iz3+Iz4;
den2 = Iyy2 + 1/4 * l3^2* m2 + l2^2*(m2+m3+m4);
% Linear State Space Model
% x = [Theta1 Theta2 Theta4 Theta1_dot Theta2_dot Theta4_dot]
% expanded state vector (including error) z = [(x1ref-x1) (x2ref-x2) x_dot]
a1 = -l2*g*(m1+m2+m4)/den1;
Ac= [0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1;
     0 0 a1 0 0 0;
     0 0 0 0 0 0;
     0 0 -c/a 0 0 -b/a];

b1 = l2/den1;
b2 = l2/den2;

Bc= [0 0 0;
    0 0 0;
    0 0 0;
    0 b1 0;
    b2 0 0;
    0 0 1/a];

Cc = [1 0 0 0 0 0
      0 1 0 0 0 0
      0 0 1 0 0 0]; 
  
Aa = [zeros(3,3) Cc;
      zeros(6,3) Ac];

Bb = [zeros(3,3);Bc];
%  B= [0;
%      (L1+L2)/a]; % considerando apenas uma entrada: forca f aplicada + no motor 1 e - no motor 2
D = 0;
sysC = ss(Aa,Bb,eye(size(Aa)),D);

% Discrete time state space model
T = 0.02;%(s)
sysD = c2d(sysC,T);
A = sysD.A;
B = sysD.B;
C = sysD.C;
D = sysD.D;

% Designing LQR tracking Controler
qe = [1 5 0.1];
qa = [1 5 0.2];
qv = [1 0.5 0.1];
Qr = diag([qe qa qv]);
Rr = diag([5 1 10]) ;

Kc = dlqr(A,B,Qr,Rr);
Kp = Kc(:,1:3)
Kx = Kc(:, 4:end)

end
