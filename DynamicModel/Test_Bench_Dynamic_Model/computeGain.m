function [Kp,Kx] = computeGain(Ts)
% Functions destinated to the desing of a LQR controller
% Input - Ts: Sampling time for discrete model
% Output - Kp: integral gain, Kx: regulator gain

%% System Parameters
% Test Bench Properties
Iz1 = 0.2876;Iz2 = 0.001; Iz3 = 0; Iz4 = 0.0135;


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

m_counter_weight = 1.7;
l4 = 0.33;
den2 = Iyy2 + 1/4 * l3^2* m2 + l2^2*(m2+m3+m4) + l4^2*m_counter_weight;
den1 = l2^2*(m2+m3+m4)+Iz1+Iz2+Iz3+Iz4 + l4^2*m_counter_weight;
a1 = -3.4*l2/den1; %based on the trimming motor thrust
% Linear State Space Model
% x = [Theta1 Theta2 Theta4 Theta1_dot Theta2_dot Theta4_dot]
% expanded state vector (including error) z = [(x1ref-x1) (x2ref-x2) x_dot]

% Theta 2 Parameters
b2 = 0.3338; %friction term
c2 = 1.0173;%*180/pi; % Motor effectiveness

% Theta 1 Parameters
%a3 = 0.1724;  % term associated with acceleration
%b3 = 0.4112; % friction term
b3 = 1.013;
%c3 = 2.542; % tilt deflection effectiveness
c3 = 6.313;
d3 = 7.756; % Theta 4 Theta 1 coupling term


Ac= [0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1;
     0 0 -d3 0 0 0;
     0 0 0 0 0 0;
     0 0 -c/a 0 0 -b/a];

b1 = l2/den1;
b2 = l2/den2;

% Bc= [0 0 0 0;
%      0 0 0 0;
%      0 0 0 0;
%      0 0 l2*1.7/den1 -l2*1.7/den1;
%      l2/den2 l2/den2 0 0;
%     L1/a -L2/a 0 0];

Bc= [0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 c3 -c3;
     b3 b3 0 0;
    L1/a -L2/a 0 0];

Cc = [1 0 0 0 0 0
      0 1 0 0 0 0
      0 0 1 0 0 0]; 
  
Aa = [zeros(3,3) Cc;
      zeros(6,3) Ac];

Bb = [zeros(3,4);Bc];

D = 0;
sysC = ss(Aa,Bb,eye(size(Aa)),D);

% Discrete time state space model

sysD = c2d(sysC,Ts);
A = sysD.A;
B = sysD.B;
C = sysD.C;
D = sysD.D;

% Designing LQR tracking Controler
qe = [12 3 0.01];
qa = [8 1 2];
qv = [10 2 2];
Qr = diag([qe qa qv]);
Rr = diag([0.1 0.1 1 1]) ;

Kc = dlqr(A,B,Qr,Rr);
Kp = Kc(:,1:3)
Kx = Kc(:, 4:end)

end
