%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Desing Altitude LQT Controler 
%%%% State Variables: Theta2 Theta2_dot 
%%%% Input: F1 F2 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Defining model
% Theta 2 Parameters
b2 = 0.22; %friction term
%b2 = 5.20;
c2 = 0.8918; % Motor effectiveness

Aalt = [0 1;0 -b2];

Balt = [0 0;
        c2 c2];
Calt = [1 0];

Aa = [zeros(1,1) Calt;
      zeros(2,1) Aalt];

Bb = [zeros(1,2);Balt];

% Discretizing Model
[Aaltd, Baltd] = c2dm(Aa, Bb, eye(size(Aa)), zeros(size(Bb)), Ts, 'zoh');

% Define Gain

Qr = diag([1,1.2,0.1]);
Rr = diag([0.7 0.7]) ;

Kc = dlqr(Aaltd,Baltd,Qr,Rr);
Kp = Kc(:,1);
Kx = Kc(:, 2:end);

damp(Aaltd - Baltd*Kc)