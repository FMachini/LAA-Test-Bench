function [Ad,Bd,Axd,Bxd,Cx] = LinearIdentifiedModel(Ts)
% First ordem model for Theta1
%Sampling time
%Ts = 0.02; 

% Theta 4 parameters
a1 = 0.0187;
b1 = 0.0495;
c1 = 0.6925;

% Theta 2 Parameters
b2 = 0.3338; %friction term
c2 = 1.0173*180/pi; % Motor effectiveness

% Theta 1 Parameters
%a3 = 0.1724;  % term associated with acceleration
%b3 = 0.4112; % friction term
b3 = 1.013;
%c3 = 2.542; % tilt deflection effectiveness
c3 = 7.313;
d3 = 7.756; % Theta 4 Theta 1 coupling term


L1 = 0.225;
L2  =0.225;

A11c =  [0 1; -c1/a1 -b1/a1]; 
B11c = [0 0; L1/a1 -L2/a1];

A12c = zeros(2,4);
B12c =  zeros(2,2);

A21c = zeros(4,2); A21c(2,1) = -d3;
B21c = zeros(4,2); B21c(4,1) = c2; B21c(4,2) = c2;

A22c = zeros(4,4); 
A22c(1,2) = 1; A22c(3,4) = 1;
A22c(4,4) = -b2; A22c(2,2) = -b3;

B22c = zeros(4,2); B22c(2,1) = c3; B22c(2,2) = -c3;

Ac = [A11c A12c;
      A21c A22c];
  
Bc = [B11c B12c;
      B21c B22c];
  
  
Cc  = zeros(4,6); Cc(1,3) = 1; Cc(2,4) = 1; Cc(3,5) = 1; Cc(4,6) = 1;

Dc = zeros(4,4);

[Ad, Bd] = c2dm(Ac, Bc, Cc, Dc, Ts, 'zoh');

%% Augmenting Matrix for state observer design
Cd = zeros(3,6);
Cd(1,1) = 1;Cd(2,3)=1; Cd(3,5)=1;

Ax = [Ac zeros(6,3);
zeros(3,6) eye(3)];
Bx = [Bc; zeros(3,4)];
Cx = [Cd eye(3)];

[Axd, Bxd] = c2dm(Ax, Bx, Cx, zeros(3,4), Ts, 'zoh');