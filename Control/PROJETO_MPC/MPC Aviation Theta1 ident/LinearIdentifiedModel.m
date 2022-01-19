function [Ad,Bd] = LinearIdentifiedModel(Ts)

%Sampling time
%Ts = 0.02; 

% Theta 4 parameters
a1 = 0.0187;
b1 = 0.0495;
c1 = 0.6925;

% Theta 2 Parameters
b2 = 0.3338; %friction term
c2 = 1.0173; % Motor effectiveness

% Theta 1 Parameters
% a3 = 0.2408;  % term associated with acceleration
% b3 = 0.9039; % friction term
% c3 = 3.638; % tilt deflection effectiveness
% d3 = 7.756; % Theta 4 Theta 1 coupling term

a3 = 2.215;  % term associated with acceleration
b3 = 1.438; % friction term
c3 = 6.584; % tilt deflection effectiveness
d3 = 7.756; % Theta 4 Theta 1 coupling term


L1 = 0.225;
L2  =0.225;

A11c =  [0 1; -c1/a1 -b1/a1]; 
B11c = [0 0; L1/a1 -L2/a1];

A12c = zeros(2,5);
B12c =  zeros(2,2);

A21c = zeros(5,2); A21c(5,1) = -d3*a3; 
B21c = zeros(5,2); B21c(4,1) = c2; B21c(4,2) = c2;

A22c = zeros(5,5); %A22c(2,2) = -1.043;
A22c(1,2) = 1; A22c(3,4) = 1; A22c(2,5) = 1;
A22c(4,4) = -b2; A22c(5,2) = -(b3-0.5*a3); A22c(5,5) = -a3;

B22c = zeros(5,2); B22c(5,1) = c3; B22c(5,2) = -c3;

Ac = [A11c A12c;
      A21c A22c];
  
Bc = [B11c B12c;
      B21c B22c];
  
  
Cc  = zeros(4,7); Cc(1,3) = 1; Cc(2,4) = 1; Cc(3,5) = 1; Cc(4,6) = 1;

Dc = zeros(4,4);

[Ad, Bd] = c2dm(Ac, Bc, Cc, Dc, Ts, 'zoh');