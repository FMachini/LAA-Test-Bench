function [Ad,Bd] = tiltDynamics(Ts)

Act = [0 1;-28.228 -8.033];


Bct = [0;28.228];

Cct = eye(2,2);

Dct = [0;0];

[Ad, Bd] = c2dm(Act, Bct, Cct, Dct, Ts, 'zoh');

