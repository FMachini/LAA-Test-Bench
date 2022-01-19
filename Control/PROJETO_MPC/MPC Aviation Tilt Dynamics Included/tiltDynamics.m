function [Ad,Bd] = tiltDynamics(Ts)

Ac = [0 1;-28.228 -8.033];


Bc = [0;28.228];

Cc = eye(2,2);

Dc = [0;0];

[Ad, Bd] = c2dm(Ac, Bc, Cc, Dc, Ts, 'zoh');

