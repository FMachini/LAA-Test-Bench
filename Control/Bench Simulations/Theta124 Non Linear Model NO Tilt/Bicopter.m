function [fBCS] = Bicopter(u)

% Compute bicopter forces and moments wrt body coordinate frame 
% Inputs: u = [f1 f2 alpha alpha2]- control input vector containing motors
% 1 and 2 forces and tilt deflections
% Output: fext = [taux tauy tauz Fx Fy Fz] - external moments (tau) and 
% forces wrt bicopter body coordinate frame.

l1 = 0.225;
l2 = 0.225;
f1trim = 1.7;
f2trim = 1.7;

f1 = u(1)+f1trim;
f2 = u(2)+f2trim;
%alpha1 = u(3);
%alpha2 = u(4);

Fz1 = f1;
Fz2 = f2;
Fz = Fz1+Fz2;
Fy = 0;%f1*sin(alpha1) - f2*sin(alpha2);

tau = Fz1*l1 - Fz2*l2;


fBCS = [0 0 tau -Fy -Fz 0]; %[Tx Ty Tz Fx Fy Fz] vector


end

