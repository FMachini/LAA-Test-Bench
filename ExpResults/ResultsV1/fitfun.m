function J = fitfun(X)

global roll roll_rate Kx;

  
F = @(t, x) [x(2); X(2)/X(1)*x(2) - X(3)/X(1)*sin(x(1))];% - Kx * [x(1);x(2)];
T = 0:0.02:(length(roll)-1)*0.02;
S = [roll(1) roll_rate(1)];
[t, y] = ode45(F, T, S);

A=X(1)
B=X(2)
C=X(3)
%D=X(4)
J = sum((y(:,1)-roll).^2 )+ sum((y(:,2)-roll_rate).^2)


end

