function J = fitfun3(X)

global roll_0 roll_dot_0;
global roll_2 roll_dot_2;
global roll_3 roll_dot_3;
global roll_4 roll_dot_4;

F = @(t, x) [x(2); -X(2)/X(1)*x(2) - X(3)/X(1)*sin(x(1))];
T = 0:0.02:(length(roll_0)-1)*0.02;
S = [roll_0(1) roll_dot_0(1)];
[t, y0] = ode45(F, T, S);

F = @(t, x) [x(2); -X(2)/X(1)*x(2) - X(3)/X(1)*sin(x(1))];
T = 0:0.02:(length(roll_2)-1)*0.02;
S = [roll_2(1) roll_dot_2(1)];
[t, y2] = ode45(F, T, S);

F = @(t, x) [x(2); -X(2)/X(1)*x(2) - X(3)/X(1)*sin(x(1))];
T = 0:0.02:(length(roll_3)-1)*0.02;
S = [roll_3(1) roll_dot_3(1)];
[t, y3] = ode45(F, T, S);

F = @(t, x) [x(2); -X(2)/X(1)*x(2) - X(3)/X(1)*sin(x(1))];
T = 0:0.02:(length(roll_4)-1)*0.02;
S = [roll_4(1) roll_dot_4(1)];
[t, y4] = ode45(F, T, S);

A=X(1)
B=X(2)
C=X(3)

J = sum((y0(:,1)-roll_0).^2 )+ sum((y0(:,2)-roll_dot_0).^2)+sum((y2(:,1)-roll_2).^2 )+ 2*sum((y2(:,2)-roll_dot_2).^2)+...
sum((y3(:,1)-roll_3).^2 )+ 2*sum((y3(:,2)-roll_dot_3).^2)+sum((y4(:,1)-roll_4).^2 )+ 2*sum((y4(:,2)-roll_dot_4).^2)


end

