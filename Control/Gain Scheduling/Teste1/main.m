% x = [omg1 omg2 omg3 omg4 alpha1 alpha2 alpha3 alpha 4]
clc



%% Função de custo
fun = @(x)0.1*(x(1)^2 + x(2)^2 + x(3)^2 + x(4)^2) + 1*(x(5)^2 + x(6)^2 + x(7)^2 + x(8)^2);  

%% Limitantes para as variáveis de otimização
ub = [5000;
      5000;
      5000;
      5000;
      90*pi/180;
      90*pi/180;
      90*pi/180;
      90*pi/180];
lb = [-5000;
      -5000;
      -5000;
      -5000;
      -90*pi/180;
      -90*pi/180;
      -90*pi/180;
      -90*pi/180];
% 
%% 
% There are no linear constraints, so set those arguments to |[]|. 
A = [];
b = [];
Aeq = [];
beq = [];  

%% 
% Choose an initial point satisfying all the constraints. 
x0 = [200;
      200;
      200;
      200;
      0;
      0;
      0;
      0];  

  
% options = optimoptions('fmincon','Algorithm','interior-point','MaxIterations',10000,'MaxFunctionEvaluations',10000, 'FiniteDifferenceType', 'central', 'SubproblemAlgorithm', 'cg','HessianApproximation', 'finite-difference')
options = optimoptions('fmincon','Algorithm','interior-point','MaxIterations',10000,'MaxFunctionEvaluations',10000)
%% 
% Solve the problem. 
nonlcon = @minharest;
[x,fval,exitflag,output,lambda,grad,hessian] = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon, options)
SystemParameters;
omg1_0 = x(1);
omg2_0 = x(2);
omg3_0 = x(3);
omg4_0 = x(4);
alpha1_0 = x(5);
alpha2_0 = x(6);
alpha3_0 = x(7);
alpha4_0 = x(8);
x = [x(1:4);x(5:end).*180/pi]

