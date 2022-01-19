%{
   Funcao para simular a dinamica nao linear de um quadricoptero com
   mecanismo de vetorizacao do empuxo

   Autores: Pedro Assis, Felipe Machini, Rubens Junqueira

   Data: 07/05/20

   Entradas:
   param - dicionario com parametros do sistema
   alpha, omg - tilt dos rotores e velocidade de rotação
   t, Y - tempo e estados (internos da funcao ode45)

   Saidas: 
   Y - estados no proximo periodo de amostragem
   

%}

function [dY]  =  SystemDynamics(t, Y, param, omg, alpha)
%% Function for estimating the states of a 6 dof rigid body for given forces and moments for each time step

%% Carregando parametros
m = param.m; % Multirrotor mass
J = param.J; % Multirrotor inertia matrix
Jxx = J(1,1);
Jyy = J(2,2);
Jzz = J(3,3);

% x_simulacao = [u v w P Q R phi theta psi X Y Z]
%% Definindo variaveis de estado
dY = zeros(9,1);
u = Y(1); 
v = Y(2); 
w = Y(3); 
P = Y(4); 
Q = Y(5);
R = 0*Y(6);
phi = Y(7); 
theta = Y(8);
psi = 0*Y(9);

%% Calculando forcas extenas
% Forces applied
[Fx, Fy, Fz] = Forces(Y, omg, alpha, param);

% Linear Momentum
dY(1) = Fx/m - Q*w + R*v;  % k
dY(2) = Fy/m + P*w - R*u;  % k
dY(3) = Fz/m - P*v + Q*u;  % k

%% Calculando momentos externos
[Mx, My, Mz] = Moments(Y, omg, alpha, param);  % k

% Angular Momentum
dY(4) = (Mx - Q*R*(Jzz - Jyy))/Jxx;  % k
dY(5) = (My - P*R*(Jxx - Jzz))/Jyy;  % k
dY(6) = (Mz - P*Q*(Jyy - Jxx))/Jzz;  % %

%% Calculando deriv dos angulos de Euler
Mat = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
       0, cos(phi),  -sin(phi);
       0, sin(phi)*sec(theta), cos(phi)*sec(theta)];  % k
   
euler = Mat*[P; Q; R];  % k
dY(7) = euler(1);  % k
dY(8) = euler(2);  % k
dY(9) = euler(3);  % k

%% Din de deslocamento em relacao a ref inercial

L1 = [cos(psi),   -sin(psi),  0;
      sin(psi),   cos(psi),   0;
       0,         0,          1];  % k
L2 = [cos(theta),   0,   sin(theta);
      0,            1,   0;
     -sin(theta),   0,   cos(theta)];  % k
L3 = [1,   0,         0;
      0,   cos(phi), -sin(phi);
      0,   sin(phi),  cos(phi)];  % k

LEB = L1*L2*L3;
V = LEB*[u;v;w]; % BCS para ICS

dY(10) = V(1);  % k x
dY(11) = V(2);  % k y
dY(12) = V(3);  % k z

end