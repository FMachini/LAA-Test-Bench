function [dY]  = Hexa(Y,param,omega,t)
global input_saida;
global referencia;
global time;
time(end+1)=t;
%% Reference signal
ref = zeros(3,1);
ref(1) = 1;
ref(2) = 1;
ref(3) = 1;
referencia(end+1,:) = ref';
% Rigid body parameters
m = param.m; % Multirrotor mass
J = param.J; % Multirrotor inertia matrix
Ixx=J(1,1); Ixy=J(1,2); Ixz=J(1,3);
Iyx=J(2,1); Iyy=J(2,2); Iyz=J(2,3);
Izx=J(3,1); Izy=J(3,2); Izz=J(3,3);

% State Variables
dY=zeros(12,1);
u=Y(1); v=Y(2); w=Y(3); P=Y(4); Q=Y(5); R=Y(6);
phi=Y(7); theta=Y(8); psi=Y(9);

%% Input calculation (Control Law)
input = omega;
input_saida(end+1,:)=input;
%% Dynamic equations for a 6dof rigid body in BCS
% Forces applied

F = Forces(Y,input,param,t);

Fx = F(1);
Fy = F(2);
Fz = F(3);

% Linear Momentum
dY(1) = Fx/m - Q*w + R*v;
dY(2) = Fy/m + P*w - R*u;
dY(3) = Fz/m - P*v + Q*u;

% Moments Calculation
Mom = Moments(param,input,Y,t);
L = Mom(1);
M = Mom(2);
N = Mom(3);

% Angular Momentum
dY(4) = (-Q*(-Izx*P-Izy*Q+Izz*R) + R*(-Iyx*P+Iyy*Q-Iyz*R)+ L)/Ixx;
dY(5) = (P*(-Izx*P-Izy*Q+Izz*R) - R*(Ixx*P-Ixy*Q-Ixz*R) + M)/Iyy;
dY(6) = (-P*(-Ixy*P+Iyy*Q-Iyz*R) + Q*(Ixx*P-Ixy*Q-Ixz*R) + N)/Izz;


Mat = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
       0  cos(phi)  -sin(phi);
       0 sin(phi)*sec(theta) cos(phi)*sec(theta)];
euler = Mat*[P;Q;R];
dY(7) = euler(1);
dY(8) = euler(2);
dY(9) = euler(3);


L1 = [cos(psi),sin(psi),0;
    -sin(psi),cos(psi),0;
    0,0,1];
L2 = [cos(theta),0,-sin(theta);
    0,1,0;
    sin(theta),0,cos(theta)];
L3 = [1,0,0;
    0,cos(phi),sin(phi);
    0,-sin(phi),cos(phi)];

LEB = L3*L2*L1;
V = LEB*[u;v;w];

dY(10) = V(1);
dY(11) = V(2);
dY(12) = V(3);  

end
