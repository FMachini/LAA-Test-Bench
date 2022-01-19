function [ cost ] = equilibrium(x)

global m Jxx Jyy Jzz k l gamma b g theta phi
n=4;
omg = x(1:4);
alpha = x(5:8);
beta = zeros(1,4);
% State Variables

zcg = 0;
% Forces

% Thrust Force
T_x = 0;
T_y = 0;
T_z = 0;

for i = 1:n
    T_x = (sin(gamma(i))*sin(beta(i))+cos(gamma(i))*cos(beta(i))*sin(alpha(i)))*k*omg(i)^2 + T_x;
    T_y = (-cos(gamma(i))*sin(beta(i))+sin(gamma(i))*sin(alpha(i))*cos(beta(i)))*k*omg(i)^2 + T_y;
    T_z =  +cos(beta(i))*cos(alpha(i))*k*omg(i)^2 + T_z; 

end

Fx = sin(theta)*m*g  + T_x;
Fy = -sin(phi)*cos(theta)*m*g + T_y;
Fz = -cos(phi)*cos(theta)*m*g + T_z;

% Torques

L = 0;
M = 0;
N = 0;

for i = 1:n
    % due to thrust
    tau_x = l*sin(gamma(i))*cos(beta(i))*cos(alpha(i))*k*omg(i)^2 + zcg*sin(beta(i))*k*omg(i)^2;
    tau_y = -l*cos(gamma(i))*cos(beta(i))*cos(alpha(i))*k*omg(i)^2 + zcg*cos(beta(i))*sin(alpha(i))*k*omg(i)^2;
    tau_z =-l*cos(gamma(i))*sin(beta(i))*k*omg(i)^2 - l*sin(gamma(i))*cos(beta(i))*sin(alpha(i))*k*omg(i)^2;

    % Fan Torques
    Ft_x = -(sin(gamma(i))*sin(beta(i))+cos(gamma(i))*cos(beta(i))*sin(alpha(i))*b*omg(i)^2)*(-1)^i;
    Ft_y = -(cos(gamma(i))*sin(beta(i))+sin(gamma(i))*cos(beta(i)*sin(alpha(i)))*b*omg(i)^2)*(-1)^i;
    Ft_z = (-cos(beta(i))*cos(alpha(i))*b*omg(i)^2)*(-1)^i;
    
    L = tau_x + Ft_x + L;
    M = tau_y + Ft_y + M;
    N = tau_z + Ft_z + N;
end


eval = [Fx/m;Fy/m;Fz/m;L/Jxx;M/Jyy;N/Jzz];

cost = norm(eval);

end

