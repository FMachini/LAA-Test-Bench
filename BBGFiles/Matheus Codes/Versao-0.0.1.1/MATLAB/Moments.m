function Mom = Moments(param,input,Y,t)
% Rigid body parameters
n = param.n; % number of eletric motors
k = param.k; %thrust coefficient
l = param.l; %multirrotor arm length
gamma = param.gamma; % multirrotor arm angle wrt x axis
JJm = param.JJm; % propeller moment of inertia
b = param.b; %propeller drag coefficient
zcg = param.zcg; % Center of gravity z position BCS

%num_inp = n+param.tilt1+param.tilt2;

omg = input(1:n);
alpha = zeros(1,6);
beta = zeros(1,6);
% alpha = input(n+1:param.tilt1+n);
% beta = input(param.tilt1+n+1:num_inp);
beta_d=[0,0,0,0,0,0];
alpha_d=[0,0,0,0,0,0];
% State Variables
P=Y(4); Q=Y(5); R=Y(6);

% Torques

L = 0;
M = 0;
N = 0;

for i = 1:n
    % due to thrust
    tau_x = l(i)*sin(gamma(i))*cos(beta(i))*cos(alpha(i))*k*omg(i)^2 + zcg(i)*sin(beta(i))*k*omg(i)^2;
    tau_y = -l(i)*cos(gamma(i))*cos(beta(i))*cos(alpha(i))*k*omg(i)^2 + zcg(i)*cos(beta(i))*sin(alpha(i))*k*omg(i)^2;
    tau_z =-l(i)*cos(gamma(i))*sin(beta(i))*k*omg(i)^2 - l(i)*sin(gamma(i))*cos(beta(i))*sin(alpha(i))*k*omg(i)^2;
    
    % gyroscopic effect
    
    Gy_x =(alpha_d(i) + Q) * JJm(i)*omg(i);
    Gy_y = (-cos(alpha(i))*beta_d(i) + P)*JJm(i)*omg(i);
    Gy_z = 0;

    % Fan Torques
    Ft_x = (-cos(beta(i))*sin(alpha(i))*b*omg(i)^2)*(-1)^i;
    Ft_y = (sin(beta(i))*b*omg(i)^2)*(-1)^i;
    Ft_z = (-cos(beta(i))*cos(alpha(i))*b*omg(i)^2)*(-1)^i;
    
    L = tau_x + Gy_x + Ft_x + L;
    M = tau_y + Gy_y + Ft_y + M;
    N = tau_z + Gy_z + Ft_z + N;
end


Mom = [L,M,N]';

end
