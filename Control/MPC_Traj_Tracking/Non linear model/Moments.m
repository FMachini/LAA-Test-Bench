function Mom = Moments(param,omg,alpha,beta,Y,t)
% Rigid body parameters
n = param.n; % number of eletric motors
k = param.k; %thrust coefficient
l = param.l; %multirrotor arm length
gamma = param.gamma; % multirrotor arm angle wrt x axis
JJm = param.JJm; % propeller moment of inertia
b = param.b; %propeller drag coefficient
zcg = param.zcg; % Center of gravity z position BCS

% alpha = input(n+1:param.tilt1+n);
% beta = input(param.tilt1+n+1:num_inp);
beta_d=zeros(1,n);
alpha_d=zeros(1,n);
omg_d = zeros(1,n);
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
    
    Hx = omg(i)*JJm(i)*(sin(gamma(i))*cos(beta(i))*beta_d(i)-cos(gamma(i))*cos(beta(i))*cos(alpha(i))*alpha_d(i)...
        +cos(gamma(i))*sin(beta(i))*sin(alpha(i))*beta_d(i)) + omg_d(i)*JJm(i)*(sin(gamma(i))*sin(beta(i))-cos(gamma(i))*cos(beta(i))*sin(alpha(i)));
    
    Hy = omg(i)*JJm(i)*(-cos(gamma(i))*cos(beta(i))*beta_d(i)-sin(gamma(i))*cos(beta(i))*cos(alpha(i))*alpha_d(i)...
        +sin(gamma(i))*sin(beta(i))*sin(alpha(i))*beta_d(i)) + omg_d(i)*JJm(i)*(-cos(gamma(i))*sin(beta(i))-sin(gamma(i))*cos(beta(i))*sin(alpha(i)));
    
    Hz = omg(i)*JJm(i)*(-cos(beta(i))*sin(alpha(i))*alpha_d(i)-sin(beta(i))*cos(alpha(i))*beta_d(i)) + omg_d(i)*JJm(i)*(cos(alpha(i)*cos(beta(i))));
    
    
    Tx = ((alpha_d(i)+Q)*cos(alpha(i))*cos(beta(i)) - (R-sin(alpha(i))*beta_d(i))*(-cos(gamma(i))*sin(beta(i))+sin(gamma(i))*sin(alpha(i))*cos(beta(i))))*omg(i)*JJm(i);
    
    Ty = ((-cos(alpha(i))*beta_d(i)-P)*cos(alpha(i))*cos(beta(i)) + (R-sin(alpha(i))*beta_d(i))*(sin(gamma(i))*sin(beta(i))+cos(gamma(i))*sin(alpha(i))*cos(beta(i))))*omg(i)*JJm(i);
    
    Tz = ((cos(alpha(i))*beta_d(i)+P)*(-cos(gamma(i))*sin(beta(i))+sin(gamma(i))*sin(alpha(i))*cos(beta(i))) - (alpha_d(i)+Q)*(sin(alpha(i))*sin(beta(i))+...
        cos(gamma(i))*sin(alpha(i))*cos(beta(i))))*omg(i)*JJm(i);
    
    Gy_x = Hx+Tx;
    Gy_y = Hy+Ty;
    Gy_z = Hz+Tz;

    % Fan Torques
    Ft_x = -(sin(gamma(i))*sin(beta(i))+cos(gamma(i))*cos(beta(i))*sin(alpha(i))*b*omg(i)^2)*(-1)^i;
    Ft_y = -(-cos(gamma(i))*sin(beta(i))+sin(gamma(i))*cos(beta(i)*sin(alpha(i)))*b*omg(i)^2)*(-1)^i;
    Ft_z = (-cos(beta(i))*cos(alpha(i))*b*omg(i)^2)*(-1)^i;
    
    L = tau_x + Gy_x + Ft_x + L;
    M = tau_y + Gy_y + Ft_y + M;
    N = tau_z + Gy_z + Ft_z + N;
end


Mom = [L,M,N]';

end