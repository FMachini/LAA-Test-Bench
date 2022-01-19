function F = Forces(Y,input,param,t)

%num_inp = param.n+param.tilt1+param.tilt2;
omg = input(1:param.n);
alpha = zeros(1,6);
beta = zeros(1,6);
% alpha = input(param.n+1:param.tilt1+param.n);
% beta = input(param.tilt1+param.n+1:num_inp);

u=Y(1); v=Y(2); w=Y(3);phi=Y(7); theta=Y(8); psi=Y(9);
% Rigid body parameters
n = param.n; % number of eletric motors
m = param.m; % Multirrotor mass
kd = param.kd; % drag coefficient
k = param.k; %thrust coefficient
g = 9.81; % graviational acceleration
gamma = param.gamma;
% Thrust Force
T_x = 0;
T_y = 0;
T_z = 0;
T = 0;
for i = 1:n
    T_x = (sin(gamma(i))*sin(beta(i))+cos(gamma(i))*cos(beta(i))*sin(alpha(i)))*k*omg(i)^2 + T_x;
    T_y = (-cos(gamma(i))*sin(beta(i))+sin(gamma(i))*sin(alpha(i))*cos(beta(i)))*k*omg(i)^2 + T_y;
    T_z =  +cos(beta(i))*cos(alpha(i))*k*omg(i)^2 + T_z; 
    T = k*omg(i)^2+T;
end
Fx = sin(theta)*m*g - kd*u + T_x;
Fy = -sin(phi)*cos(theta)*m*g - kd*v + T_y;
Fz = -cos(phi)*cos(theta)*m*g - kd*w + T_z;


F = [Fx,Fy,Fz]';

end
