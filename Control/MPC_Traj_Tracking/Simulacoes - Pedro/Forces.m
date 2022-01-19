function [Fx, Fy, Fz] = Forces(Y, omg, alpha, param)
% x_simulacao = [u v w P Q R phi theta psi X Y Z]
u = Y(1);
v = Y(2);
w = Y(3);
phi = Y(7);
theta = Y(8);
% psi=Y(9);

% Rigid body parameters
m = param.m; % Multirrotor mass
kd = param.kd; % drag coefficient
kp = param.kp_m; %thrust coefficient
g = param.g; % graviational acceleration

% Thrust Force
FT_z = 0;
for i = 1:4
    FT_z = kp*cos(alpha(i))*omg(i)^2 + FT_z;  % k
end
FT_x = kp*(sin(alpha(1))*omg(1)^2 - sin(alpha(3))*omg(3)^2);  % k
FT_y = kp*(sin(alpha(2))*omg(2)^2 - sin(alpha(4))*omg(4)^2);  % k

% Total (grav+drag+thrust)
Fx = m*g*sin(theta) - kd*u + FT_x;  % k
Fy = -m*g*sin(phi)*cos(theta) - kd*v + FT_y;  % k
Fz = -m*g*cos(phi)*cos(theta) - kd*w + FT_z;  % k

end
