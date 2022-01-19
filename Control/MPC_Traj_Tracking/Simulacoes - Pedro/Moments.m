function [Mx, My, Mz] = Moments(Y, omg, alpha, param)
% Rigid body parameters
kp = param.kp_m; %thrust coefficient
l_b = param.l_b; %multirrotor arm length
gamma = param.gamma; % multirrotor arm angle wrt x axis
Jm = param.Jm; % propeller moment of inertia
b = param.b; %propeller drag coefficient

% State Variables
P=Y(4); Q=Y(5); R=Y(6);

% Torques
Mx = 0;
My = 0;
Mz = 0;

    for i = 1:4
        % due to thrust - MT

        MT_x = l_b*kp*sin(gamma(i))*cos(alpha(i))*omg(i)^2;  %  k
        MT_y = -l_b*kp*cos(gamma(i))*cos(alpha(i))*omg(i)^2;  % k  
        MT_z = 0;

        % gyroscopic effect

        Hx = 0;
        Hy = 0;
        Hz = 0;
        tau_x = ( Q*cos(alpha(i)) - R*sin(gamma(i))*sin(alpha(i)) )*Jm(i)*omg(i);  % k

        tau_y = (-P*cos(alpha(i)) + R*cos(gamma(i))*sin(alpha(i)) )*Jm(i)*omg(i);  % k

        tau_z = ( P*sin(gamma(i))*sin(alpha(i)) - Q*cos(gamma(i))*sin(alpha(i)) )*Jm(i)*omg(i);   % k

        MG_x = Hx + tau_x;
        MG_y = Hy + tau_y;
        MG_z = Hz + tau_z;

        % Fan Torques
        MF_x = -b*cos(gamma(i))*sin(alpha(i))*omg(i)^2;%*(-1)^i; % k
        MF_y =  b*sin(gamma(i))*sin(alpha(i))*omg(i)^2;%*(-1)^i; % k
        MF_z = -b*sign(omg(i))*cos(alpha(i))*omg(i)^2; % k

        Mx = MT_x + MG_x + MF_x + Mx;
        My = MT_y + MG_y + MF_y + My;
        Mz = MT_z + MG_z + MF_z + Mz;
    end

end