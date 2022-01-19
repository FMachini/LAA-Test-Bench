%% System Parameters
mass = 0.468; % Multirrotor mass
Jintert = [0.049 0      0;
     0     0.049  0;
     0     0      0.088];%inertia matrix
Jxx = Jintert(1,1);
Jyy = Jintert(1,1);
Jzz = Jintert(1,1);

kd =4.8e-3; % drag coefficient
kp_m = 2.9e-5; %thrust coefficient
l_b = 0.225; %multirrotor arm length
gamma = [0  90  180  270]*pi/180; % multirrotor arm angle wrt x axis
JJm =3.357e-5; % propeller moment of inertia
Jm1 = JJm;
Jm2 = JJm;
Jm3 = JJm;
Jm4 = JJm;
b_drag = 1.1e-6; %propeller drag coefficient

grav = 9.81;

omg0_verdaeiro = sqrt((mass*grav)/(kp_m*4));%equilibium rotation

% Valores de equilíbrio
theta0 = 30*pi/180;
phi0 = 0*pi/180;
psi0 = 0;