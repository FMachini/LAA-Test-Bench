%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%% Trim Condition Evaluation %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% Author: Felipe Machini %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc;
global m Jxx Jyy Jzz k l gamma b g theta phi
%% System Parameters
m = 0.468; % Multirrotor mass
Jintert = [0.049 0      0;
     0     0.049  0;
     0     0      0.088];%inertia matrix
Jxx = Jintert(1,1);
Jyy = Jintert(1,1);
Jzz = Jintert(1,1);

kd =4.8e-3; % drag coefficient
k = 2.9e-5; %thrust coefficient
l = 0.225; %multirrotor arm length
gamma = [0  90  180  270]*pi/180; % multirrotor arm angle wrt x axis
JJm =3.357e-5; % propeller moment of inertia
Jm1 = JJm;
Jm2 = JJm;
Jm3 = JJm;
Jm4 = JJm;
b  = 1.1e-6; %propeller drag coefficient

g = 9.81;

%omg0_verdaeiro = sqrt((mass*grav)/(kp_m*4));%equilibium rotation

% Valores de equilíbrio
theta = 30*pi/180;
phi = 0*pi/180;
psi0 = 0;


%% Minimization


% Bounds 

% optimization boundary
ub = [500;
      0*500;
      500;
      0*500;
      90*pi/180;
      90*pi/180;
      90*pi/180;
      90*pi/180];
lb = [0*-500;
      -500;
      0*-500;
      -500;
      -90*pi/180;
      -90*pi/180;
      -90*pi/180;
      -90*pi/180];
% 
%% 
% There are no linear constraints, so set those arguments to |[]|. 
A = [];
bb = [];
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


[y,fval] = ga(@equilibrium,8,[],[],[],[],lb,ub)