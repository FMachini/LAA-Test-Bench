clear all;close all;clc;

global roll roll_rate Kx;
% Kx = [-1.3855,0.0677
%       1.3239,-0.0647];

% A = textread('V_1_7_2020/V_n20.txt');
A = textread('C:\Users\-F.Machini\Desktop\bancada\Results v1\Identification_motor_on3.txt');
%A = A(34:end,:);
roll = sgolayfilt (A(:,1), 3, 13);
roll_rate = sgolayfilt (A(:,2), 3, 13);
% roll = (A(155:end,1)-A(end,1))*pi/180;
% roll_rate = A(155:end,2)*pi/180;
roll = (roll(82:170,1)-roll(170,1))*pi/180;
roll_rate = roll_rate(82:170)*pi/180;

out = ga(@fitfun,3,[],[],[],[],[0.001 -1 -10],[0.020 1 10])

 F = @(t, x) [x(2); out(2)/out(1)*x(2) - out(3)/out(1)*sin(x(1))] ;
%F = @(t, x) [x(2); -0.38/0.001*x(2) - 0.0399/0.001*sin(x(1))];
T = 0:0.02:(length(roll)-1)*0.02;
S = [roll(1) roll_rate(1)];
[t, yfit] = ode45(F, T, S);

Jfinal = sum((yfit(:,1)-roll).^2 )+ sum((yfit(:,2)-roll_rate).^2);
figure
subplot(2,1,1)
plot(0.02*[1:length(roll)],roll,'ro',0.02*[1:length(roll)],yfit(:,1))

subplot(2,1,2)
plot(0.02*[1:length(roll_rate)],roll_rate,'ro',0.02*[1:length(roll)],yfit(:,2))

%Q = cumtrapz(A(116:322,2)*pi/180)