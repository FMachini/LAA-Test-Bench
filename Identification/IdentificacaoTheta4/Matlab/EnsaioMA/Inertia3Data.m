%   Universidade Federal de Uberlândia
%   Faculdade de Engenharia Mecânica
%   LAA-Laboratorio de Aeronave Autonomas 
%   
%   Identificação da Inercia Ix_4 da bancada
%   Professor: Felipe Machini   
%   Aluno: Gabriel Costa e Silva
% =================================

clear all;
close all;
clc;

global roll_0 roll_dot_0;
global roll_1 roll_dot_1;
global roll_2 roll_dot_2;
global roll_3 roll_dot_3;
global roll_4 roll_dot_4;

rad2deg=180/pi;
deg2rad=pi/180;

data_exp0 = dlmread('States_init_cond.txt');

data_exp2 = dlmread('teste_theta4_2.txt');

data_exp3 = dlmread('teste_theta4_3.txt');

data_exp4 = dlmread('teste_theta4_4.txt');


go=155;
stop=go+140;
media=mean(data_exp0(stop-15:stop,1)*(deg2rad));

roll_0 = data_exp0(go:stop,1)*(deg2rad)-media;
roll_dot_0 = data_exp0(go:stop,2)*(deg2rad);


go=110;
stop=250;
media=mean(data_exp2(stop-5:stop,1)*(deg2rad));

roll_2 = data_exp2(110:250,1)*(deg2rad)-media;
roll_dot_2 = data_exp2(110:250,2)*(deg2rad);

go=86;
stop=226;
media=mean(data_exp3(stop-10:stop,1)*(deg2rad));
roll_3 = data_exp3(go:stop,1)*(deg2rad)-media;
roll_dot_3 = data_exp3(go:stop,2)*(deg2rad);

go=39;
stop=179;
media=mean(data_exp4(stop-10:stop,1)*(deg2rad));

roll_4 = data_exp4(go:stop,1)*(deg2rad)-media;
roll_dot_4 = data_exp4(go:stop,2)*(deg2rad);

out = ga(@fitfun3,3,[],[],[],[],[0.001 -0.5 0.1],[0.02 0.9 1])

F = @(t, x) [x(2); -out(2)/out(1)*x(2) - out(3)/out(1)*sin(x(1))];
T = 0:0.02:(length(roll_0)-1)*0.02;
S = [roll_0(1) roll_dot_0(1)];
[t, yfit] = ode45(F, T, S);

F = @(t, x) [x(2); -out(2)/out(1)*x(2) - out(3)/out(1)*sin(x(1))];
T = 0:0.02:(length(roll_2)-1)*0.02;
S = [roll_2(1) roll_dot_2(1)];
[t, yfit2] = ode45(F, T, S);

F = @(t, x) [x(2); -out(2)/out(1)*x(2) - out(3)/out(1)*sin(x(1))];
T = 0:0.02:(length(roll_3)-1)*0.02;
S = [roll_3(1) roll_dot_3(1)];
[t, yfit3] = ode45(F, T, S);

F = @(t, x) [x(2); -out(2)/out(1)*x(2) - out(3)/out(1)*sin(x(1))];
T = 0:0.02:(length(roll_4)-1)*0.02;
S = [roll_4(1) roll_dot_4(1)];
[t, yfit4] = ode45(F, T, S);


figure
subplot(2,4,1)
plot(0.02*[1:length(roll_0)],roll_0,'b',0.02*[1:length(roll_0)],yfit(:,1),'r--')
xlabel('Time [s]')
ylabel('Rad')
legend('Exp.Data','Mod.Indent')
title('Ensaio 0')

subplot(2,4,2)
plot(0.02*[1:length(roll_2)],roll_2,'b',0.02*[1:length(roll_2)],yfit2(:,1),'r--')
xlabel('Time [s]')
ylabel('Rad')
legend('Exp.Data','Mod.Indent')
title('Ensaio 1')

subplot(2,4,3)
plot(0.02*[1:length(roll_3)],roll_3,'b',0.02*[1:length(roll_3)],yfit3(:,1),'r--')
xlabel('Time [s]')
ylabel('Rad')
legend('Exp.Data','Mod.Indent')
title('Ensaio 2')

subplot(2,4,4)
plot(0.02*[1:length(roll_4)],roll_4,'b',0.02*[1:length(roll_4)],yfit4(:,1),'r--')
xlabel('Time [s]')
ylabel('Rad')
legend('Exp.Data','Mod.Indent')
title('Ensaio 3')


subplot(2,4,5)
plot(0.02*[1:length(roll_dot_0)],roll_dot_0,'k',0.02*[1:length(roll_0)],yfit(:,2),'r--')
xlabel('Time [s]')
ylabel('Rad/s')
legend('Exp.Data','Mod.Indent')

subplot(2,4,6)
plot(0.02*[1:length(roll_dot_2)],roll_dot_2,'k',0.02*[1:length(roll_2)],yfit2(:,2),'r--')
xlabel('Time [s]')
ylabel('Rad/s')
legend('Exp.Data','Mod.Indent')

subplot(2,4,7)
plot(0.02*[1:length(roll_dot_3)],roll_dot_3,'k',0.02*[1:length(roll_3)],yfit3(:,2),'r--')
xlabel('Time [s]')
ylabel('Rad/s')
legend('Exp.Data','Mod.Indent')

subplot(2,4,8)
plot(0.02*[1:length(roll_dot_4)],roll_dot_4,'k',0.02*[1:length(roll_4)],yfit4(:,2),'r--')
xlabel('Time [s]')
ylabel('Rad/s')
legend('Exp.Data','Mod.Indent')


figure

y=fft(roll_0);
fs = 1/0.02;
n = length(roll_0);     % number of samples
f = (0:n-1)*(fs/n);     % frequency range
power = abs(y).^2/n; 
subplot(4,1,1) 
plot(f,power,'b-*')
xlabel('Frequency')
ylabel('Power')
title('Ensaio 0')
hold on

ymod=fft(yfit(:,1));
fs = 1/0.02;
n = length(roll_0);        % number of samples
fmod = (0:n-1)*(fs/n);     % frequency range
powermod = abs(ymod).^2/n;  
plot(fmod,powermod,'r')

y=fft(roll_2);
fs = 1/0.02;
n = length(roll_2);     % number of samples
f = (0:n-1)*(fs/n);     % frequency range
power = abs(y).^2/n;  
subplot(4,1,2)
plot(f,power,'b-*')
xlabel('Frequency')
ylabel('Power')
title('Ensaio 1')
hold on

ymod=fft(yfit2(:,1));
fs = 1/0.02;
n = length(roll_2);        % number of samples
fmod = (0:n-1)*(fs/n);     % frequency range
powermod = abs(ymod).^2/n;  
plot(fmod,powermod,'r')

legend('Exp.Data','Mod.Indent')

y=fft(roll_3);
fs = 1/0.02;
n = length(roll_3);     % number of samples
f = (0:n-1)*(fs/n);     % frequency range
power = abs(y).^2/n;  
subplot(4,1,3)
plot(f,power,'b-*')
xlabel('Frequency')
ylabel('Power')
title('Ensaio 2')
hold on

ymod=fft(yfit3(:,1));
fs = 1/0.02;
n = length(roll_3);        % number of samples
fmod = (0:n-1)*(fs/n);     % frequency range
powermod = abs(ymod).^2/n;  
plot(fmod,powermod,'r')

legend('Exp.Data','Mod.Indent')

y=fft(roll_4);
fs = 1/0.02;
n = length(roll_4);     % number of samples
f = (0:n-1)*(fs/n);     % frequency range
power = abs(y).^2/n;  
subplot(4,1,4)
plot(f,power,'b-*')
xlabel('Frequency')
ylabel('Power')
title('Ensaio 3')
hold on

ymod=fft(yfit4(:,1));
fs = 1/0.02;
n = length(roll_4);        % number of samples
fmod = (0:n-1)*(fs/n);     % frequency range
powermod = abs(ymod).^2/n;  
plot(fmod,powermod,'r')

legend('Exp.Data','Mod.Indent')