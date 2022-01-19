 %% Mapeamento do tilt rotor
close all
clear all
clc
% Constantes
rpm_2_rads=((2*pi)/60);
Kg_2_N=(9.81);
Ts=0.025;
% 
% txtfile = dlmread('T2_ME_01.txt');
% data = iddata([txtfile(:,2) txtfile(:,4) txtfile(:,3)],txtfile(:,1),Ts,'ExperimentName','COMPORTAMENTO Do TILT');
% data.OutputName = {'PWM','Angle','ADC'};
% data.OutputUnit = {'duty','degrees','bits'};
% data.InputName = {'Time'};
% data.InputUnit = {'seconds'};
% 
% txtfile = dlmread('T2_ME_02.txt');
% data2 = iddata([txtfile(:,2) txtfile(:,4) txtfile(:,3)],txtfile(:,1),Ts,'ExperimentName','COMPORTAMENTO Do TILT');
% data2.OutputName = {'PWM','Angle','ADC'};
% data2.OutputUnit = {'duty','degrees','bits'};
% data2.InputName = {'Time'};
% data2.InputUnit = {'seconds'};
% 
% txtfile = dlmread('T2_ME_03.txt');
% data3 = iddata([txtfile(:,2) txtfile(:,4) txtfile(:,3)],txtfile(:,1),Ts,'ExperimentName','COMPORTAMENTO Do TILT');
% data3.OutputName = {'PWM','Angle','ADC'};
% data3.OutputUnit = {'duty','degrees','bits'};
% data3.InputName = {'Time'};
% data3.InputUnit = {'seconds'};
% 
% 
% angle_eye=[30 26.5 23 19.5 17 13.5 11 8 6 3.5 1.5 0 -1.5 -2.5 -1.5 0 1.5 3.5 6 8 11 13.5 17 20 23 26 28.5;...
%     30.5 27 23.5 21.5 17.5 14.5 11.5 8.5 6.5 4.5 2.5 0.5 -0.5 -1.5 -0.5 0.5 2.5 4.5 6.5 9 11.5 14.5 17.5 20.5 23.5 26.5 30;...
%     30 26.5 23.5 20 17 14 11 8.5 6 4 2 0 -1 -2 -1 0 2 4 6 8.5 11 14.5 17 20 23 26 29.5];
% 
% CAD =[882 896 908 916.6 930 941.8 952 961 970 978 985 991.5 996.3 999.5 997 992 985.8 979 971 963 952.9 943.7 931 920.8 909 898 885 874;...
%       882 896 908 920 930 942 951.8 961.8 971 978 985 991.6 996 1000 996 992 985 979 971.2 963 953.2 943.2 931 921 909.2 898 885.8 873.8;...
%       882.2 895 907 919 930 941.4 951.6 961.4 970 978 985 991.6 996 1000 997 992 985 979 971 963 953 943.8 930.6 920 909 898 885.4 874];
% %command = [1700 1750 1800 1850 1900 1950 2000 2050 2100 2150 2200 2250 2300 2350 2300 - 1650];
% command = 1650:50:2350;
% command = [command(2:end), fliplr(command(2:end-1))];
% time  = linspace(0,336,27);
% 
% 
% 
% save('data.mat','data')
% 
% figure
% subplot(4,1,1)
% plot(data.u,movmean(data.y(:,3),10),'r*')
% title("Ensaio 1 Mapeamento Tilt 2")
% xlabel("Time [seconds]")
% ylabel("ADC [bits]")
% 
% subplot(4,1,2)
% plot(data.u,data.y(:,1),'b*')
% xlabel("Time [seconds]")
% ylabel("PWM [duty]")
% 
% subplot(4,1,3)
% plot(time,angle_eye(1,:),'g*')
% xlabel("Time [seconds]")
% ylabel("Angle [degrees]")
% 
% subplot(4,1,4)
% plot(command,angle_eye(1,:),'c*')
% xlabel("PWM [duty]")
% ylabel("Angle [degrees]")
% 
% 
% figure
% subplot(4,1,1)
% plot(data2.u,movmean(data2.y(:,3),5),'r*')
% title("Ensaio 2 Mapeamento Tilt 2")
% xlabel("Time [seconds]")
% ylabel("ADC [bits]")
% 
% subplot(4,1,2)
% plot(data2.u,data2.y(:,1),'b*')
% xlabel("Time [seconds]")
% ylabel("PWM [duty]")
% 
% subplot(4,1,3)
% plot(time,angle_eye(2,:),'g*')
% xlabel("Time [seconds]")
% ylabel("Angle [degrees]")
% 
% subplot(4,1,4)
% plot(command,angle_eye(2,:),'c*')
% xlabel("PWM [duty]")
% ylabel("Angle [degrees]")
% 
% figure
% subplot(4,1,1)
% plot(data3.u,movmean(data3.y(:,3),5),'r*')
% title("Ensaio 3 Mapeamento Tilt 2")
% xlabel("Time [seconds]")
% ylabel("ADC [bits]")
% 
% subplot(4,1,2)
% plot(data3.u,data3.y(:,1),'b*')
% xlabel("Time [seconds]")
% ylabel("PWM [duty]")
% 
% subplot(4,1,3)
% plot(time,angle_eye(3,:),'g*')
% xlabel("Time [seconds]")
% ylabel("Angle [degrees]")
% 
% subplot(4,1,4)
% plot(command,angle_eye(3,:),'c*')
% xlabel("PWM [duty]")
% ylabel("Angle [degrees]")
% 
% 
% ytotal= [angle_eye(1,:) angle_eye(2,:) angle_eye(3,:)];
% xtotal= [command command command];
% 
% 
% p=polyfit(xtotal,ytotal,1)
% 
% f= polyval(p,xtotal)
% 
% figure
% 
% plot(xtotal,ytotal,'r*');
% title("PWM Angle")
% hold on
% plot(xtotal,f,'b');
% 
% command = 1650:50:2350;
% command = [command(2:end), fliplr(command(1:end-1))];
% 
% 
% angle_eye=[30 26.5 23 19.5 17 13.5 11 8 6 3.5 1.5 0 -1.5 -2.5 -1.5 0 1.5 3.5 6 8 11 13.5 17 20 23 26 28.5 32.5;...
%    30.5 27 23.5 21.5 17.5 14.5 11.5 8.5 6.5 4.5 2.5 0.5 -0.5 -1.5 -0.5 0.5 2.5 4.5 6.5 9 11.5 14.5 17.5 20.5 23.5 26.5 30 33;...
%    30 26.5 23.5 20 17 14 11 8.5 6 4 2 0 -1 -2 -1 0 2 4 6 8.5 11 14.5 17 20 23 26 29.5 33];
% 
% 
% 
% 
% 
% xtotal= [CAD(1,:) CAD(2,:) CAD(3,:)];
% ytotal= [angle_eye(1,:) angle_eye(2,:) angle_eye(3,:)];
% 
% 
% p1=polyfit(xtotal,ytotal,1)
% 
% f1= polyval(p1,xtotal)
% 
% figure
% 
% plot(xtotal,ytotal,'r*');
% title("ANGLE CAD")
% hold on
% plot(xtotal,f1,'b');


txtfile = dlmread('T2_MD_01_15.txt');
data = iddata([txtfile(:,2) txtfile(:,4) txtfile(:,3)],txtfile(:,1),Ts,'ExperimentName','COMPORTAMENTO Do TILT');
data.OutputName = {'PWM','Angle','ADC'};
data.OutputUnit = {'duty','degrees','bits'};
data.InputName = {'Time'};
data.InputUnit = {'seconds'};

txtfile = dlmread('T2_MD_01_35.txt');
data2 = iddata([txtfile(:,2) txtfile(:,4) txtfile(:,3)],txtfile(:,1),Ts,'ExperimentName','COMPORTAMENTO Do TILT');
data2.OutputName = {'PWM','Angle','ADC'};
data2.OutputUnit = {'duty','degrees','bits'};
data2.InputName = {'Time'};
data2.InputUnit = {'seconds'};


% figure
% 
% subplot(3,1,1)
% plot(data.u,data.y(:,2),'r');
% title('Ensaio 1 motor ligado a 15%')
% ylabel("Angle [degrees]")
% xlabel("Time [s]")
% 
% subplot(3,1,2)
% plot(data.u,data.y(:,1),'g');
% ylabel("PWM tilt [duty]")
% xlabel("Time [s]")
% 
% subplot(3,1,3)
% plot(data.y(:,1),movmean(data.y(:,2),30),'r*');
% title('Relação PWM -> Angulo')
% hold on
% title("ANGLE PWM")
% ylabel("Angle [degrees]")
% xlabel("PWM [duty]")
% 
% 
% p2=polyfit(data.y(:,1),movmean(data.y(:,2),15),1)
% 
% f2= polyval(p2,data.y(:,1))
% 
% plot(data.y(:,1),f2,'b');
% 

figure

subplot(3,1,1)
plot(data2.u,data2.y(:,2),'r');
title('Ensaio 1 motor ligado a 35%')
ylabel("Angle [degrees]")
xlabel("Time [s]")

subplot(3,1,2)
plot(data2.u,data2.y(:,1),'g');
ylabel("PWM tilt [duty]")
xlabel("Time [s]")

subplot(3,1,3)
plot(movmean(data2.y(:,2),15),data2.y(:,1)*1000,'r*');
title('Relação PWM -> Angulo')
hold on
title("ANGLE PWM")
ylabel("Angle [degrees]")
xlabel("PWM [duty]")


p3=polyfit(movmean(data2.y(:,2),15),data2.y(:,1)*1000,1)

f3= polyval(p3,movmean(data2.y(:,2),15))

plot(movmean(data2.y(:,2),15),f3,'b');
p3(1)
p3(2)

% 
% 
% figure
% 
% subplot(4,1,2)
% plot(data.u,data.y(:,2),'r*')
% xlabel("Time [seconds]")
% ylabel("Angle [degrees]")
% 
% subplot(4,1,3)
% plot(data.u,data.y(:,1),'r*')
% xlabel("Time [seconds]") 
% ylabel("PWM [duty]")
% 
% subplot(4,1,4)
% plot(data.y(:,1),data.y(:,2),'r*')
% xlabel("PWM [duty]")
% ylabel("Angle [degrees]")

% 
% 
% figure
% plot(data.y,data.u,'r*')
% title("Mapeamento do angulo e da entrada em pwm")
% xlabel("PWM")
% ylabel("Angle [degress]")



% p=polyfit(xtotal,ytotal,1)
% 
% f= polyval(p,xtotal)
% 
% plot(xtotal,ytotal,'r*');
% hold on
% plot(xtotal,f,'b');
% 
% 
