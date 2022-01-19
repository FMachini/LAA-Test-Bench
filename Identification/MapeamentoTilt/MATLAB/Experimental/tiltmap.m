%% Mapeamento do tilt rotor
clear all
close all


% Mapeamento estático



% Resultado dos ensaios
% ADC com alimentação 5 V
% Tilt do braço maior marcado como m1
% potenciometro

y1 = [0 1 3 6 9 11 14 17 20 24 26 31 34 39 42];
y2 = [-1 1 3 6 8 11 14 17 20 23 26 31 34 39 42];
y3 = [-1 1 2 5 8 11 13 17 20 23 27 30 34 39 42];

x1 = [263 254 247 237 227 218 208 198 187 174 162 150 137 124 109];
x2 = [265 257 250 240 234 220 212 201 190 174 164 151 139 126 111];
x3 = [265 257 250 241 234 220 212 201 190 177 165 151 139 127 111];

z=linspace(1650,2350,50);

% ADC com alimentação 5 V
% Tilt do braço menor marcado como m2

y1 = [0 3 5 7 9 12 15 18 21 24 27 30 33 37 40];
y2 = [2 3 5 7 10 12 14 18 21 24 27 30 34 37 40];
y3 = [1 3 5 7 9 12 15 18 21 24 27 31 33 37 40];

x1 = [801 796 789 780 771 761 750 737 724 709 696 682 668 651 639];
x2 = [801 796 789 780 771 761 750 737 725 710 696 683 668 652 638];
x3 = [801 796 789 781 771 761 750 737 725 710 696 683 669 652 639];

z=linspace(1650,2350,50);

xtotal= [x1 x2 x3];
ytotal= [y1 y2 y3];


p=polyfit(xtotal,ytotal,1)

f= polyval(p,xtotal) 

plot(xtotal,ytotal,'r*');
hold on
plot(xtotal,f,'b');


