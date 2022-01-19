%LQRi - Controle de atitude e altura(theta24)

function data = readExpData(filename)
global Ts;

rad2deg=180/pi

txtfile = dlmread(filename);
data = iddata(txtfile(:,1:6)*rad2deg,txtfile(:,7:10),Ts,'ExperimentName','Ensaio Controle LQRi 124 com tilt');
data.OutputName = {'Theta1','Theta2','Theta4','Theta1_dot','Theta2_dot','Theta4_dot'};
data.OutputUnit = {'graus','graus','graus','graus/s','graus/s','graus/s'};
%data.OutputUnit = {'rad','rad','rad','rad/s','rad/s','rad/s',};
data.InputName = {'F1','F2','alpha1','alpha2'};
data.InputUnit = {'N','N','rad','rad'};

end
