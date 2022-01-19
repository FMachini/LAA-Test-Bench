function dydt = fnlin(t, y, u)
a = 0.0187;
b = 0.06;%0.0495;
c = 0.6;%0.6925;
p = 5.348;
q = 13.43;
r = 1.319;
L1 = 0.225;
L2 = 0.215;

% theta = x1
% f1 = x2
% f3 = x3
% theta_p = x4
% f1_p = x5
% f2_p = x6
% u1 = pwm1
% u2 = pwm2

dydt = zeros(6,1);
dydt(1) = y(4);

%dydt(4) = (1/a)*(L1*u(1)*r/q - L2*u(2)*r/q - b*y(4) - c*y(1));
% caso queria simular sem a din. dos motores, descomentar l acima e comentar
% ls abaixo

dydt(2) = y(5);
dydt(3) = y(6);
dydt(4) = (1/a)*(L1*y(2) - L2*y(3) - b*y(4) - c*y(1));
dydt(5) = r*u(1) - p*y(5) - q*y(2);
dydt(6) = r*u(2) - p*y(6) - q*y(3);
end