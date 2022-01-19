function [c,ceq] = minharest(x)
ceq  = [];
SystemParameters;

omg1_0 = x(1);
omg2_0 = x(2);
omg3_0 = x(3);
omg4_0 = x(4);
alpha1_0 = x(5);
alpha2_0 = x(6);
alpha3_0 = x(7);
alpha4_0 = x(8);

tol = 0;
c = [
     (grav*mass*sin(theta0) + kp_m*(omg1_0^2*sin(alpha1_0) - omg3_0^2*sin(alpha3_0)))/mass - tol; % uponto
      -(grav*mass*sin(theta0) + kp_m*(omg1_0^2*sin(alpha1_0) - omg3_0^2*sin(alpha3_0)))/mass + tol; % uponto
       %
       (-grav*mass*sin(phi0)*cos(theta0) + kp_m*(omg2_0^2*sin(alpha2_0) - omg4_0^2*sin(alpha4_0)))/mass - tol;  % vponto
       -(-grav*mass*sin(phi0)*cos(theta0) + kp_m*(omg2_0^2*sin(alpha2_0) - omg4_0^2*sin(alpha4_0)))/mass + tol;  % vponto
       %
       (-grav*mass*cos(phi0)*cos(theta0) + kp_m*omg1_0^2*cos(alpha1_0) + kp_m*omg2_0^2*cos(alpha2_0) + kp_m*omg3_0^2*cos(alpha3_0) + kp_m*omg4_0^2*cos(alpha4_0))/mass - tol;  % wponto
       -(-grav*mass*cos(phi0)*cos(theta0) + kp_m*omg1_0^2*cos(alpha1_0) + kp_m*omg2_0^2*cos(alpha2_0) + kp_m*omg3_0^2*cos(alpha3_0) + kp_m*omg4_0^2*cos(alpha4_0))/mass + tol;  % wponto     
       %
       (Jm2*omg2_0*sin(alpha2_0)^2 - Jm4*omg4_0*sin(alpha4_0)^2 + b_drag*(-omg1_0^2*sin(alpha1_0) + omg3_0^2*sin(alpha3_0)) + kp_m*l_b*(omg2_0^2*cos(alpha2_0) - omg4_0^2*cos(alpha4_0)))/Jxx - tol;  % pponto
       -(Jm2*omg2_0*sin(alpha2_0)^2 - Jm4*omg4_0*sin(alpha4_0)^2 + b_drag*(-omg1_0^2*sin(alpha1_0) + omg3_0^2*sin(alpha3_0)) + kp_m*l_b*(omg2_0^2*cos(alpha2_0) - omg4_0^2*cos(alpha4_0)))/Jxx + tol;  % pponto
       %
       (-Jm1*omg1_0*sin(alpha1_0)^2 + Jm3*omg3_0*sin(alpha3_0)^2 + b_drag*(omg2_0^2*sin(alpha2_0) - omg4_0^2*sin(alpha4_0)) + kp_m*l_b*(-omg1_0^2*cos(alpha1_0) + omg3_0^2*cos(alpha3_0)))/Jyy - tol;  % qponto
       -(-Jm1*omg1_0*sin(alpha1_0)^2 + Jm3*omg3_0*sin(alpha3_0)^2 + b_drag*(omg2_0^2*sin(alpha2_0) - omg4_0^2*sin(alpha4_0)) + kp_m*l_b*(-omg1_0^2*cos(alpha1_0) + omg3_0^2*cos(alpha3_0)))/Jyy + tol;  % qponto
       %
       (-b_drag*omg1_0^2*cos(alpha1_0) + b_drag*omg2_0^2*cos(alpha2_0) - b_drag*omg3_0^2*cos(alpha3_0) + b_drag*omg4_0^2*cos(alpha4_0))/Jzz - 1e-3;  % rponto
       -(-b_drag*omg1_0^2*cos(alpha1_0) + b_drag*omg2_0^2*cos(alpha2_0) - b_drag*omg3_0^2*cos(alpha3_0) + b_drag*omg4_0^2*cos(alpha4_0))/Jzz + 1e-3;  % rponto
%        omg1_0 - omg3_0;
%        omg2_0 - omg4_0;
%        alpha1_0 - alpha3_0;
%        alpha2_0 - alpha4_0;
       ]

end