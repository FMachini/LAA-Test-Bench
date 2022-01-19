%Gabriel Renato Oliveira Alves 
%LQRi - Controle de atitude (theta4)

function res = odemaple(t, x ,xp, u)
    %Criando a bancada
    m1 = 2.761;
    m2 = 0.154;
    m3 = 0.115;
    m4 = 0.559;
    l1 = 0;
    l2 = 0.595;
    l3 = 0.173;
    l4 = 0;
    Ix1 = 0.004165125;
    Ix2 = 0.001006095;
    Ix3 = 0.00001723333;
    Ix4 = 0.01358627;
    Iy1 = 0.289994709;
    Iy2 = 0.001002423;
    Iy3 = 0.00008259913;
    Iy4 = 0.00051408;
    Iz1 = 0.28761702;
    Iz2 = 0.00001835292;
    Iz3 = 0.00007608364;
    Iz4 = 0.01329993;
    b1 = 0.215;
    b2 = 0.225;
    
    % Parametros de theta4
    mi_d = 0.0495 * 1.0;
    CG = 0.6925 * 1.0;
    
    % Parametros do contra-peso
    mcp = 0.6730; %0.729;
    lcp = 0.322;
    
    % Parametros do motor
    p = [7.571,     0;
             0, 6.911];
    q = [0.7827,      0; 
              0, 0.7360];
    %Dinamica do motor
    Fm = [x(4);
          x(5)];
      
    Fmp = [xp(4);
           xp(5)];
    
    res45 = Fmp + p * Fm - q * u([1,2],1);
    
    % Parametros do tilt
    pt = [7.8413,      0;
              0, 7.5815];
    qt = eye(2);
    %Dinamica do ilte
    alfa = [x(6);
            x(7)];
      
    alfap = [xp(6);
             xp(7)];
    
    res67 = alfap + pt * alfa - qt * u([3,4],1);
    
    %Dinamica da bancada
    q =     [ x(1),  x(2), -x(2)-pi/2,  x(3)];
    qdot =  [ x(8),  x(9),      -x(9),  x(10)];
    qddot = [xp(8), xp(9),     -xp(9), xp(10)];
    
    % Definindo os termo Dq,Cq,Gq,nJp
    [Dq,Cq,Gq,nJp] = matrizesBancada(q(1),q(2),q(3),q(4),qdot(1),qdot(2),qdot(3),qdot(4));
    %alfa = [ 0;5*pi/180];
    %Definindo termo Gamma b
    tilt = [-cos(alfa(1)),  -cos(alfa(2));
            sin(alfa(1)), -sin(alfa(2));
                       0,             0;
                       0,             0;
                       0,             0;
                     b1*cos(alfa(1)), -b2*cos(alfa(2))];
        
    f = (Fm+[2.2;2.2]);
    %f =(m1+m2+m4)*[1;1];% -(Fm+[2.2;2.2]);
    gamma_b = nJp * tilt * f; 
    
    Extra = [0;
             -lcp*mcp*9.81*cos(q(2));
             0;
             mi_d*qdot(4) + CG*q(4)];
         
    res8910 = Dq*qddot' + Cq + Extra + Gq - gamma_b;
    
    % Implicit function
    res123 = [xp(1) - x(8);
              xp(2) - x(9);
              xp(3) - x(10)];
    res = [res123;
           res45;
           res67;
           res8910([1,2,4])];
end