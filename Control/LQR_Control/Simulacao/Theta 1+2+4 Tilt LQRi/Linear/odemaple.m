%Gabriel Renato Oliveira Alves 
%LQRi - Controle de atitude e altura(theta24)

function res = odemaple(t, x ,xp, u)
    %Criando a bancada
    m1 = 2.761;
    m2 = 0.154;
    m3 = 0.115;
    m4 = 1.278;
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
    b2 = 0.215;
    
    % Parametros de theta4
    mi_d = 0.0495 * 1.0;
    CG = 0.6925 * 1.0;
    
    % Parametros do contra-peso
    mcp = 1.709; %0.729;
    %mcp = 0.729; 
    lcp = 0.33;
    
    % Parametros do motor
    %p = [9.072, 0;
    %     0, 8.016];
     
    p = [9.072, 0;
         0, 8.016];
    coef2 = [-0.000021708654545   0.010473514036364  -0.077660122909090];
    coef1 = [-0.000063260930216   0.013139466306494  -0.175069018957576];
    
    q = [polyval(coef1,u(1)),                   0; 
                           0, polyval(coef2,u(2))];
 
    %Dinamica do motor
    Fm = [x(3);
          x(4)];
      
    Fmp = [xp(3);
           xp(4)];
    
    res34 = Fmp + p * Fm - q * u;
    
    %Dinamica da bancada
    q =     [0, x(1),     -x(1)-pi/2, x(2)];
    qdot =  [0, x(5),     -x(5), x(6)];
    qddot = [0, xp(5),    -xp(5), xp(6)];
    
    % Definindo os termo Dq,Cq,Gq,nJp
    [Dq,Cq,Gq,nJp] = matrizesBancada(q(1),q(2),q(3),q(4),qdot(1),qdot(2),qdot(3),qdot(4));
    %Definindo termo Gamma e
    tilt = [-1, -1;
            0, 0;
            0, 0;
            0, 0;
            0, 0;
          b1, -b2];
    f = (Fm);
    gamma_b = nJp * tilt * f; 
    
    Extra = [0;
             lcp*mcp*9.81*cos(q(2));
             0;
             -mi_d*qdot(4) + -CG*q(4)];
         
    res56 = Dq*qddot' + Cq - Extra + Gq - gamma_b;
    
    %como calcular forcas iniciais
    %A=[b1 -b1; 1 1]
    %B=[.023*sind(1)*m4*9.81;(-Extra(2)+Gq(2))/l2]
    %forcas=inv(A)*B
    % Implicit function
    res1 = xp(1) - x(5);
    res2 = xp(2) - x(6);
    res = [res1;
           res2;
           res34;
           res56([2,4])];
end