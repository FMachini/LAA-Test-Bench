function [out] = estimateInput(u,x)

m4 = 0.559;
L1 = 0.215;
L2 = 0.225;
g = 9.81;

    Fz = u(1)+m4*g*cos(x(3));
    Fy = u(2)+m4*g*sin(x(3));
    
    if u(2)>0.0001
        f2 = (L1*Fz - u(3))/(L1+L2);
        alpha1 = atan2( Fy , (Fz - f2));
        f1 = Fy / sin(alpha1);
        alpha1 = abs(alpha1);
        alpha2 = 0;
        %f1(1,i)+f2(1,i)
    elseif  u(2)<-0.0001
            f1 = (L2*Fz + u(3))/(L1+L2);
            alpha2 = atan2( Fy, ( Fz - f1 ));
            f2 = Fy / sin(alpha2);
            alpha2 = abs(alpha2);
            alpha1 = 0;

    elseif u(2) > -0.0001 || u(2) < 0.001
            f1 = ( Fz + u(3)/L2 ) / (1+L1/L2);
            f2 = Fz - f1(1);
            alpha1 = 0;
            alpha2 = 0;
    end
    
    f1 = f1-m4*g/2;
    f2 = f2-m4*g/2;
    
    out = [f1;f2;alpha1;alpha2];
end