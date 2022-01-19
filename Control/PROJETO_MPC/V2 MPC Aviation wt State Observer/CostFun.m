function J = CostFun(x)

global A Ad Bd Cd Cc vel Axd Cxd Bxd;

Qob = diag(x(1:9));

Rob = diag(x(10:end));

Cc = [0 0 1*pi/180 0 0 0;
    1*pi/180 0 0 0 0 0;
    0 1*pi/180 0 0 0 0];

xmpck = zeros(9,size(vel,1));
up = zeros(4,size(A,1));
J = 1e6;
try
    L = dlqr(Ax',Cx', Qob,Rob);
    
catch L
    if (strcmp(L.identifier,'Control:design:lqr2')) || (strcmp(L.identifier,'Control:design:lqr1'))
        J = 1e6;
     
    else
        LL = dlqr(Ax',Cx', Qob,Rob);
        for  k = 1:(size(vel,1)-1)
            up(:,k) = [A(k,7)-1.7 A(k,8)-1.7 A(k,9)*pi/180 A(k,10)*pi/180]';
            yout = Cc*A(k,1:6)';
            
            % State Observer prediction
            ye = Cx*xmpck(:,k);
            xmpck(:,k+1) = Ax*xmpck(:,k) + LL'*(yout - ye) + Bx*up(:,k);
        end
        
        J = sum((xmpck(4,:)-vel').^2)/size(A,1);
    end
end

J

end

