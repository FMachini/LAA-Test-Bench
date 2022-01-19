function J = CostFun2(x)

global A Cc vel Axd Cxd Bxd;

Qob = diag(x(1:9));

Rob = diag(x(10:end));

L = dlqr(Axd',Cxd', Qob,Rob);

xmpck = zeros(9,size(vel,1));
up = zeros(4,size(A,1));

for  k = 1:(size(vel,1)-1)
    up(:,k) = [A(k,7)-1.7 A(k,8)-1.7 A(k,9)*pi/180 A(k,10)*pi/180]';
    yout = Cc*A(k,1:6)';
    
    % State Observer prediction
    ye = Cxd*xmpck(:,k);
    xmpck(:,k+1) = Axd*xmpck(:,k) + L'*(yout - ye) + Bxd*up(:,k);
end

J = sum((xmpck(4,:)-vel').^2)/size(A,1)


end

