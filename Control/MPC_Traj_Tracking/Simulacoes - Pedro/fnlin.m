%equa��o n�o linear que rege o comportamento do sistema
function xdot= fnlin(t,x,u,G,Km,Mb,i0,xb0)

x1d = x(2);
x2d = G - 0.95*Km*u^2/(2*Mb*(x(1)+xb0)^2);
% x2d=G - 1*Km*(u+i0)^2/(2*Mb*(x(1)+xb0)^2);
xdot = [x1d;x2d];

end