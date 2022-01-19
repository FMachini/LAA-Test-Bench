clear all; close all; clc;

load exampleRobots.mat lbr
showdetails(lbr)
lbr.DataFormat = 'row';

lbr.Gravity = [0 0 -9.81];

q = homeConfiguration(lbr);
show(lbr,q,'PreservePlot',false);
axis([-1,1,-1,1,-1.5,1.5])
ndof = size(q,2);

wrench = [0 0 0 0 0 0];
kend = 100;
x = zeros(2*7,kend);
x(:,1) = [q 0 0 0 0 0 0 0];
Ts = 0.02;

for  i = 1:kend
    fext = externalForce(lbr,'tool0',wrench,x(1:7,i)');
    qddot = forwardDynamics(lbr,x(1:7,i)',x(8:end,i)',[],fext);
    qdot = x(ndof+1:end,i) + qddot'*Ts;
    q = x(1:ndof,i) + qdot*Ts;
    x(:,i+1) = [q;qdot];
    %x(:,i+1) = x(:,i) + [qdot q]';
    show(lbr,x(1:7,i+1)','PreservePlot',false);
    axis([-1,1,-1,1,-1.5,1.5])
    pause(0.01)
end
