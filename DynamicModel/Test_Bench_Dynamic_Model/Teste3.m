clear all; close all; clc
clc

  m = [2.761, 0.154, 0.115, 0.6];
  l = [ 0.31, 0.595, 0.173,  0.05];
  Ix= [0.004165125, 0.001006095, 0.00001723333, 0.01];
  Iy= [0.289994709, 0.306954, 0.00008259913, 0.00051408];
  Iz= [ 0.28761702, 0.00001835292, 0.00007608364, 0.01329993];
  
  dhparams = [0       0     l(1)/2       0;
              0     -pi/2    l(1)/2    0;
              l(2)    0    0    0;
              l(3)    pi/2   0   0;
              l(4)       0    0   0;
              0       0     0       0];

         
 robot = rigidBodyTree; % creates rigid body object
 robot.Gravity = [0 0 -9.81]; %define gravity direction wrt base
 robot.DataFormat = 'row';
 
 %% Creating joints
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');

setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;

addBody(robot,body1,'base')

body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt3.HomePosition = 0*pi/180;

setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');

 
body2.Joint = jnt2;
body3.Joint = jnt3;

%% Defining Bodies Properties

body1.Mass = 0*0.914;
body1.CenterOfMass = [0 0 0*l(1)/2];
body1.Inertia = 0*[0.007 0.007 6.835e-4 0 0 0];

body2.Mass = 0*m(1);
body2.CenterOfMass = [0 0 0];
body2.Inertia = 0*[Ix(1) Iy(1) 0.8 0 0 0];

body3.Mass = m(4);
body3.CenterOfMass = [0 0 0];
body3.Inertia = 0*[0.01 6e-4 0.0187 0 0 0];

%% Adding Bodies


addBody(robot,body2,'body1')
addBody(robot,body3,'body2')

showdetails(robot)


% show(robot);
% axis([-1,1,-1,1,-0.5,1])
% grid on

%robot.Gravity = [0 0 -9.81]; %define gravity direction wrt base


%% Input external forces on rigid body
q = homeConfiguration(robot); 
ndof = size(q,2);
wrench = [0 0 0 0 0 0]; %[Tx Ty Tz Fx Fy Fz] vector
fext = externalForce(robot,'body3',wrench,q);
show(robot,q,'PreservePlot',false)
comPos = centerOfMass(robot);
hold on
plot3(comPos(1),comPos(2),comPos(3),'or')

%% Compute the resultant joit accelerations
kend = 100;
Ts = 0.02;
x = zeros(2*ndof,kend);
x(:,1) = [q zeros(size(q))];

for  i = 1:kend
    fext = externalForce(robot,'body3',wrench,x(1:ndof,i)');
    qddot = forwardDynamics(robot,x(1:ndof,i)',x(ndof+1:end,i)',[],fext);
    qdot = x(ndof+1:end,i) + qddot'*Ts;
    q = x(1:ndof,i) + qdot*Ts;
    x(:,i+1) = [q;qdot];
    show(robot,x(1:ndof,i+1)','PreservePlot',false);
    axis([-1,1,-1,1,-0.5,1])
    pause(0.01)
end

figure
subplot(2,2,1)
plot(x(1,:))
ylabel('$\theta_1$','Interpreter','latex')

subplot(2,2,2)
plot(x(3,:))
ylabel('$\dot{\theta}_1$','Interpreter','latex')

subplot(2,2,3)
plot(x(2,:))
ylabel('$\theta_2$','Interpreter','latex')

subplot(2,2,4)
plot(x(4,:))
ylabel('$\dot{\theta}_2$','Interpreter','latex')

