clear all; close all; clc
clc

  m = [2.761, 0.154, 0.115, 0.6];
  l = [ 0.31, 0.595, 0.173,  0.05];
  Ix= [0.004165125, 0.001006095, 0.00001723333, 0.01];
  Iy= [0.289994709, 0.306954, 0.00008259913, 0.00051408];
  Iz= [ 0.28761702, 0.00001835292, 0.00007608364, 0.01329993];
  
  dhparams = [0       0     0       0;
              0     -pi/2    l(1)    0;
              l(2)    0    0    0;
              l(3)    pi/2   0   0;
              l(4)       0    0   0;
              0       0     0       0];

         
 robot = rigidBodyTree; % creates rigid body object
 
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
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','fixed');
jnt4.HomePosition = pi/2;

setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');
 
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;

addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')


showdetails(robot)

show(robot);
axis([-1,1,-1,1,-0.5,1])
grid on

robot.Gravity = [0 0 9.81]; %define gravity direction wrt base

%% Defining Bodies Properties

body1.Mass = 0.914;
body1.CenterOfMass = [0 0 l(1)/2];
body1.Inertia = [0.007 0.007 6.835e-4 0 0 0];

body2.Mass = m(1);
body2.CenterOfMass = [0 0 0];
body2.Inertia = [Ix(1) Iy(1) 0.8 0 0 0];

body3.Mass = 0.351;
body3.CenterOfMass = [l(3)/2 0 0];
body3.Inertia = [Ix(1) Iy(1) Iz(1) 0 0 0];

body5.Mass = m(4);
body5.CenterOfMass = [0 0 0];
body5.Inertia = [0.01 6e-4 0.0187 0 0 0];

%% Input external forces on rigid body
robot.DataFormat = 'row';
q = homeConfiguration(robot); 
ndof = size(q,2);
wrench = [0 0 0 1 0 0]; %[Tx Ty Tz Fx Fy Fz] vector
fext = externalForce(robot,'body4',wrench,q);
show(robot,q,'PreservePlot',false)
%% Compute the resultant joit accelerations
kend = 100;
Ts = 0.02;
x = zeros(2*ndof,kend);
x(:,1) = [q zeros(size(q))];

for  i = 1:100
    fext = externalForce(robot,'body4',wrench,x(1:ndof,i)');
    qddot = forwardDynamics(robot,x(1:ndof,i)',x(ndof+1:end,i)',[],fext);
    qdot = qddot*Ts;
    q = qdot*Ts;
    x(:,i+1) = x(:,i) + [q qdot]';
    show(robot,x(1:ndof,i+1)','PreservePlot',false);
    axis([-1,1,-1,1,-0.5,1])
    pause(0.01)
end



