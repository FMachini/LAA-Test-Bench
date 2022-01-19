 % This function creates the test bench robotic model considering 4 DOF
 % (theta1,theta2,theta3 and theta4). The Bicopter is assumed as the
 % endeffector body with fixed reference frame. Mass properties are defined
 % for each element.

  m = 0*[2.761, 0.154, 0.115, 0.6];
  l = [ 0.31, 0.605, 0.173,  0.05];
  Ix= 0*[0.004165125, 0.001006095, 0.00001723333, 0.01];
  Iy= 0*[0.289994709, 0.306954, 0.00008259913, 0.00051408];
  Iz= 0*[ 0.28761702, 0.00001835292, 0.00007608364, 0.01329993];

%% Creating Robot Tree
robot = rigidBodyTree;
robot.Gravity = [0 0 -9.81]; %define gravity direction wrt base
robot.DataFormat = 'row';

%% Creating Bodies
% Joint 1  
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
tform = trvec2tform([0, 0, l(1)/2]);
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;
body1.Mass = 0.3662;
body1.CenterOfMass = [0 0 l(1)/2];
body1.Inertia = [0.003 0.003 0 0 0 0];
addBody(robot,body1,'base')


% Joint 2  
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = 18.7*pi/180;
tform2 = trvec2tform([0, 0, l(1)/2])*eul2tform([0, 0, -pi/2]); %ZYX rotation sequence for euler2tform
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
body2.Mass = 0.7413;
body2.CenterOfMass = [-0.11 0 0];%[-0.0267 0 0];
body2.Inertia = [0 0.0226 0.0226 0 0 0];

addBody(robot,body2,'body1');

% Joint 3  
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt3.HomePosition = -18.7*pi/180;
tform3 = trvec2tform([l(2), 0, 0]); 
setFixedTransform(jnt3,tform3);
body3.Joint = jnt3;
body3.Mass = 0.115/2;
body3.CenterOfMass = [0 l(3)/2 0];
body3.Inertia = [0 1.4341e-04 1.4341e-04 0 0 0];

addBody(robot,body3,'body2');

% Joint 4  
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
tform4 = trvec2tform([0, l(3), 0])*eul2tform([0, pi/2, 0]); %ZYX rotation sequence for euler2tform
setFixedTransform(jnt4,tform4);
body4.Joint = jnt4;
% body4.Mass = 0;
% body4.CenterOfMass = [0 0 0];
% body4.Inertia = 0*[Ix(1) Iy(1) 0.8 0 0 0];

addBody(robot,body4,'body3');

% Counter Weight
counterWeight = rigidBody('counterWeight');
tformCW = trvec2tform([-0.33, 0, 0]);
setFixedTransform(counterWeight.Joint,tformCW);
counterWeight.Mass = 1*1.7;
counterweight.CenterOfMass = [0 0 0];
counterweight.Inertia = [0 0 0 0 0 0];

addBody(robot,counterWeight,'body2')

% End Effector
endeffector = rigidBody('endeffector');
tform5 = trvec2tform([0, l(4), 0]);
setFixedTransform(endeffector.Joint,tform5);
endeffector.Mass = 0.35;%m(4);
endeffector.CenterOfMass = [0 0 0];
endeffector.Inertia = [0.01 6e-4 0.0187 0 0 0];

addBody(robot,endeffector,'body4');

figure('rend','painters','pos',[10 10 1000 800])
show(robot)
comPos = centerOfMass(robot);
hold on
plot3(comPos(1),comPos(2),comPos(3),'or')