close all; clear all; clc;

%% Creating Robot Tree
robot = rigidBodyTree;

robot.DataFormat = 'row';
l = 0.2;
gamma = pi/4;
%% Creating Bodies
% Joint 1  
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
tform = trvec2tform([l*cos(gamma), l*sin(gamma), 0])*eul2tform([gamma, 0, pi/2]);
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;
addBody(robot,body1,'base')


% Joint 2  
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
tform2 = trvec2tform([0, 0, 0])*eul2tform([0, pi/2, 0]); %ZYX rotation sequence for euler2tform
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;

addBody(robot,body2,'body1');

% Joint 3  
% body3 = rigidBody('body3');
% jnt3 = rigidBodyJoint('jnt3','revolute');
% jnt3.HomePosition = -18.7*pi/180;
% tform3 = trvec2tform([l(2), 0, 0]); 
% setFixedTransform(jnt3,tform3);
% body3.Joint = jnt3;
% body3.Mass = 0.115/2;
% body3.CenterOfMass = [0 l(3)/2 0];
% body3.Inertia = [0 1.4341e-04 1.4341e-04 0 0 0];
% 
% addBody(robot,body3,'body2');
% 
% Joint 4  
% body4 = rigidBody('body4');
% jnt4 = rigidBodyJoint('jnt4','revolute');
% tform4 = trvec2tform([0, l(3), 0])*eul2tform([0, pi/2, 0]); %ZYX rotation sequence for euler2tform
% setFixedTransform(jnt4,tform4);
% body4.Joint = jnt4;
% body4.Mass = 0;
% body4.CenterOfMass = [0 0 0];
% body4.Inertia = 0*[Ix(1) Iy(1) 0.8 0 0 0];
% 
% addBody(robot,body4,'body3');
% 
% Counter Weight
% counterWeight = rigidBody('counterWeight');
% tformCW = trvec2tform([-0.33, 0, 0]);
% setFixedTransform(counterWeight.Joint,tformCW);
% counterWeight.Mass = 1.7;
% counterweight.CenterOfMass = [0 0 0];
% counterweight.Inertia = [0 0 0 0 0 0];
% 
% addBody(robot,counterWeight,'body2')
% 
% End Effector
% endeffector = rigidBody('endeffector');
% tform5 = trvec2tform([0, l(4), 0]);
% setFixedTransform(endeffector.Joint,tform5);
% endeffector.Mass = 0.35;%m(4);
% endeffector.CenterOfMass = [0 0 0];
% endeffector.Inertia = [0.01 6e-4 0.0187 0 0 0];
% 
% addBody(robot,endeffector,'body4');

figure('rend','painters','pos',[10 10 1000 800])
show(robot)
