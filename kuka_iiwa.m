close all,
clear,clc;

%% Modified DH parameters

L1 =  Link([ 0,    0,       0,        0,      0], 'modified');
L2 =  Link([ 0,    0,      -0.03,      -pi/2,    0], 'modified');
L3 =  Link([ 0,    0,      0.34,       0,      0], 'modified');
L4 =  Link([ 0,    0.338,   -0.04,   -pi/2,    0], 'modified');
L5 =  Link([ 0,    0,      0,        pi/2,    0], 'modified');
L6 =  Link([ 0,    0,      0,       -pi/2,    0], 'modified');
robot_modified = SerialLink([L1,L2,L3,L4,L5,L6]); 
% robot_modified.display();  
robot_modified.teach([0 0 0 0 0 0]);

theta_target = [deg2rad(19.8), deg2rad(-33.6), deg2rad(-13.6), deg2rad(7.2), deg2rad(-17.2), deg2rad(37.2)];

p_target = robot_modified.fkine(theta_target);
q = rad2deg(robot_modified.ikine(p_target));

%% 
