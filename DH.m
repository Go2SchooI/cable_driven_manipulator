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
%% 正逆运动学均可调用自己编写库

N = 1;
x_target = p_target.t';
x_ = zeros(1,3);
e = zeros(1,6);
eplot = zeros(1,N);
theta = [0, 0, 0, 0, 0, 0];

use_Pseudo = 0;

for j = 1 : 1000
    %平移    
    p = fkine(theta);
    x_(1) = p(1,4);
    x_(2) = p(2,4);
    x_(3) = p(3,4);
%     p = robot_modified.fkine(theta);
%     x_(1:3) = p.t';
    e(1:3) = x_target - x_;
    
    %旋转
    Rq = t2r(p);
    rrr = t2r(p_target) * Rq';
    [thn,V] = tr2angvec(t2r(p_target) * Rq');
    e(4:6) = thn * V;
    eplot(j) = e * e';
   
%     Jaco = jacob0(robot_modified, theta);
    Jaco = jacobe(theta);

    if (use_Pseudo)
        dq = Jaco' \ (Jaco * Jaco') * e';
    else
        dq = Jaco' * e';
    end
    
    dq = 0.05 * dq;

    theta = theta + dq';
end 

thetadeg = rad2deg(theta);
%% 

figure();
plot(eplot)