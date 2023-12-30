close all,
clear,clc;

%% Modified DH parameters

L1 =  Link([ 0,    0.36,   0,       0,       0], 'modified');
L2 =  Link([ 0,    0,      0,      -pi/2,    0], 'modified');
L3 =  Link([ 0,    0.42,   0,       pi/2,    0], 'modified');
L4 =  Link([ 0,    0,      0,      -pi/2,    0], 'modified');
L5 =  Link([ 0,    0.4,    0,       pi/2,    0], 'modified');
L6 =  Link([ 0,    0,      0,      -pi/2,    0], 'modified');
L7 =  Link([ 0,    0.126,  0,       pi/2,    0], 'modified');
robot_modified = SerialLink([L1,L2,L3,L4,L5,L6,L7]); 
robot_modified.display();  
robot_modified.teach([0 0 0 0 0 0 0]);

theta_target = [deg2rad(19.8), deg2rad(-33.6), deg2rad(-13.6), deg2rad(7.2), deg2rad(-17.2), deg2rad(37.2), deg2rad(-17.2)];

p_target = robot_modified.fkine(theta_target);
q = rad2deg(robot_modified.ikine(p_target));

%% 
%% 正逆运动学均可调用自己编写库

N = 400;
x_target = p_target.t';
x_ = zeros(1,3);
e = zeros(1,6);
eplot = zeros(1,N);
theta = zeros(1,robot_modified.n);

use_Pseudo = 1;
for j = 1 : N
    %平移    
    p = fkine(theta, robot_modified.n, robot_modified.alpha, robot_modified.a, robot_modified.d);
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
    Jaco = jacobe(theta, robot_modified.n, robot_modified.alpha, robot_modified.a, robot_modified.d);
    if (use_Pseudo)
%         dq = Jaco' * inv(Jaco * Jaco') * e';
        dq = pinv(Jaco) * e';
    else
        dq = Jaco' * e';
    end
    
    dq = 0.005 * dq;

    theta = theta + dq';
end 

thetadeg = rad2deg(theta);
%% 

figure();
plot(eplot)