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
% robot_modified.teach([0 0 0 0 0 0 0]);

theta_target = [deg2rad(19.8), deg2rad(-33.6), deg2rad(-13.6), deg2rad(7.2), deg2rad(-17.2), deg2rad(37.2), deg2rad(-17.2)];
robot_modified.teach(theta_target);

%% 正逆运动学均可调用自己编写库

p_target = fkine(theta_target, robot_modified.n, robot_modified.alpha, robot_modified.a, robot_modified.d);
[theta,eplot] = (ikine(robot_modified.n, robot_modified.alpha, robot_modified.a, robot_modified.d,...
    p_target, 0.0001, 3, 20));

robot_modified.teach(theta);

%% 图形输出

figure();
plot(eplot)
%% 蒙特卡洛法计算工作空间
% 
% x_f = zeros(3,50000);
% theta_f = zeros(1,robot_modified.n);
% 
% for k = 1:50000
%     for i = 1:7
%      theta_f(i) = -pi + 2 * pi * rand();
%     end
%     p = fkine(theta_f, robot_modified.n, robot_modified.alpha, robot_modified.a, robot_modified.d);
%     x_f(1,k) = p(1,4);
%     x_f(2,k) = p(2,4);
%     x_f(3,k) = p(3,4);
% end
