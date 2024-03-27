close all,
clear,clc;

%% standard DH parameters, revolute, d, a, alhpa, offset
L1 =  Link([ 0,    0.36,   0,       0,       0], 'standard');
L2 =  Link([ 0,    0,      0,      -pi/2,    0], 'standard');
L3 =  Link([ 0,    0.42,   0,       pi/2,    0], 'standard');
L4 =  Link([ 0,    0,      0,      -pi/2,    0], 'standard');
L5 =  Link([ 0,    0.4,    0,       pi/2,    0], 'standard');
L6 =  Link([ 0,    0,      0,      -pi/2,    0], 'standard');
L7 =  Link([ 0,    0.126,  0,       pi/2,    0], 'standard');
cable_driven = SerialLink([L1,L2,L3,L4,L5,L6,L7]); 
cable_driven.display();  

theta_target = [deg2rad(19.8), deg2rad(-33.6), deg2rad(-13.6), deg2rad(7.2), deg2rad(-17.2), deg2rad(37.2), deg2rad(-17.2)];
cable_driven.teach(theta_target);

%% 正逆运动学均可调用自己编写库

p_target = fkine(theta_target, cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d);
[theta,eplot] = (ikine(cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d,...
    p_target, 0.0001, 3, 20));

cable_driven.teach(theta);

%% 图形输出

figure();
plot(eplot)
%% 蒙特卡洛法计算工作空间
% 
% x_f = zeros(3,50000);
% theta_f = zeros(1,cable_driven.n);
% 
% for k = 1:50000
%     for i = 1:7
%      theta_f(i) = -pi + 2 * pi * rand();
%     end
%     p = fkine(theta_f, cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d);
%     x_f(1,k) = p(1,4);
%     x_f(2,k) = p(2,4);
%     x_f(3,k) = p(3,4);
% end
