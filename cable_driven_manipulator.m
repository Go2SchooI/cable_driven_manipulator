close all,
clear,clc;

%% standard DH parameters, revolute, d, a, alhpa, offset
L1 =  Link([ 0,    0.177,   0,      pi/2,    0], 'standard');
L2 =  Link([ 0,    0.127,   0,      pi/2,    0], 'standard');
L3 =  Link([ 0,    0.293,   0,      pi/2,    0], 'standard');
L4 =  Link([ 0,    0,       0.07,   0,       0], 'standard');
L5 =  Link([ 0,    0,       0.237,  0,       0], 'standard');
L6 =  Link([ 0,    0,       0,     -pi/2,    0], 'standard');
L7 =  Link([ 0,    0,       0.095,  0,       0], 'standard');
L8 =  Link([ 0,    0,       0,      pi/2,    0], 'standard');
L9 =  Link([ 0,    0,       0,      pi/2,    0], 'standard');
L10=  Link([ 0,    0.06,    0,      0,       0], 'standard');

cable_driven = SerialLink([L1,L2,L3,L4,L5,L6,L7,L8,L9,L10]); 
cable_driven.display();  

%theta_target = [deg2rad(19.8), deg2rad(-33.6), deg2rad(-13.6), deg2rad(7.2), deg2rad(-17.2), deg2rad(37.2), deg2rad(-17.2)];
%for j = 1:10
% theta_target = rand(1,10) * pi/2;
% theta_target(5) = theta_target(4);
% theta_target(8) = theta_target(7);
% cable_driven.teach(theta_target);

%% 正逆运动学均可调用自己编写库

p_target = fkine(theta_target, cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d);
% [theta,eplot] = (ikine(cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d,...
%     p_target, 0.0001, 3, 20));

% cable_driven.teach(theta);

%% 蒙特卡洛法计算工作空间
N_f = 10000;
x_f = zeros(3,N_f);
theta_f = zeros(1,cable_driven.n);

for k = 1:N_f
    for i = 1:cable_driven.n
        theta_f(i) = -pi + 2 * pi * rand();
    end
    theta_f(1) = theta_f(1) + pi/2;
    theta_f(2) = theta_f(2) - pi/2;
    theta_f(3) = theta_f(3) - pi/2;
    theta_f(4) = pi/2 * rand() - pi/2;
    theta_f(5) = theta_f(4);
    theta_f(6) = -pi/3 + 2 * pi/3 * rand();
    theta_f(7) = -pi/3 + 2 * pi/3 * rand();
    theta_f(8) = theta_f(7);
    theta_f(9) = theta_f(6) + pi/2;
        
    p = fkine(theta_f, cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d);
    x_f(1,k) = p(1,4);
    x_f(2,k) = p(2,4);
    x_f(3,k) = p(3,4);
end

%% 图形输出

% figure();
% plot(eplot)
figure();
scatter3(x_f(1,:),x_f(2,:),x_f(3,:),5)
