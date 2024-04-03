close all,
clear,clc;

%% standard DH parameters, revolute, d, a, alhpa, offset
% L1 =  Link([ 0,    0.177,   0,      pi/2,    0], 'standard');
% L2 =  Link([ 0,    0.127,   0,      pi/2,    0], 'standard');
% L3 =  Link([ 0,    0.293,   0,      pi/2,    0], 'standard');
% L4 =  Link([ 0,    0,       0.07,   0,       0], 'standard');
% L5 =  Link([ 0,    0,       0.237,  0,       0], 'standard');
% L6 =  Link([ 0,    0,       0,     -pi/2,    0], 'standard');
% L7 =  Link([ 0,    0,       0.095,  0,       0], 'standard');
% L8 =  Link([ 0,    0,       0,      pi/2,    0], 'standard');
% L9 =  Link([ 0,    0,       0,      pi/2,    0], 'standard');
% L10=  Link([ 0,    0.06,    0,      0,       0], 'standard');

L0 =  Link('revolute', 'd',0,     'a',0,    'alpha',pi/2,   'offset',0,     'standard'); %使机械臂架构与sw相同
L1 =  Link('revolute', 'd',0.197, 'a',0,    'alpha',pi/2,   'offset',pi/2,  'standard');
L2 =  Link('revolute', 'd',0.122, 'a',0,    'alpha',-pi/2,  'offset',pi/2,  'standard');
L3 =  Link('revolute', 'd',0.293, 'a',0,    'alpha',pi/2,   'offset',pi/2,  'standard');

L4 =  Link('revolute', 'd',0,     'a',0.07, 'alpha',0,      'offset',pi/2,  'standard');
L5 =  Link('revolute', 'd',0,     'a',0,    'alpha',-pi/2,  'offset',-pi/2, 'standard');

L6 =  Link('revolute', 'd',0.24,  'a',0,    'alpha',pi/2,   'offset',0,     'standard');
L7 =  Link('revolute', 'd',0,     'a',0.095,'alpha',0,      'offset',pi/2,  'standard');
L8 =  Link('revolute', 'd',0,     'a',0,    'alpha',-pi/2,  'offset',-pi/2, 'standard');
L9 =  Link('revolute', 'd',0,     'a',0,    'alpha',0,      'offset',0,     'standard');

L10=  Link('revolute', 'd',0.04,  'a',0,    'alpha',0,      'offset',0,     'standard');

cable_driven = SerialLink([L0,L1,L2,L3,L4,L5,L6,L7,L8,L9,L10], 'name', 'cdm'); 
cable_driven.display();  
theta_target = zeros(1,11);
cable_driven.teach(theta_target);

%% 蒙特卡洛法计算工作空间
N_f = 20000;
x_f = zeros(3,N_f);
theta_f = zeros(1,cable_driven.n);

for k = 1:N_f
    for i = 1:cable_driven.n
        theta_f(i) = -pi + 2 * pi * rand();
    end
    
    theta_f(1) = 0;
    
    theta_f(5) = pi/2 * rand();
    theta_f(6) = theta_f(5);
  
    theta_f(8) = - pi/4 + 2 * pi/4 * rand();
    theta_f(9) = theta_f(8);
    theta_f(10) = -theta_f(7);
    
    p = fkine(theta_f, cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d, cable_driven.offset);
    x_f(1,k) = p(1,4);
    x_f(2,k) = p(2,4);
    x_f(3,k) = p(3,4);
end

%% 正逆运动学均可调用自己编写库

theta_target = theta_f;
% cable_driven.teach(theta_target);
p_target = fkine(theta_target, cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d, cable_driven.offset);
[theta,eplot] = (c_ikine(cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d,...
    cable_driven.offset, p_target, 0.0001, 2, 100));

% cable_driven.teach(theta);

%% 图形输出

figure();
plot(eplot)
figure();
scatter3(x_f(1,:),x_f(2,:),x_f(3,:),3)