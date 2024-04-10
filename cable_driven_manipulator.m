close all,
clear,clc;

%% standard DH parameters, revolute, d, a, alhpa, offset

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
% cable_driven.display();  
theta_target = zeros(1,11);
cable_driven.teach(theta_target);

%% 蒙特卡洛法计算工作空间
N_f = 1; %30000
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

% theta_target = theta_f;
% % cable_driven.teach(theta_target);
% 
% p_target = fkine(theta_target, cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d, cable_driven.offset);
% [theta,eplot,e_final] = c_ikine(cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d,...
%     cable_driven.offset, p_target, 0.0001, 500, 3);
% theta_indeg = rad2deg(theta);

% 验证雅可比矩阵正确性
% Jaco = jacobe(theta_target, cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d, cable_driven.offset);
% j_t = jacob0(cable_driven,theta_target);
% cable_driven.teach(theta);
%% 轨迹跟踪

p_circle = zeros(3,100);
e_circle = zeros(1,100);
theta_circle = zeros(7,100);
for i = 1:100
    p_circle(1,i) = 0.6; 
    p_circle(2,i) = 0.3 * sin(pi/50*i) - 0.2; 
    p_circle(3,i) = 0.3 * cos(pi/50*i); 
    
    p_16 = [1 0 0 p_circle(1,i);
            0 1 0 p_circle(2,i);
            0 0 1 p_circle(3,i)
            0 0 0 1];
        
%     [theta,eplot,e_final] = c_ikine(cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d,...
%         cable_driven.offset, p_16, 0.0001, 500, 3);
%     e_circle(i) = e_final;
%     
%     theta_circle(1,i) = rad2deg(theta(2));
%     theta_circle(2,i) = rad2deg(theta(3));
%     theta_circle(3,i) = rad2deg(theta(4));
%     theta_circle(4,i) = rad2deg(theta(5));
%     theta_circle(5,i) = rad2deg(theta(7));
%     theta_circle(6,i) = rad2deg(theta(8));
%     theta_circle(7,i) = rad2deg(theta(11));
end

    [theta,eplot,e_final] = c_ikine(cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d,...
        cable_driven.offset, p_16, 0.0001, 200, 1);
    cable_driven.teach(theta);
%     
%         [theta,eplot2,e_final] = c_ikine(cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d,...
%         cable_driven.offset, p_16, 0.0001, 200, 1);

%% 图形输出

% figure();
% plot(eplot)
% hold on
% plot(eplot2)
% figure();
% scatter3(x_f(1,:),x_f(2,:),x_f(3,:),3)
% figure();
% scatter3(p_circle(1,:),p_circle(2,:),p_circle(3,:),3)

% figure();
% for i = 1:7
% plot(theta_circle(i,:))
% hold on;
% end

figure();
plot(e_circle)

