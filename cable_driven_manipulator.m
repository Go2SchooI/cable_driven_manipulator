close all,
clear,clc;

%% standard DH parameters, revolute, d, a, alhpa, offset

L0 =  Link('revolute', 'd',0,     'a',0,    'alpha',pi/2,   'offset',0,     'standard'); %使机械臂架构与sw相同
L1 =  Link('revolute', 'd',0.143, 'a',0,    'alpha',pi/2,   'offset',pi/2,  'standard');
L2 =  Link('revolute', 'd',0.078, 'a',0,    'alpha',-pi/2,  'offset',pi/2,  'standard');
L3 =  Link('revolute', 'd',0.291, 'a',0,    'alpha',pi/2,   'offset',pi/2,  'standard');

L4 =  Link('revolute', 'd',0,     'a',0.07, 'alpha',0,      'offset',pi/2,  'standard');
L5 =  Link('revolute', 'd',0,     'a',0,    'alpha',-pi/2,  'offset',-pi/2, 'standard');

% L6 =  Link('revolute', 'd',0.237,  'a',0,    'alpha',pi/2,   'offset',0,     'standard');
L6 =  Link('revolute', 'd',0.237,  'a',0,    'alpha',pi/2,   'offset',-pi/2,     'standard');
L7 =  Link('revolute', 'd',0,     'a',0.095,'alpha',0,      'offset',pi/2,  'standard');
L8 =  Link('revolute', 'd',0,     'a',0,    'alpha',-pi/2,  'offset',-pi/2, 'standard');
L9 =  Link('revolute', 'd',0.04,  'a',0,    'alpha',0,      'offset',0,     'standard');

L10=  Link('revolute', 'd',0,     'a',0,    'alpha',0,      'offset',0,     'standard');

cable_driven = SerialLink([L0,L1,L2,L3,L4,L5,L6,L7,L8,L9,L10], 'name', 'cdm');  
theta_target = zeros(1,11);
cable_driven.teach(theta_target);

%% 蒙特卡洛法计算工作空间
% N_f = 1; %30000
% x_f = zeros(3,N_f);
% theta_f = zeros(1,cable_driven.n);
% 
% for k = 1:N_f
%     for i = 1:cable_driven.n
%         theta_f(i) = -pi + 2 * pi * rand();
%     end
%     
%     theta_f(1) = 0;
%     theta_f(3) = - pi/4 + 2 * pi/4 * rand();
%     
%     theta_f(5) = pi/2 * rand();
%     theta_f(6) = theta_f(5);
%   
%     theta_f(8) = - pi/4 + 2 * pi/4 * rand();
%     theta_f(9) = theta_f(8);
%     theta_f(10) = -theta_f(7);
%     
%     p = fkine(theta_f, cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d, cable_driven.offset);
%     x_f(1,k) = p(1,4);
%     x_f(2,k) = p(2,4);
%     x_f(3,k) = p(3,4);
% end
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

% p_circle = zeros(3,100);
% e_circle = zeros(1,100);
% iter_circle = zeros(1,100);
% theta_circle = zeros(7,100);
% theta = zeros(1,cable_driven.n);
% 
% plot_theta = zeros(100,cable_driven.n);
% plot_s = zeros(6,100);
% 
% 
% for i = 1:100
%     p_circle(1,i) = 0.6; 
%     p_circle(2,i) = 0.4 * sin(pi/50*i) - 0.2; 
%     p_circle(3,i) = 0.4 * cos(pi/50*i); 
%     
%     p_16 = [1 0 0 p_circle(1,i);
%             0 1 0 p_circle(2,i);
%             0 0 1 p_circle(3,i)
%             0 0 0 1];
%         
%     [jaco,iter,theta,eplot,e_final] = c_ikine(cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d,...
%         cable_driven.offset,p_16, 0.00003,650,3);
%     e_circle(i) = e_final;
%     iter_circle(i) = iter;
%     plot_theta(i,:) = theta;
%     
%     theta_circle(1,i) = rad2deg(theta(2));
%     theta_circle(2,i) = rad2deg(theta(3));
%     theta_circle(3,i) = rad2deg(theta(4));
%     theta_circle(4,i) = 2 * rad2deg(theta(5));
%     theta_circle(5,i) = rad2deg(theta(7));
%     theta_circle(6,i) = rad2deg(theta(8));
%     theta_circle(7,i) = rad2deg(theta(11));
%     
% %     测试
%     jacobian = jacob0(cable_driven,theta);
%     plot_s(:,i) = svd(jacobian);
% end

%     [theta,eplot,e_final] = c_ikine(cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d,...
%         cable_driven.offset, p_16, 0.00005, 500, 2);
%     cable_driven.teach(theta);
%     
%         [theta,eplot2,e_final] = c_ikine(cable_driven.n, cable_driven.alpha, cable_driven.a, cable_driven.d,...
%         cable_driven.offset, p_16, 0.0001, 200, 1);

%% 图形输出
% 收敛速度对比
% figure();
% plot(eplot)
% hold on
% plot(eplot2)

% 工作空间
% figure();
% scatter3(x_f(1,:),x_f(2,:),x_f(3,:),3)

% 轨迹跟踪
% cable_driven.teach(theta);
% hold on;
% scatter3(p_circle(1,:),p_circle(2,:),p_circle(3,:),3)
% hold off;
% cable_driven.plot(plot_theta,'trail','r');
% % cable_driven.plot(plot_theta,'trail','r','movie','trail.gif');
% xlabel('X(m)')
% ylabel('Y(m)')
% zlabel('Z(m)')

% 轨迹展示
% figure();
% scatter3(p_circle(1,:),p_circle(2,:),p_circle(3,:),3)
% xlabel('X(m)')
% ylabel('Y(m)')
% zlabel('Z(m)')

% % 关节角度
% figure();
% for i = 1:7
%     if(i == 2 || i == 4 || i == 6)
%         plot(theta_circle(i,:), 'linewidth',1.75)  
%     else
% plot(theta_circle(i,:))
%     end
% hold on;
% end
% legend('肩部1','肩部2','肩部3','肘部','腕部回转','腕部俯仰','腕部末端')
% xlabel('跟踪轨迹点数n')
% ylabel('关节角度（°）')
% grid on;
% 
% % 末端误差
% figure();
% plot(e_circle)
% xlabel('跟踪轨迹点数n')
% ylabel('末端位置误差（m）')
% grid on;
% 
% figure();
% plot(iter_circle)