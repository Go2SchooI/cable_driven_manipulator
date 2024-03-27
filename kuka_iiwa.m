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

%% 正逆运动学均可调用自己编写库
N = 20;
j = 0;

x_target = p_target.t';
x_ = zeros(1,3);

e = ones(1,6);
eplot = zeros(1,N);
dqplot = zeros(1,N);

theta = zeros(1,robot_modified.n);
local_opt = zeros(7,N);

use_Pseudo = 1;
use_DLS = 0;
use_GPM = 1;

% gpm_K = ones(1,N);
lamda = 0.3;

while (norm(e) > 0.0001)
    j = j + 1;
    
    if(j > N)
        break
    end
    
    %平移    
    p = fkine(theta, robot_modified.n, robot_modified.alpha, robot_modified.a, robot_modified.d);
    x_(1) = p(1,4);
    x_(2) = p(2,4);
    x_(3) = p(3,4);
    e(1:3) = x_target - x_;
    
    %旋转
    Rq = t2r(p);
    [thn,V] = tr2angvec(t2r(p_target) * Rq');
    e(4:6) = thn * V;
    eplot(j) = norm(e);
   
   %雅可比矩阵计算
%     Jaco = jacob0(robot_modified, theta);
    Jaco = jacobe(theta, robot_modified.n, robot_modified.alpha, robot_modified.a, robot_modified.d);
    JtJ = Jaco * Jaco';
    
    %迭求解逆运动学
    if (use_Pseudo)
        if(use_DLS)
            Jaco_dls = Jaco' * pinv(JtJ + lamda * lamda * eye(6));
            dq = Jaco_dls * e';
        elseif(use_GPM)
%             Jaco_pinv = Jaco' * pinv(JtJ);   
            Jaco_pinv = Jaco' * pinv(JtJ + lamda * lamda * eye(6));
            
            % assume theta5 has max as pi/2 and min as pi/3
            gradient_H = [0 0 0 0 -(theta(5)-5/12*pi)/pi/pi*36 0 0];
            gpm_K = 0.1;
%              gpm_k = lamda * norm(Jaco_pinv * e') / norm((eye(7) - Jaco_pinv * Jaco) * gradient_H');
            local_opt(:,j) = gpm_K * (eye(7) - Jaco_pinv * Jaco) * gradient_H';
            dq = Jaco_pinv * e' + local_opt(:,j);
        else
            Jaco_pinv = Jaco' * pinv(JtJ);
            dq = Jaco_pinv * e';
        end
    else
        dq = Jaco' * e';
    end

    dqplot(j) = norm(dq);
    theta = theta + 0.2 * dq';
end 

thetadeg = rad2deg(theta);
robot_modified.teach(theta);
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
%% 图形输出
% 
% figure();
% scatter3(x_f(1,:),x_f(2,:),x_f(3,:),5)
% figure();
% plot(eplot)
% figure();
% plot(dqplot)