close all,
clear,clc;

%% Modified DH parameters
% 
% L1 =  Link('revolute', 'd', 0.36, 'a', 0, 'aplpa', 0, 'modified');
% L2 =  Link('revolute', 'd', 0, 'a', 0, 'aplpa', -pi/2, 'modified');
% L3 =  Link('revolute', 'd', 0.42, 'a', 0, 'aplpa', pi/2, 'modified');
% L4 =  Link('revolute', 'd', 0, 'a', 0, 'aplpa', -pi/2, 'modified');
% L5 =  Link('revolute', 'd', 0.4, 'a', 0, 'aplpa', pi/2, 'modified');
% L6 =  Link('revolute', 'd', 0, 'a', 0, 'aplpa', -pi/2, 'modified');
% L7 =  Link('revolute', 'd', 0.126, 'a', 0, 'aplpa', pi/2, 'modified');
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

p_target = robot_modified.fkine(theta_target);
q = rad2deg(robot_modified.ikine(p_target));

%% 正逆运动学均可调用自己编写库

N = 50;
x_target = p_target.t';
x_ = zeros(1,3);
e = zeros(1,6);
eplot = zeros(1,N);
theta = zeros(1,robot_modified.n);

use_Pseudo = 1;
use_DLS = 0;
use_GPM = 1;

gpm_k = zeros(1,N);
lamda = 0.1;

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
    [thn,V] = tr2angvec(t2r(p_target) * Rq');
    e(4:6) = thn * V;
    eplot(j) = norm(e);
   
%     Jaco = jacob0(robot_modified, theta);
    Jaco = jacobe(theta, robot_modified.n, robot_modified.alpha, robot_modified.a, robot_modified.d);
    JtJ = Jaco * Jaco';
    
    if (use_Pseudo)
        if(use_DLS)
            Jaco_dls = Jaco' * pinv(JtJ + lamda * lamda * eye(6));
            dq = Jaco_dls * e';
        elseif(use_GPM)
            % assume theta4 has max as 90 and min as 60
            Jaco_pinv = Jaco' * pinv(JtJ);   
            gradient_H = [0 0 0 -(rad2deg(theta(4))-75)/900 0 0 0];
            gpm_k(j) = lamda * norm(Jaco_pinv * e') / norm((eye(7) - Jaco_pinv * Jaco) * gradient_H');
            local_opt = gpm_k(j) * (eye(7) - Jaco_pinv * Jaco) * gradient_H';
            dq = Jaco_pinv * e' + local_opt;
        else
            Jaco_pinv = Jaco' * pinv(JtJ);
            dq = Jaco_pinv * e';
        end
    else
        dq = Jaco' * e';
    end
    
    dq = 1 * dq;

    theta = theta + dq';
end 

thetadeg = rad2deg(theta);
robot_modified.teach(theta);
%% 

figure();
plot(eplot)