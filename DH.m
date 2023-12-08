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

%% 正运动学手搓好使

% alpha = [0 -pi/2 0 -pi/2 pi/2 -pi/2];
% d = [0 0 0 0.338 0 0];
% a = [0 -0.03 0.34 -0.04 0 0];
% T06 = eye(4);

%     for k = 1:6
%         T_XY_1 = cos(theta(k));
%         T_XY_2 = -sin(theta(k));
%         T_XY_3 = 0;
%         T_XY_4 = a(k);
%         T_XY_5 = sin(theta(k)) * cos(alpha(k));
%         T_XY_6 = cos(theta(k)) * cos(alpha(k));
%         T_XY_7 = -sin(alpha(k));
%         T_XY_8 = -sin(alpha(k)) * d(k);
%         T_XY_9 = sin(theta(k)) * sin(alpha(k));
%         T_XY_10 = cos(theta(k)) * sin(alpha(k));
%         T_XY_11 = cos(alpha(k));
%         T_XY_12 = cos(alpha(k)) * d(k);
% 
%         T_XY = [T_XY_1  T_XY_2  T_XY_3  T_XY_4;
%                 T_XY_5  T_XY_6  T_XY_7  T_XY_8;
%                 T_XY_9  T_XY_10 T_XY_11 T_XY_12;
%                 0       0       0       1];
% 
%         T06 = T06 * T_XY;
%     end
%% 逆运动学中雅可比矩阵求解较为复杂，暂时调用Robotics Toolbox

N = 1000;
x_target = p_target.t';
x_ = zeros(1,3);
e = zeros(1,6);
eplot = zeros(1,N);
theta = [0, 0, 0, 0, 0, 0];

for j = 1 : N
    %平移    
    p = robot_modified.fkine(theta);
    x_(1:3) = p.t';
    e(1:3) = x_target - x_;
    
    %旋转
    Rq = t2r(p);
    rrr = t2r(p_target) * Rq';
    [thn,V] = tr2angvec(t2r(p_target) * Rq');
    e(4:6) = thn * V;
    eplot(j) = e * e';
   
    Jaco = jacob0(robot_modified, theta);
    dq = Jaco' * e';
    dq = 0.01 * dq;

    theta = theta + dq';
end 

thetadeg = rad2deg(theta);
%% 

figure();
plot(eplot)
