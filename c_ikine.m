function[Jaco,j,theta,eplot,e_final] = c_ikine(n,alpha,a,d,offset,p_target,max_tol,max_iter,method)

x_target = zeros(1,3);
x_ = zeros(1,3);

e = ones(1,6);
eplot = zeros(1,max_iter);

theta = zeros(1,n);
% theta(5) = pi/8;
% theta(5) = pi/8;

local_opt = zeros(n,max_iter);

% gpm_k = 0.015 * ones(1,n);
lamda = 0.24;

x_target(1) = p_target(1,4);
x_target(2) = p_target(2,4);
x_target(3) = p_target(3,4);

j = 0;

while (norm(e) > max_tol) % 0.0001
    j = j + 1;
    
    if(j > max_iter)
        j = max_iter;
        break
    end
    
    %平移    
    p = fkine(theta, n, alpha, a, d, offset);
    x_(1) = p(1,4);
    x_(2) = p(2,4);
    x_(3) = p(3,4);
    e(1:3) = x_target - x_;

    %旋转
    Rq = t2r(p);
    [thn,V] = tr2angvec(t2r(p_target) * Rq');
    e(4:6) = thn * V;
    e(4:6) = 0;
    eplot(j) = norm(e);
   
   %雅可比矩阵计算
    Jaco = jacobe(theta, n, alpha, a, d, offset);
%     Jaco(:,1) = 0;
%     Jaco = jacob0(robot,theta); % robotics tool 
    JtJ = Jaco * Jaco';
    
    %迭求解逆运动学
    if (method == 1) % Pseudo_inverse
        Jaco_pinv = Jaco' * pinv(JtJ);
        dq = Jaco_pinv * e';
    elseif(method == 2) % DLS
        Jaco_dls = Jaco' * pinv(JtJ + lamda * lamda * eye(6));
        dq = Jaco_dls * e';
    elseif(method == 3) % GPM
%         Jaco_dls = Jaco' * pinv(JtJ);   
      Jaco_dls = Jaco' * pinv(JtJ + lamda * lamda * eye(6));
            
        % 将theta5限制在0到pi/2中，梯度下降法若要使H最小，可在梯度取负值或步长取负值
        gradient_H = [0,0,-(theta(3)-0)/pi/pi*4,0,-3*(theta(5)-pi/4)/pi/pi*4,0,0,... 
            -(theta(8)-0)/pi/pi*4,0,0,0];
%         H3 = pi*pi/4*(2*theta(3))/4/(pi/4-theta(3))^2/(theta(3)+pi/4)^2;
%         H5 = pi*pi/4*(2*theta(5)-pi/4-0)/4/(pi/2-theta(5))^2/(theta(5)-0)^2;
%         H8 = pi*pi/4*(2*theta(8))/4/(pi/4-theta(8))^2/(theta(8)+pi/4)^2;
%         gradient_H = [0,0,-H3,0,-H5,0,0,-H8,0,0,0];
%         gpm_k = 0.015;
        gpm_k = lamda * norm(Jaco_dls * e') / norm((eye(11) - Jaco_dls * Jaco) * gradient_H');
        local_opt(:,j) = gpm_k * (eye(n) - Jaco_dls * Jaco) * gradient_H';
        dq = Jaco_dls * e' + local_opt(:,j);
    else
        dq = Jaco' * e';
    end

    dq(1) = 0;
    dq(6) = dq(5);
    dq(9) = dq(8);
    dq(10) = -dq(7);
    
    theta = theta + 0.25 * dq';

end    
    e_final = eplot(j);
end