function[theta,eplot,e_final] = c_ikine(n,alpha,a,d,offset,p_target,max_tol,max_iter,method)

x_target = zeros(1,3);
x_ = zeros(1,3);

e = ones(1,6);
eplot = zeros(1,max_iter);

theta = zeros(1,n);
local_opt = zeros(n,max_iter);

% gpm_k = ones(1,max_iter);
lamda = 0.15;

x_target(1) = p_target(1,4);
x_target(2) = p_target(2,4);
x_target(3) = p_target(3,4);

j = 0;

while (norm(e) > max_tol) % 0.0001
    j = j + 1;
    
    if(j > max_iter)
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
        Jaco_pinv = Jaco' * pinv(JtJ);   
%       Jaco_dls = Jaco' * pinv(JtJ + lamda * lamda * eye(6));
            
        % 将theta5限制在0到pi/2中，梯度下降法若要使H最小，可在梯度取负值或步长取负值
        gradient_H = [0,0,0,0,-(theta(5)-pi/4)/pi/pi*4,0,0,... 
            -(theta(8)-0)/pi/pi*4,0,0,0];
        gpm_k = 0.015;
%       gpm_k = lamda * norm(Jaco_pinv * e') / norm((eye(7) - Jaco_pinv * Jaco) * gradient_H');
        local_opt(:,j) = gpm_k * (eye(n) - Jaco_pinv * Jaco) * gradient_H';
        dq = Jaco_pinv * e' + local_opt(:,j);
    else
        dq = Jaco' * e';
    end

    dq(1) = 0;
    dq(6) = dq(5);
    dq(9) = dq(8);
    dq(10) = -dq(7);
    
    theta = theta + 0.15 * dq';

end    
    e_final = eplot(j-1);
end