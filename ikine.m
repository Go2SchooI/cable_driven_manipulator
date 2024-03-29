function[theta,eplot] = ikine(n,alpha,a,d,p_target,max_tol,method,max_iter)

x_target = zeros(1,3);
x_ = zeros(1,3);

e = ones(1,6);
eplot = zeros(1,max_iter);

theta = zeros(1,n);
local_opt = zeros(7,max_iter);

% gpm_k = ones(1,max_iter);
lamda = 0.3;

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
    p = fkine(theta, n, alpha, a, d);
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
    Jaco = jacobe(theta, n, alpha, a, d);
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
%        Jaco_pinv = Jaco' * pinv(JtJ + lamda * lamda * eye(6));
            
         % assume theta5 has max as pi/2 and min as pi/3
         gradient_H = [0 0 0 0 -(theta(5)-5/12*pi)/pi/pi*36 0 0];
         gpm_k = 0.1;
%        gpm_k = lamda * norm(Jaco_pinv * e') / norm((eye(7) - Jaco_pinv * Jaco) * gradient_H');
         local_opt(:,j) = gpm_k * (eye(7) - Jaco_pinv * Jaco) * gradient_H';
         dq = Jaco_pinv * e' + local_opt(:,j);
    else
        dq = Jaco' * e';
    end

    theta = theta + 0.2 * dq';
end 

end