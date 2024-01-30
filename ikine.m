function[thetadeg, eplot] = ikine(theta,n,alpha,a,d,p_target,max_tol,use_Pseudo,method)

N = 20;
j = 0;

x_target = p_target.t';
x_ = zeros(1,3);

e = ones(1,6);
eplot = zeros(1,N);

theta = zeros(1,robot_modified.n);
local_opt = zeros(7,N);

lamda = 0.1;

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
%     p = robot_modified.fkine(theta);
%     x_(1:3) = p.t';
    e(1:3) = x_target - x_;
    
    %旋转
    Rq = t2r(p);
    [thn,V] = tr2angvec(t2r(p_target) * Rq');
    e(4:6) = thn * V;
    eplot(j) = norm(e);
   
%     Jaco = jacob0(robot_modified, theta);
    Jaco = jacobe(theta, n, alpha, a, d);
    JtJ = Jaco * Jaco';
    
    if (use_Pseudo)
        if(method == 1)
            Jaco_dls = Jaco' * pinv(JtJ + lamda * lamda * eye(6));
            dq = Jaco_dls * e';
        elseif(method == 2)
            Jaco_pinv = Jaco' * pinv(JtJ);   
            % assume theta5 has max as pi/2 and min as pi/3
            gradient_H = [0 0 0 0 -(theta(5)-5/12*pi)/pi/pi*36 0 0];
            gpm_k = 0.1;
%              gpm_k = lamda * norm(Jaco_pinv * e') / norm((eye(7) - Jaco_pinv * Jaco) * gradient_H');
            local_opt(:,j) = gpm_k * (eye(7) - Jaco_pinv * Jaco) * gradient_H';
            dq = Jaco_pinv * e' + local_opt(:,j);
        else
            Jaco_pinv = Jaco' * pinv(JtJ);
            dq = Jaco_pinv * e';
        end
    else
        dq = Jaco' * e';
    end

    theta = theta + dq';
end 

thetadeg = rad2deg(theta);
end