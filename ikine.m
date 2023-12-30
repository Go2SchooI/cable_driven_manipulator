function[thetadeg, eplot] = ikine(p_target)

N = 100;
x_ = zeros(1,3);
e = zeros(1,6);
eplot = zeros(1,N);
theta = [0, 0, 0, 0, 0, 0];

use_Pseudo = 0;

for j = 1 : N
    %平移    
    p = fkine(theta);
    x_(1) = p(1,4);
    x_(2) = p(2,4);
    x_(3) = p(3,4);
    e(1:3) = p_target - x_;
    
    %旋转
    Rq = t2r(p);
    rrr = t2r(p_target) * Rq';
    [thn,V] = tr2angvec(t2r(p_target) * Rq');
    e(4:6) = thn * V;
    eplot(j) = e * e';
   
    Jaco = jacobe(theta);

    if (use_Pseudo)
        dq = Jaco' \ (Jaco * Jaco') * e';
    else
        dq = Jaco' * e';
    end
    
    dq = 0.05 * dq;

    theta = theta + dq';
end 

thetadeg = rad2deg(theta);

end