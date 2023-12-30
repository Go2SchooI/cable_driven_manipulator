function[J_] = jacobe(theta,n,alpha,a,d)
p_e = transl(fkine(theta,n,alpha,a,d));
J_ = zeros(6,n);
T_iminus1 = eye(4);
Z_iminus1 = zeros(3,1);
P_iminus1 = zeros(3,1);
J_pi = zeros(3,1);
J_oi = zeros(3,1);

for iminus1 = 1:n
   
    T_iminus1 = T_iminus1 * trans(theta,iminus1,alpha,a,d);
      
    Z_iminus1(1) = T_iminus1(1,3);
    Z_iminus1(2) = T_iminus1(2,3);
    Z_iminus1(3) = T_iminus1(3,3);
    P_iminus1 = transl(T_iminus1);
    
    J_pi = cross(Z_iminus1, p_e - P_iminus1);
    J_oi = Z_iminus1;
    
    J_(1,iminus1) = J_pi(1,1);
    J_(2,iminus1) = J_pi(2,1);
    J_(3,iminus1) = J_pi(3,1);
    J_(4,iminus1) = J_oi(1,1);
    J_(5,iminus1) = J_oi(2,1);
    J_(6,iminus1) = J_oi(3,1);
end

end