function[T0n] = fkine(theta_t,n,alpha,a,d,offset)

T0n = eye(4);

theta = theta_t + offset;
for k = 1:n
    T0n = T0n * trans(theta,k,alpha,a,d);
end
    
end