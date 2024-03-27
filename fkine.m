function[T0n] = fkine(theta,n,alpha,a,d)

T0n = eye(4);
for k = 1:n
    T0n = T0n * trans(theta,k,alpha,a,d);
end
    
end