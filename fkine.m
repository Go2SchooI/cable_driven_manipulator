%% %% 正运动学手搓好使
function[T06] = fkine(theta)
T06 = eye(4);

for k = 1:6
    T06 = T06 * trans(theta(k),k);
end
    
end