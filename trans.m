function[T_XY] = trans(theta,k,alpha,a,d)

T_XY = eye(4);
method = 0;

%% 1为改进法，其余标准法
if method == 1
    T_XY_1 = cos(theta(k));
    T_XY_2 = -sin(theta(k));
    T_XY_3 = 0;
    T_XY_4 = a(k);
    
    T_XY_5 = sin(theta(k)) * cos(alpha(k));
    T_XY_6 = cos(theta(k)) * cos(alpha(k));
    T_XY_7 = -sin(alpha(k));
    T_XY_8 = -sin(alpha(k)) * d(k);
    
    T_XY_9 = sin(theta(k)) * sin(alpha(k));
    T_XY_10 = cos(theta(k)) * sin(alpha(k));
    T_XY_11 = cos(alpha(k));
    T_XY_12 = cos(alpha(k)) * d(k);
else 
    T_XY_1 = cos(theta(k));
    T_XY_2 = -sin(theta(k)) * cos(alpha(k));
    T_XY_3 = sin(theta(k)) * sin(alpha(k));
    T_XY_4 = cos(theta(k)) * a(k);
    
    T_XY_5 = sin(theta(k));
    T_XY_6 = cos(theta(k)) * cos(alpha(k));
    T_XY_7 = -cos(theta(k)) * sin(alpha(k));
    T_XY_8 = sin(theta(k)) * a(k);
    
    T_XY_9 = 0;
    T_XY_10 = sin(alpha(k));
    T_XY_11 = cos(alpha(k));
    T_XY_12 = d(k);   
end

T_XY = [T_XY_1  T_XY_2  T_XY_3  T_XY_4;
    T_XY_5  T_XY_6  T_XY_7  T_XY_8;
    T_XY_9  T_XY_10 T_XY_11 T_XY_12;
    0       0       0       1];

end
