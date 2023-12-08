%% %% 正运动学手搓好使
function[T06] = fkine(theta)
alpha = [0 -pi/2 0 -pi/2 pi/2 -pi/2];
d = [0 0 0 0.338 0 0];
a = [0 -0.03 0.34 -0.04 0 0];
T06 = eye(4);

for k = 1:6
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
    
    T_XY = [T_XY_1  T_XY_2  T_XY_3  T_XY_4;
        T_XY_5  T_XY_6  T_XY_7  T_XY_8;
        T_XY_9  T_XY_10 T_XY_11 T_XY_12;
        0       0       0       1];
    
    T06 = T06 * T_XY;
end
    
end