clear;
clc;

N = 1000;
d0 = 25;
d = sqrt(4*4+32*32);
phi = atan2(1,8);
n = 4;

theta6 = zeros(N,N);
theta7 = zeros(N,N);
dL = zeros(N,N);
% dtheta_mo = zeros(N,N);

for i = 1:N
    for j = 1:N
        theta6(j,i) = -pi + 2 * pi / N * (i-1);
        theta7(j,i) = -pi/2 + pi / N * (i-1);
%         dL(k,j) = d * n * sin(phi-deg2rad(theta6(k,j))) * sin(deg2rad(theta7(k,j)/2));
%         dtheta_mo(j) = rad2deg(2 * n * d / d0 * sin(phi-deg2rad(theta6(j)));
    end
end

for i = 1:N
    for j = 1:N
        dL(j,i) = d * n * sin(phi-theta6(i,j)) * sin(theta7(i,i)/2);
    end
end

figure;
surf(theta6, theta7, dL);

figure;
[x, y] = meshgrid(-4:0.2:4);
z = x.^2 + y.^2;
figure
surf(x, y, z);

