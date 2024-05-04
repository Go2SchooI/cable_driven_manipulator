% clear;
% clc;

N = 1000;
d0 = 25;
d = sqrt(4*4+32*32);
phi = atan2(1,8);
n = 4;

% theta6 = zeros(N,N);
% theta7 = zeros(N,N);
% dL = zeros(N,N);
% dtheta_mo = zeros(N,N);

% for i = 1:N
%     for j = 1:N
%         theta6(j,i) = -pi + 2 * pi / N * (i-1);
%         theta7(j,i) = -pi/2 + pi / N * (i-1);
% %         dL(k,j) = d * n * sin(phi-deg2rad(theta6(k,j))) * sin(deg2rad(theta7(k,j)/2));
% %         dtheta_mo(j) = rad2deg(2 * n * d / d0 * sin(phi-deg2rad(theta6(j)));
%     end
% end

% for i = 1:N
%     for j = 1:N
%         dL(j,i) = d * n * sin(phi-theta6(i,j)) * sin(theta7(i,i)/2);
%     end
% end

figure(1);
[theta6, theta7] = meshgrid(-180:6:180,-90:3:90);

temp1 = sin(phi-deg2rad(theta6));
temp2 = sin(deg2rad(theta7/2));

temp1 = temp1(1,:)';
temp2 = temp2(:,1)';

% 通过数值比对发现需要取转置，具体原因待分析
dL1 = (d * n * temp1 * temp2)';
surf(theta6, theta7, dL1);
xlabel('腕部回转角度（°）')
ylabel('腕部俯仰角度（°）')
zlabel('腕部绳索1变化长度（mm）')

figure(2);
[theta6, theta7] = meshgrid(-180:6:180,-90:3:90);

temp3 = cos(phi-deg2rad(theta6));
temp3 = temp3(1,:)';

dL2 = (d * n * temp3 * temp2)';
surf(theta6, theta7, dL2);
xlabel('腕部回转角度（°）')
ylabel('腕部俯仰角度（°）')
zlabel('腕部绳索2变化长度（mm）')

zhat = d * n * cos(phi-deg2rad(72))*sin(deg2rad(-48/2));

% figure(2);
% [x, y] = meshgrid(-4:0.2:4);
% z = x.^2 + y.^2;
% figure
% surf(x, y, z);

