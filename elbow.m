clear;
clc;

N = 1000;
d0 = 25;
d = 70;
n = 4;

theta_el = zeros(N);
dtheta_mo = zeros(N);
dL = zeros(N);
dl_linear = zeros(N);

for j = 1:N
    theta_el(j) = 180 / N * (j-1);
    dL(j) = d * n * sin(deg2rad(theta_el(j)/2 - 45));
    dtheta_mo(j) = rad2deg(2 * n * d / d0 * sin(deg2rad(theta_el(j)/2 - 45)));
end

a = (dtheta_mo(N) - dtheta_mo(1)) / 180;
b = dtheta_mo(1);
dm_linear = a * theta_el + b;

figure(1)
hold on
plot(theta_el,dtheta_mo,'linewidth',1.5);
plot(theta_el,dm_linear,'r--','linewidth',1.5);
hold off
legend('电机角度变化','角度线性变化')
xlabel('肘部关节角度（°）')
ylabel('肘部驱动电机角度（°）')
grid on


figure(2);
plot(theta_el,dtheta_mo-dm_linear,'linewidth',1.5);

