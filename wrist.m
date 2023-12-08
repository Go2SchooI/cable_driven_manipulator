wc = 30;
lc = 85.75;
ho = 4.84;
hc = (lc * lc - wc * wc) ^ 0.5;

N = 1000;
phi = zeros(1,N);
r = zeros(1,N);
numerator = zeros(1,N);
denominator = zeros(1,N);
value1 = zeros(1,N);
value2 = zeros(1,N);
value3 = zeros(1,N);

for k = 1:N
     phi(k) = k * 45 / N;
%     value1 = ho * ho;
%     value2 = 1 + wc * wc / hc / hc * tan(phi(k)) * tan(phi(k));
%     value3 = wc * wc / 4 - ho * ho;
%     numerator = (value1 + value2 * value3)^0.5 + ho;
% %     numerator = ho + (ho * ho + (1 + wc * wc / hc / hc * tan(phi(k)) * tan(phi(k))) * (wc * wc / 4 - ho * ho))^0.5;
%     denominator = cos(phi(k)) * (1 + wc * wc / hc / hc * tan(phi(k)) * tan(phi(k)));
    
    value1(k) = hc^4/4/lc/lc*tan(deg2rad(phi(k)))^2;
    value2(k) = hc^2/4;
    value3(k) = hc^2*ho^2/lc/lc*tan(deg2rad(phi(k)))^2;
    numerator(k) = (value1(k) + value2(k) - value3(k))^0.5 + ho;
    denominator(k) = cos(deg2rad(phi(k))) * (1 + hc^2/lc/lc * tan(deg2rad(phi(k)))^2);

    r(k) = numerator(k) / denominator(k);
end

figure();
plot(phi,r)