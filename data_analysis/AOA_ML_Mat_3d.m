function [theta_est,phi_est] = AOA_ML_Mat_3d( pdoa_ob,antenna_index,fc,c,radius,low_threshold_phi,high_threshold_phi)
%% 参数列表
% pdoa_ob观察到的 pdoa
% anteenna_index 选用的天线阵列
% fc 载频 中心频率
% c 光速
% radius 天线阵型半径
% low/high threshold 搜索的高低阈值

antenna_index = [1 2 3 4 5 6 7 8];
low_threshold_phi= 0;
high_threshold_phi = pi;
antenna_num = length(antenna_index);
for i = 1:antenna_num
    k = antenna_index(i);
    alpha(i) = wrapToPi((k-1)*pi/4);
end


theta = 0:0.01:pi/2;
phi = low_threshold_phi : 0.001 : high_threshold_phi;
theta_phi = zeros(length(theta),length(phi),2);

for i = 1:length(phi)
    theta_phi(:,i,1) = theta;
end
for i = 1:length(theta)
    theta_phi(i,:,2) = phi;
end

std_phase = 2 * pi * fc / c * radius .* cos(theta_phi(:,:,2)) .* sin ( theta_phi(:,:,1));

for i = 1:antenna_num
    phase(:,:,i) =    2 * pi * fc / c * radius .* cos(theta_phi(:,:,2) - alpha(i)) .* sin ( theta_phi(:,:,1));
    predict_pdoa(:,:,i) = wrapToPi(phase(:,:,i) - std_phase );
end
pdoa_diff = wrapToPi(predict_pdoa - reshape(pdoa_ob,1,1,8));
pdoa_square = pdoa_diff.* pdoa_diff;
pdoa_sum_square = sum(pdoa_square,3);
sco_ppmin = min(min(pdoa_sum_square));

[theta_index,phi_index] = find(pdoa_sum_square == sco_ppmin);

theta_est = theta(theta_index);
phi_est = phi(phi_index);

end

