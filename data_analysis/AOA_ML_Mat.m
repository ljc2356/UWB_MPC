function [theta_est,sco_ppmin,sco_pp] = AOA_ML_Mat( pdoa_ob,antenna_index,fc,c,radius,low_threshold,high_threshold)
%% �����б�
% pdoa_ob�۲쵽�� pdoa
% anteenna_index ѡ�õ���������
% fc ��Ƶ ����Ƶ��
% c ����
% radius �������Ͱ뾶
% low/high threshold �����ĸߵ���ֵ


antenna_num = length(antenna_index);
for i = 1:antenna_num
    k = antenna_index(i);
    alpha(i) = wrapToPi((k-1)*pi/4);
end

theta = low_threshold : 0.001 : high_threshold;
std_phi = 2 * pi * fc / c * radius * cos(theta);   % ȫ��������� 1�����ߵ�pdoa
for i = 1:antenna_num
    phi(i,:) =    2 * pi * fc / c * radius * cos(theta - alpha(i));
    predict_pdoa(i,:) = wrapToPi( phi(i,:) - std_phi );
end
pdoa_diff = predict_pdoa - pdoa_ob';
pdoa_square = pdoa_diff.* pdoa_diff;
pdoa_sum_square = sum(pdoa_square,1);
sco_ppmin = min(pdoa_sum_square);
target_index = find(pdoa_sum_square == sco_ppmin);
theta_est = theta(target_index);
sco_pp = pdoa_sum_square;
end

