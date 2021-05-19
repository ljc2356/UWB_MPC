function [theta_est,sco_ppmin,sco_pp] = AOA_ML_Mat( pdoa_ob,antenna_index,fc,c,radius,low_threshold,high_threshold)
%% PDOA ������Ȼ����㷨 
%����
% pdoa_ob�۲쵽�� pdoa
% anteenna_index ѡ�õ���������
% fc ��Ƶ ����Ƶ��
% c ����
% radius �������Ͱ뾶
% low/high threshold �����ĸߵ���ֵ
%���
% theta_est �����Ƕȹ���ֵ
% sco_ppmin ����ֵ��Ӧ��loss
% sco_pp    ���нǶȵ�loss�����������Թ۲��Ƿ����ģ������

%% ��ʼ�����еĽǶ�λ�ýǶ�
    antenna_num = length(antenna_index);
    for i = 1:antenna_num
        k = antenna_index(i);
        alpha(i) = wrapToPi((k-1)*pi/4);
    end
    theta_est = 0;
    sco_ppmin = 100000;

%% �����нǶȽ��б��� 
    theta = low_threshold : 0.01 : high_threshold;
    std_phi = 2 * pi * fc / c * radius * cos(theta);
    for i = 1:antenna_num
        phi(i,:) =    2 * pi * fc / c * radius * cos(theta - alpha(i));
        predict_pdoa(i,:) = wrapToPi( phi(i,:) - std_phi );
    end
    %�������нǶ�pdoa ��۲��pdoa�����ƽ����
    pdoa_diff = predict_pdoa - pdoa_ob';
    pdoa_square = pdoa_diff.* pdoa_diff;
    pdoa_sum_square = sum(pdoa_square,1);
    %�ҵ������С�Ƕ� �����
    sco_ppmin = min(pdoa_sum_square);
    target_index = find(pdoa_sum_square == sco_ppmin);
    theta_est = theta(target_index);
    sco_pp = pdoa_sum_square;
end

