function [theta_est,sco_ppmin,sco_pp] = AOA_ML( pdoa_ob,antenna_index,fc,c,radius,low_threshold,high_threshold)
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

theta_est = 0;
sco_ppmin = 100000;

kk = 1;
% for theta = - pi : 0.01 : pi
for theta = low_threshold : 0.01 : high_threshold
    phase_1 =   2 * pi * fc / c * radius * cos(theta - alpha(1));
    for i = 1:antenna_num
        phi(i) =    2 * pi * fc / c * radius * cos(theta - alpha(i));
        
        predict_pdoa(i) = wrapToPi(phi(i) - phase_1);
        pdoa_diff(i) = wrapToPi(pdoa_ob(i) - predict_pdoa(i));
    end

    sco_pp(kk) = sum(pdoa_diff.^2);
    if sco_pp(kk) < sco_ppmin
        theta_est = wrapToPi(theta);
        sco_ppmin = sco_pp(kk);
    end
    kk = kk + 1;
end



end

