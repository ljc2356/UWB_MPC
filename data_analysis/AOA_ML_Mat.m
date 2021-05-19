function [theta_est,sco_ppmin,sco_pp] = AOA_ML_Mat( pdoa_ob,antenna_index,fc,c,radius,low_threshold,high_threshold)
%% PDOA Maximum Likelihood Angle Measurement Algorithm
%Input
% pdoa observed by pdoa_ob
% anteenna_index The selected antenna array
% fc Carrier frequency Center frequency
% c speed of light
% radius Antenna array radius
% low/high threshold Search high and low threshold
% Output
% theta_est Estimated value of incoming wave angle
% loss corresponding to the estimated value of sco_ppmin
% sco_pp loss vector of all angles, (to observe whether there is blurring)

%% initialize all angle position angle
    antenna_num = length(antenna_index);
    for i = 1:antenna_num
        k = antenna_index(i);
        alpha(i) = wrapToPi((k-1)*pi/4);
    end
    theta_est = 0;
    sco_ppmin = 100000;

%% traverse all angles
    theta = low_threshold: 0.01: high_threshold;
    std_phi = 2 * pi * fc / c * radius * cos(theta);
    for i = 1:antenna_num
        phi(i,:) = 2 * pi * fc / c * radius * cos(theta-alpha(i));
        predict_pdoa(i,:) = wrapToPi( phi(i,:)-std_phi );
    end
    % Calculate the sum of squared errors of all angles pdoa and observed pdoa
    pdoa_diff = predict_pdoa-pdoa_ob';
    pdoa_square = pdoa_diff.* pdoa_diff;
    pdoa_sum_square = sum(pdoa_square,1);
    % Find the minimum error angle and output
    sco_ppmin = min(pdoa_sum_square);
    target_index = find(pdoa_sum_square == sco_ppmin);
    theta_est = theta(target_index);
    sco_pp = pdoa_sum_square;
end

