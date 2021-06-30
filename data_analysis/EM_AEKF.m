clear all;close all;clc;
load('data/20210603/move_05.mat')
global result;
global Los_result;
global Mpc_result;
%% 进行参数设置
antenna_num = 8;
index = antenna_num - 2;
learning_rate = 0.05;
iteration = 2000;
run("Properties.m");
%%  融合
useful_num = length(result(index,1).los_d.data);
for k = 1:8
    alpha(k) = wrapToPi((k-1)*pi/4);
end
for iter = 1:iteration
    iter
    run("AEKF_new_noneIMU.m");
    %% 根据融合定位结果计算稍微正确的首达径位置
    for i = 1:useful_num
        correctD(i,1) = norm(Mpc_result(index,1).m(i,1:2));
        correctAngle(i,1) = atan2(Mpc_result(index,1).m(i,2),Mpc_result(index,1).m(i,1));
        
        std_phi = 2 * pi * fc / c * radius * cos(correctAngle(i,1)); 
        for k = 1:8
            phi(k) =    2 * pi * fc / c * radius * cos(correctAngle(i,1) - alpha(k));
            predict_pdoa(k) = wrapToPi( phi(k) - std_phi );
        end
        correctPDOA(i,1:8) = predict_pdoa;
        Diff_D(i,1) = (result(index,1).los_d.data(i,1) - correctD(i,1)) * learning_rate;
    end
    Diff_PDOA = (result(index,1).los_pdoa.data - correctPDOA) * learning_rate;
    result(index,1).los_d.data = result(index,1).los_d.data - Diff_D;
    result(index,1).los_pdoa.data = result(index,1).los_pdoa.data - Diff_PDOA;
    for i = 1:useful_num
        [result(index,1).los_phi.data(i,1),~,~] = AOA_ML_Mat(result(index,1).los_pdoa.data(i,:),formation{antenna_num}, fc , c , radius, -pi , pi);
    end
    
    if mod(iter,100) == 0
        run("DrawTwoFast.m");
        title(iter)
    end
end
run("DrawTwoFast.m");
