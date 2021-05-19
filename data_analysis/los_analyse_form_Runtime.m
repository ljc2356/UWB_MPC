function [D_Phi,sco_pp_los_] = los_analyse_form_Runtime(Signal,los_path,antenna_index)
%% 首达径测距测角函数
% 输入
% Signal        当前时刻records结构体中对应的条目
% los_path      首达径索引 当前函数中默认为8
% antenna_index 使用的天线组合
% 输出
% D_phi         当前时刻直达径测距、测角结果
% sco_pp_los_   当前时刻测角误差向量，方便查看是否存在模糊现象
%% 
%读取配置文件
run("Properties.m"); 
antenna_num = length(antenna_index);

% 提取直射径的角度 和 直射径距离
[los_phi,~,sco_pp_los_] = AOA_ML_Mat(Signal.meaResult.pdoa(antenna_index),antenna_index, fc , c , radius, -pi , pi);
los_phi = wrapToPi(los_phi);
los_d = Signal.meaResult.D;

D_Phi(1) = los_d;
D_Phi(2) = los_phi;
end