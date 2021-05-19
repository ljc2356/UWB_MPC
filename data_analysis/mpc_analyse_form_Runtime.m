function [D_Phi,sco_pp_mpc_] = mpc_analyse_form_Runtime(Signal,multi_path,los_path,antenna_index)
%% 多径测距测角函数
% 输入
% Signal        当前时刻records结构体中对应的条目
% los_path      首达径索引 当前函数中默认为8
% multi_path    多径索引 由多径识别算法导出
% antenna_index 使用的天线组合
% 输出
% D_phi         当前时刻多径测距、测角结果
% sco_pp_los_   当前时刻多径测角误差向量，方便查看是否存在模糊现象
%%
%读取配置文件
run("Properties.m"); 
antenna_num = length(antenna_index);

%提取多径相对直射径的距离
mpc_delay = (multi_path - los_path) / 1000000000 * 300000000;
multi_path = round(multi_path)    ; %换算成整数索引，提取PDOA

% 提取多径的PDOA
for k = 1:8
    mpc_phase(1,k) = angle(Signal.uwbResult.cir{1,k}(multi_path,1));
    mpc_pdoa(1,k) = wrapToPi(mpc_phase(1,k) - mpc_phase(1,1))  ;
end
mpc_pdoa = wrapToPi(mpc_pdoa - cali);   % 减去对应的校准值 这里是由于 首达径信息已经是校准完毕的结果 而多径信息则是从未校准的波形中提取的 因此需要校准值

% 多径测角 测距
[mpc_phi,~ ,sco_pp_mpc_] = AOA_ML_Mat(mpc_pdoa(antenna_index),antenna_index,fc , c , radius , -1*pi  ,  pi  ); 
los_d = Signal.meaResult.D;
mpc_d = los_d + mpc_delay;
mpc_phi = wrapToPi(mpc_phi) ;
D_Phi(1) = mpc_d;
D_Phi(2) = mpc_phi;
end