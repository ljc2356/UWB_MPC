function [] = Flat_angle()
%% 多径观测角度阈值滤波脚本
%输入 全局变量 global result 测距测角全局变量
%输出 滤波后的全局变量 global result 测距测角全局变量

%注：在多径角度计算误差较大时使用该脚本，可以对多径角度估计值进行平滑，提升联邦定位性能，
%如多径角度估计准确 则无需该滤波
%% 
    global result;

    useful_num = length(result(1,1).los_d.data);

    for i = 2:useful_num
        %% 开始进行滤波 若当前时刻多径角度变化量不现实，使用上一时刻角度代替
        for antenna_num = 3:8
             index = antenna_num - 2 ;
             if (abs(result(index,1).mpc_phi.data(i,1) -  result(index,1).mpc_phi.data(i-1,1)  ) > 0.3)
                 result(index,1).mpc_phi.data(i,1) =  result(index,1).mpc_phi.data(i - 1,1);
             end
        end
    end

end