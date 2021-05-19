function [] = EKF_angle()
%% 多径观测角度卡尔曼滤波脚本
%输入 全局变量 global result 测距测角全局变量
%输出 滤波后的全局变量 global result 测距测角全局变量

%注：在多径角度计算误差较大时使用该脚本，可以对多径角度估计值进行平滑，提升联邦定位性能，
%如多径角度估计准确 则无需该滤波
%% 预测-更新
    global result;
    global EKF_result;
    run("Properties.m");
    useful_num = length(result(1,1).los_d.data);
    % 初始化
    for antenna_num = 3:8
        index = antenna_num - 2 ;
        EKF_result(index,1).mpc_phi.data(1,1) = result(index,1).mpc_phi.data(1,1);
        Pk{index}(1,1) = 1;
    end
    for i = 2:useful_num
        for antenna_num = 3:8
            % 预测
            index = antenna_num - 2 ;
            xk_minus = EKF_result(index,1).mpc_phi.data(i-1,1);
            Pk_minus = Pk{index}(i-1,1) + Sigma_angle_move^2;
            % 更新
            vk = result(index,1).mpc_phi.data(i,1) - xk_minus;
            Sk = Pk_minus + Sigma_angle_ob^2;
            Kk = Pk_minus* (Sk.^(-1));

            EKF_result(index,1).mpc_phi.data(i,1) = xk_minus + Kk*vk;
             Pk{index}(i,1) = Pk_minus - Kk*Sk*Kk';

        end
    end
    % 输出赋值
    for antenna_num = 3:8
        index = antenna_num - 2 ;
        result(index,1).mpc_phi.data = EKF_result(index,1).mpc_phi.data;
    end
end