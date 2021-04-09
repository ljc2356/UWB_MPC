function [] = EKF_angle()
    global result;
    global EKF_result;
    run("Properties.m");
    useful_num = length(result(1,1).los_d.data);
    for antenna_num = 3:8
        index = antenna_num - 2 ;
        EKF_result(index,1).mpc_phi.data(1,1) = result(index,1).mpc_phi.data(1,1);
        Pk{index}(1,1) = 1;
    end
    for i = 2:useful_num
        %% 开始进行预测
        for antenna_num = 3:8
            
            index = antenna_num - 2 ;
            xk_minus = EKF_result(index,1).mpc_phi.data(i-1,1);
            Pk_minus = Pk{index}(i-1,1) + Sigma_angle_move^2;

            vk = result(index,1).mpc_phi.data(i,1) - xk_minus;
            Sk = Pk_minus + Sigma_angle_ob^2;
            Kk = Pk_minus* (Sk.^(-1));

            EKF_result(index,1).mpc_phi.data(i,1) = xk_minus + Kk*vk;
             Pk{index}(i,1) = Pk_minus - Kk*Sk*Kk';

        end
    end
    for antenna_num = 3:8
        index = antenna_num - 2 ;
        result(index,1).mpc_phi.data = EKF_result(index,1).mpc_phi.data;
    end
end