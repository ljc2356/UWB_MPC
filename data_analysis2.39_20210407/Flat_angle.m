function [] = Flat_angle()
    global result;

    useful_num = length(result(1,1).los_d.data);

    for i = 2:useful_num
        %% 开始进行预测
        for antenna_num = 3:8
             index = antenna_num - 2 ;
             if (abs(result(index,1).mpc_phi.data(i,1) -  result(index,1).mpc_phi.data(i-1,1)  ) > 0.1)
                 result(index,1).mpc_phi.data(i,1) =  result(index,1).mpc_phi.data(i - 1,1);
             end
        end
    end

end