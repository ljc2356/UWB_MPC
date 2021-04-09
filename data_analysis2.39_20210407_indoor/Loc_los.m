function [loc_result] = Loc_los(enable)
%% 声明要用的全局变量
    global result;
    global loc_result;
    if enable == 1
        useful_num = length(result(1).los_d.data);
    else
        useful_num = 10;
    end
    
%% 开始定位   
    for antenna_num = 3:8
        index = antenna_num - 2 ; %从3开始
        C = diag([result(index,1).los_d.std_d,result(index,1).los_phi.std_phi]);
        C = C^2;

        data_mat(:,1) = result(index,1).los_d.data;
        data_mat(:,2) = result(index,1).los_phi.data;

        for i = 1:useful_num

            loc = optimvar('loc',1,2);
            data = data_mat(i,1:2)';
            obj = fcn2optimexpr(@func_los,loc,C,data);
            prob = optimproblem('Objective',obj);
            nlcons = loc(1)^2 + loc(2)^2 <= 2;
            prob.Constraints.cons = nlcons;
            x0.loc = [0.8,0];
            [sol,fval] = solve(prob,x0);
            loc_result(index,1).XYloc_los(i,1:2) = sol.loc;
        end

        loc_result(index,1).los_meanXY = mean(loc_result(index,1).XYloc_los,1);
        loc_result(index,1).los_stdXY = std(loc_result(index,1).XYloc_los,1);
    end
end

function p = func_los(loc,C,mean_data)
%% 读取场景参数
    run("Properties.m");
%% 
    mu = [norm([loc(1),loc(2)]) ;atan2(loc(2),loc(1))];
    p = -(-1/2 * (mean_data - mu)' * (C^(-1)) * (mean_data - mu));
end