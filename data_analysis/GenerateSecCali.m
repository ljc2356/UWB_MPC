function [sec_cali] = GenerateSecCali(filenames,enable)
%     clear all; clc;close all;
    run("Properties.m");   %读取参数列表 
    records = loadRecordFile(filenames);
    file_num = length(records);
    [useful_num,After_records]= UsefulSelect(records);
    
    if enable == 1       %% 生成校准值
        for antenna_num = 3:8
            index = antenna_num - 2 ; %从3开始

            for i = 1:useful_num
                [After_records(i,1).los_phi,~,sco_pp_los(i,:)] = AOA_ML_nan(After_records(i,1).meaResult.pdoa(1:antenna_num),antenna_num, fc , c , radius, -pi  , pi );
            end

            for i = 1:useful_num
                los_d(i,1) = After_records(i,1).meaResult.D;
                los_phi(i,1) = After_records(i,1).los_phi;
            end
            sec_cali(antenna_num,1) = mean(los_d) - los_d_truth;
            sec_cali(antenna_num,2)  = mean(los_phi) - los_phi_truth;
        end

    else              % 不采用校准值
        for antenna_num = 3:8
            sec_cali(antenna_num,1) = 0;
            sec_cali(antenna_num,2)  = 0;
        end
    end

end

    
    

    
    

