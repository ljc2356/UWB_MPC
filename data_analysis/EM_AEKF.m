clear all;close all;clc;
load('data/20210413_indoor/move_07_2.mat')
global result;
global Los_result;
global Mpc_result;
%% 进行参数设置
antenna_num = 8;
index = antenna_num - 2;
over_max = 40;
ob_threhold = 0.00001;
ob_mpc_threhold = 0.00001;
dis_threhold = 0.15;
iteration = 100;
%%  融合
  for iter = 1:iteration
%  for iter = 1
    iter
    run("Copy_of_AEKF_new_noneIMU.m");
    useful_num = length(Los_result(index,1).m(:,1));
    %% 直达径的校准
    correct_los_d = sqrt(sum(Los_result(index,1).m(:,1:2).*Los_result(index,1).m(:,1:2),2));
    correct_los_phi = atan2(Los_result(index,1).m(:,2),Los_result(index,1).m(:,1));
    Diff(:,1) = result(index,1).los_d.data - correct_los_d;
    Diff(:,2) = wrapToPi(result(index,1).los_phi.data - correct_los_phi);
    Diff_norm = sqrt(sum(Diff(:,1:2).*Diff(:,1:2),2));
    Wrong_index = find(Diff_norm(over_max+1:end) > ob_threhold) + over_max
    Wrong_Diff = Diff(Wrong_index,1:2)
    if length(Wrong_index)>1
        Z = linkage(Wrong_Diff,'average','chebychev');
        T = cluster(Z,'cutoff',dis_threhold,'Criterion','distance');
        for k = 1:max(T)
            NLOS_index = find(T == k);
            Avg_Diff{k} = mean(Wrong_Diff(NLOS_index,1:2),1);
            result(index,1).los_d.data(Wrong_index(NLOS_index),1) = result(index,1).los_d.data(Wrong_index(NLOS_index),1) - Avg_Diff{k}(1,1);
            result(index,1).los_phi.data(Wrong_index(NLOS_index),1) = result(index,1).los_phi.data(Wrong_index(NLOS_index),1) - Avg_Diff{k}(1,2);
        end
    else
        mean_Diff = mean(Wrong_Diff,1);
        result(index,1).los_d.data(Wrong_index,1) = result(index,1).los_d.data(Wrong_index,1) - mean_Diff(1,1);
        result(index,1).los_phi.data(Wrong_index,1) = result(index,1).los_phi.data(Wrong_index,1) - mean_Diff(1,2);
    end
    %% 开始进行多径校准
%     [xm,ym] = mirror(Mpc_result(index,1).m(:,1),Mpc_result(index,1).m(:,2),Mpc_result(index,1).m(:,5),Mpc_result(index,1).m(:,6));
%     Mpc_loc = [xm,ym];
%     correct_mpc_d = sqrt(sum(Mpc_loc(:,1:2).*Mpc_loc(:,1:2),2));
%     correct_mpc_phi = atan2(Mpc_loc(:,2),Mpc_loc(:,1));
%     Diff_mpc(:,1) = result(index,1).mpc_d.data - correct_mpc_d;
%     Diff_mpc(:,2) = wrapToPi(result(index,1).mpc_phi.data - correct_mpc_phi);
%     Diff_mpc_norm = sqrt(sum(Diff(:,1:2).*Diff(:,1:2),2));
%     Wrong_mpc_index = find(Diff_mpc_norm(over_max+1:end) > ob_mpc_threhold) + over_max;
%     Wrong_mpc_Diff = Diff_mpc(Wrong_mpc_index,1:2)
%     
%     if length(Wrong_mpc_index)>1
%         Z = linkage(Wrong_mpc_Diff,'average','chebychev');
%         T = cluster(Z,'cutoff',dis_threhold,'Criterion','distance');
%         for k = 1:max(T)
%             NLOS_index = find(T == k);
%             Avg_Mpc_Diff{k} = mean(Wrong_mpc_Diff(NLOS_index,1:2),1);
%             result(index,1).mpc_d.data(Wrong_mpc_index(NLOS_index),1) = result(index,1).mpc_d.data(Wrong_mpc_index(NLOS_index),1)  - Avg_Mpc_Diff{k}(1,1);
%             result(index,1).mpc_phi.data(Wrong_mpc_index(NLOS_index),1)= result(index,1).mpc_phi.data(Wrong_mpc_index(NLOS_index),1) - Avg_Mpc_Diff{k}(1,2);
%         end
%     end
%     
    
%     
%  if((isempty(Wrong_index) == 1)&&(isempty(Wrong_mpc_Diff) == 1))
    if(isempty(Wrong_index) == 1)
        break;
    end
% move_04 三角形
% move_06_2 菱形
% move_07_2 8字形
    % i= 1的时候存一下不进行EM的数据
    if iter == 1
        save("./data/八天线定位结果/move_07_2 8字型/Los_result.mat","Los_result");
        figure(1)
        run("DrawOneFast.m");
        title("EM step = 1");
    end
  end
figure(2)
run("DrawOneFast.m");
title("EM finally");
save("./data/八天线定位结果/move_07_2 8字型/Mpc_result.mat","Mpc_result");