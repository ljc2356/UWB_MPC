clear all;close all;clc;
Json_folder = './ml_data/20210413_indoor/';
filenames = 'move_08_2';
Json_filenames =  [Json_folder,filenames,'.json'];
Result_folder = './data/20210413_indoor/';
Result_filenames = [Result_folder,filenames,'.mat'];
After_folder = './data/20210413_indoor_AfterRecords/';
After_filenames = [After_folder,filenames,'.mat'];

%% 挑选有用数据
records = loadRecordFile(Json_filenames);
[useful_num,After_records]= UsefulSelect(records);

load(Result_filenames)
global result;
global Los_result;
global Mpc_result;
%% 进行参数设置
antenna_num = 8;
index = antenna_num - 2;
over_max = 40;
ob_threhold = 0.1;
dis_threhold = 0.15;
%%  求解

for iter = 1
    iter
    run("Copy_of_AEKF_new_noneIMU.m");
    useful_num = length(Los_result(index,1).m(:,1));
    %% 直达径的校准
    correct_los_d = sqrt(sum(Los_result(index,1).m(:,1:2).*Los_result(index,1).m(:,1:2),2));
    correct_los_phi = atan2(Los_result(index,1).m(:,2),Los_result(index,1).m(:,1));
    Diff(:,1) = result(index,1).los_d.data - correct_los_d;
    Diff(:,2) = wrapToPi(result(index,1).los_phi.data - correct_los_phi);
    Diff_norm = sqrt(sum(Diff(:,1:2).*Diff(:,1:2),2));
    Wrong_index = find(Diff_norm(over_max+1:end) > ob_threhold) + over_max;
    Wrong_Diff = Diff(Wrong_index,1:2);
    for i = 1:useful_num
        After_records(i,1).los_phi = result(index,1).los_phi.data(i,1);
    end
    

    Z = linkage(Wrong_Diff,'average','chebychev');
    T = cluster(Z,'cutoff',dis_threhold,'Criterion','distance');
    for k = 1:max(T)
        NLOS_index = find(T == k);
        Avg_Diff{k} = mean(Wrong_Diff(NLOS_index,1:2),1);
        for Input_iter = 1:length(NLOS_index)
            After_records(Wrong_index(NLOS_index(Input_iter,1)),1).los_phi = result(index,1).los_phi.data(Wrong_index(NLOS_index(Input_iter)),1) - Avg_Diff{k}(1,2);
        end
    end

end
run("DrawOneFast.m");
save(After_filenames,'After_records');