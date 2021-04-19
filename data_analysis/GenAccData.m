clear all;
cd("D:\myProject\华为项目_数据处理\UWB_SLAM1.0\data_analysis");
folder = './data/ACC_data/';
After_folder = './data/ACC_data/After_data/';
File = dir(fullfile(folder,'*.mat')); 
FileNames = {File.name}';
file_num = length(FileNames);
for i = 1:file_num
    i
    filenames{i} = [folder, FileNames{i,1}];
    load(filenames{i});
    mean_X = mean(Acceleration.X);
    mean_Y = mean(Acceleration.Y);
    Acceleration.X = Acceleration.X - mean_X;
    Acceleration.Y = Acceleration.Y - mean_Y;
    save([After_folder,FileNames{i,1}],"Acceleration");
end

cd("D:\myProject\华为项目_数据处理\UWB_SLAM1.0\data_analysis\data\ACC_data\After_data"); % 进入工作文件夹
%% 正方形加速度生成
% Square_Acc_X = [0];
% Square_Acc_Y = [0];
% Index_array = [1,111,254,410,603];
% for i = 2:length(Index_array)
%     Square_Acc_X = [Square_Acc_X;Acceleration.X(11:10 + Index_array(i) - Index_array(i-1),1)];
%     Square_Acc_Y = [Square_Acc_Y;Acceleration.Y(11:10 + Index_array(i) - Index_array(i-1),1)];
% end
% Acceleration = [Square_Acc_X,Square_Acc_Y];
% load("711.mat")
% load("131.mat")
% load("391.mat")
% load("971.mat")

%% 菱形加速度生成
Acc_X = [0];
Acc_Y = [0];
Index_array = [1,87,181,294,474];
for i = 2:length(Index_array)
    Acc_X = [Acc_X;Acceleration.X(11:10 + Index_array(i) - Index_array(i-1),1)];
    Acc_Y = [Acc_Y;Acceleration.Y(11:10 + Index_array(i) - Index_array(i-1),1)];
end
Acceleration = [Acc_X,Acc_Y];
load("842.mat")
load("422.mat")
load("262.mat")
load("682.mat")
%% 三角加速度生成
% Acc_X = [0];
% Acc_Y = [0];
% Index_array = [1,124,247,454];
% for i = 2:length(Index_array)
%     Acc_X = [Acc_X;Acceleration.X(11:10 + Index_array(i) - Index_array(i-1),1)];
%     Acc_Y = [Acc_Y;Acceleration.Y(11:10 + Index_array(i) - Index_array(i-1),1)];
% end
% Acceleration = [Acc_X,Acc_Y];
% load("811.mat")
% load("131.mat")
% load("381.mat")

%% 8字加速度生成
% Acc_X = [0];
% Acc_Y = [0];
% Index_array = [1,105,337,460,674];
% for i = 2:length(Index_array)
%     Acc_X = [Acc_X;Acceleration.X(11:10 + Index_array(i) - Index_array(i-1),1)];
%     Acc_Y = [Acc_Y;Acceleration.Y(11:10 + Index_array(i) - Index_array(i-1),1)];
% end
% Acceleration = [Acc_X,Acc_Y];
% load("711.mat")
% load("191.mat")
% load("931.mat")
% load("371.mat")