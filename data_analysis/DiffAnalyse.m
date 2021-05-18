clear all;
cd("D:\myProject\华为项目_数据处理\UWB_SLAM1.0\data_analysis");
folder = './ml_data/20210507_11201/';
File = dir(fullfile(folder,'*.json')); 
FileNames = {File.name}';
file_num = length(FileNames);
run("Properties.m");
global result;

% for i = 1:file_num
%     
%     filenames{i} = [folder, FileNames{i,1}];
%     records = loadRecordFile(filenames{i});
%     [useful_num,After_records]= UsefulSelect(records);
%     for j = 1:useful_num
%         pdoa{i}(j,:) = After_records(j,1).meaResult.pdoa;
%         D{i}(j,:) = After_records(j,1).meaResult.D;
%     end
%     AnlyResult(i,1).meanPdoa = mean(pdoa{i},1);
%     AnlyResult(i,1).D = mean(D{i},1);
%     AnlyResult(i,1).angle = main_n_function(FileNames{i,1});
%     clear global result;
%     AnlyResult(i,1).name = FileNames{i,1};
%     
% end
% save("data/AnlyResult.mat","AnlyResult");
%%
load("data/AnlyResult.mat");

for i = 1:8
    alpha(i) = wrapToPi((i-1)*pi/4);
end


for i = 1:file_num
    
    real_x = str2num(FileNames{i,1}(1)) * 0.6;
    if  FileNames{i,1}(2) == "-"
        real_y = -1 * str2num(FileNames{i,1}(3)) * 0.6;
    else
        real_y = str2num(FileNames{i,1}(2)) * 0.6;
    end
    real_angle(i) = atan2(real_y,real_x);
    
    std_phi = 2 * pi * fc / c * radius * cos(real_angle(i));
    for j = 1:8
        phi(j,:) = 2 * pi * fc / c * radius * cos(real_angle(i) - alpha(j));
        predict_pdoa(j,:) = wrapToPi( phi(j,:) - std_phi );
    end
    AnlyResult(i,1).realPdoa(1,:) = predict_pdoa;
    AnlyResult(i,1).DiffPdoa(1,:) = wrapToPi(AnlyResult(i,1).meanPdoa - AnlyResult(i,1).realPdoa);
    AnlyResult(i,1).realAngle = real_angle(i);
    AnlyResult(i,1).DiffAngle = wrapToPi(AnlyResult(i,1).angle - real_angle(i));
end




