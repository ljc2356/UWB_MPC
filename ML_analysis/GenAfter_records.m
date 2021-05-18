clear all;clc;close all;
%% 加载数据
folder = './ml_data/20210507_11201/';
savefolder = './ml_data/20210507_11201/After_records/';
File = dir(fullfile(folder,'*.json')); 
FileNamesList = {File.name}';

for i = 1:length(FileNamesList)
    filename{i} = [folder,FileNamesList{i,1}];
    records = loadRecordFile(filename{i});
    [useful_num,After_records]= UsefulSelect(records);
    After_records(1,1).SumTime = 0;
    for k = 2:useful_num
        delta_time = ReadDeltaTime(After_records,k);
        After_records(k,1).SumTime = After_records(k-1,1).SumTime + delta_time;
    end
    for k = 1:useful_num
        After_records(k,1).Target_loc = GetXY(FileNamesList{i,1});
    end
    save([savefolder,FileNamesList{i,1}(1:end - 5),'.mat'],"After_records");
end