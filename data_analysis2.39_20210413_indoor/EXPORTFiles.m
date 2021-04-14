clear all;
folder = './ml_data/20210413_indoor/';
File = dir(fullfile(folder,'*.json')); 
FileNames = {File.name}';

file_num = length(FileNames);


for i = 1:file_num
    i
    
    folder = './ml_data/20210413_indoor/';
    File = dir(fullfile(folder,'*.json')); 
    FileNames = {File.name}';
    
    filenames{i} = [folder, FileNames{i,1}];
    main_n_function(FileNames{i,1});
    clear all;close all;
end




