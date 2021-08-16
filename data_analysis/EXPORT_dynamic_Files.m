clear all;
folder = './ml_data/20210628_indoor_4/';
savaFolder = './data/20210628_indoor_4/';
File = dir(fullfile(folder,'move*.json')); 
FileNames = {File.name}';
file_num = length(FileNames);
h = str2num(folder(end-1));

for i = 1:file_num
    i
    filenames{i} = [folder, FileNames{i,1}];
    records = loadRecordFile(filenames{i});
    [useful_num,After_records]= UsefulSelect(records);
    
    y_index = 5;
    x_index = 5;
    x = 4 + (x_index - 1)* 0.25;
    y = -1 + (y_index -1) * 0.5;
    [d_los,d_mpc,diff_d_mpc,diff_index_mpc,index_mpc,phi_mpc,phi_los] = GenScenario(h,x,y);
    los_pdoa = GenPDOA(phi_los);
    mpc_pdoa = GenPDOA(phi_mpc);
    for k = 1:useful_num
        After_records(k,1).los_label.d_los = d_los;
        After_records(k,1).los_label.phi_los = phi_los;
        After_records(k,1).los_label.pdoa_los = los_pdoa;
        
        After_records(k,1).mpc_label.diff_d_mpc = diff_d_mpc;
        After_records(k,1).mpc_label.phi_mpc = phi_mpc;
        After_records(k,1).mpc_label.pdoa_moc = mpc_pdoa;
    end
    save_fillename = [savaFolder,FileNames{i,1}(1:end-5),'.mat'];
    save(save_fillename,'After_records');
end
