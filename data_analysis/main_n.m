clear all;clc;close all;
%% 读取数据
folder = './ml_data/20210413_indoor/';
files_name = 'move_07.json';
filenames{1} = [folder,files_name];  %读取分析数据参数
records = loadRecordFile(filenames{1});
 
cali_folder = './ml_data/20210413_indoor/';     %读取二次校准数据参数
cali_filenames = [cali_folder, 'cali_01.json'];

Back_folder ='./ml_data/20210413_indoor/';   
Back_filenames = [Back_folder, 'cali_01.json'];  %读取模板生成参数
    
global result;
for antenna_num = 3:8 
    index = antenna_num - 2 ; %从3开始
    result(index,1).antenna_num = antenna_num;  %开辟空间
end
%% 参数设置
run("Properties.m");
%% 挑选有用数据
[useful_num,After_records]= UsefulSelect(records);
%% 生成校准值
[sec_cali] = GenerateSecCali(cali_filenames,0);
%% 生成模板
[ Template, Conv_Template ] = GenerateTemplate(Back_filenames);
%% 定位多径位置
% close all
% figure();
% hold on;
% for i = 1:8
%     hd = plot(abs(After_records(300,1).uwbResult.cir{1,i}));
% end
% Ori_CIR = abs(After_records(299,1).uwbResult.cir{1,1});
% hd = plot(Ori_CIR);
% set(hd,'color','b','linewidth',2);
% xlabel('Time [ns]');
% ylabel('Amplitude');
% grid on;
% set(gca,'FontSize',14);
% legend("Original CIR Waveform");
% 
% CIR = zeros(1,50);
% for i = 1:8
%     CIR = CIR + (abs(After_records(300,1).uwbResult.cir{1,i}));
% end
% CIRhd = plot(CIR);
% set(CIRhd,'color','b','linewidth',2);
% xlabel('Time [ns]');
% ylabel('Amplitude');
% grid on;
% set(gca,'FontSize',14);
% legend("Original CIR Waveform");


mpc_index = zeros(useful_num,1);
start_index = 1;
min_distance = 3;
max_distance = 10;
mpc_update_mat = zeros(1,1);
mpc_select_style = 0;
if mpc_select_style == 1
    for i = start_index:useful_num
        mpc_index_tem = MPCDetect(After_records(i,1).uwbResult.cir,Template,Conv_Template,mpc_select_style);
        if (isempty(mpc_index_tem) == 0)
            mpc_index(i,1:length(mpc_index_tem)) = mpc_index_tem;  % 获得多径的位置
        end
    end
     
elseif mpc_select_style == 0
     for i = start_index:useful_num
        mpc_index_tem = MPCDetect(After_records(i,1).uwbResult.cir,Template,Conv_Template,mpc_select_style);
        if (isempty(mpc_index_tem) == 0)
            if i>1
                matching_index = zeros(1,length(mpc_update_mat));
                mpc_updata_mat_back = mpc_update_mat;
                for kk = 1:length(mpc_index_tem)
                    last_distance_mat =  abs(mpc_updata_mat_back - mpc_index_tem(kk)); 
                    last_distance = min(last_distance_mat);
                    last_index = find(last_distance_mat == last_distance);

                    if last_distance<min_distance
                        matching_index(1,last_index) = mpc_index_tem(kk);
                        mpc_update_mat(1,last_index) = mpc_index_tem(kk);
                    elseif last_distance>max_distance

                    else
                        now_length = length(matching_index);
                        matching_index(1,now_length+1) = mpc_index_tem(kk);
                        mpc_update_mat(1,now_length+1) = mpc_index_tem(kk);
                    end
                end
                mpc_update_mat = unique(mpc_update_mat,'stable');
                mpc_index(i,1:length(matching_index)) = matching_index;
            else
                mpc_update_mat = mpc_index_tem;
                mpc_index(i,1:length(mpc_index_tem)) = mpc_index_tem;  % 获得多径的位置
            end

        end

     end
end

 

data_index = 1:useful_num;
% data_index = 1:100;
for i = data_index
    i
    for antenna_num = 3:8
        index = antenna_num - 2 ;
        antenna_index = formation{antenna_num};
        ReadTime(After_records,index,i);
        
        
         %[D_Phi,sco_pp_los{antenna_num}(i,:) ] = los_analyse_form_Runtime(After_records(i,1),los_path,antenna_index,sec_cali);  %分析此数据的直达径
         [D_Phi,~ ] = los_analyse_form_Runtime(After_records(i,1),los_path,antenna_index,sec_cali);  %分析此数据的直达径
         result(index,1).los_d.data(i,1) = D_Phi(1);
         result(index,1).los_phi.data(i,1) = D_Phi(2);
         result(index,1).los_pdoa.data(i,:) = After_records(i,1).meaResult.pdoa;
         
         for mpc_th = 1:length(mpc_index(i,:))  % 如果是空的，则不会填写
             if mpc_index(i,mpc_th) == 0
                 result(index,1).mpc_d.data(i,mpc_th) = 0;
                 result(index,1).mpc_phi.data(i,mpc_th) = 0;
             else
                 [D_Phi,sco_pp_mpc{antenna_num}(i,:),mpc_pdoa] = mpc_analyse_form_Runtime(After_records(i,1),mpc_index(i,mpc_th),los_path,antenna_index,sec_cali);
    %                  [D_Phi,~] = mpc_analyse_form_Runtime(After_records(i,1),temp_mpc_index(mpc_th),los_path,antenna_index,sec_cali);
                 result(index,1).mpc_d.data(i,mpc_th) = D_Phi(1);
                 result(index,1).mpc_phi.data(i,mpc_th) = D_Phi(2);
             end
         end

    end
end

savejson('',result,files_name);


% figure(1);
% for antenna_num = 3:8
%     
%     plot(mean(sco_pp_mpc{antenna_num}(data_index,:),1));
%     hold on;
% end

%% 对角度进行离线滤波
% std(result(index,1).los_phi.data)
% mean(result(index,1).los_phi.data)

% EKF_angle();
% Flat_angle()
% 
data_folder = './data/20210413_indoor_update/';
data_name = [data_folder,files_name(1:end - 5),'.mat'];
save(data_name,"result");


% %% 绘制sco_pp图
% close all;
% % figure();
% for antenna_num = 3:8
% figure(antenna_num);
% subplot(2,1,1);
% plot(mean(sco_pp_mpc{antenna_num},1));
% hold on
% title("sco pp mpc");
% subplot(2,1,2);
% plot(mean(sco_pp_los{antenna_num},1));
% hold on;
% title("sco pp los")
% end
