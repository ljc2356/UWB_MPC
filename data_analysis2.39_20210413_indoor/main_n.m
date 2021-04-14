clear all;clc;close all;
%% 读取数据
folder = './ml_data/20210413_indoor/';
files_name = 'move_04_2.json';
filenames{1} = [folder,files_name];  %读取分析数据参数
records = loadRecordFile(filenames{1});
 
cali_folder = './ml_data/20210407_indoor/';     %读取二次校准数据参数
cali_filenames = [cali_folder, 'background_01.json'];

Back_folder ='./ml_data/20210407_indoor/';   
Back_filenames = [Back_folder, 'background_01.json'];  %读取模板生成参数
    
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
% figure(1);
% for i = 1:8
% hd = plot(abs(After_records(1).uwbResult.cir{1,i}));
% hold on;
% end
% set(hd,'color','b','linewidth',2);
% xlabel('Time [ns]');
% ylabel('Amplitude');
% grid on;
% set(gca,'FontSize',14);
% legend("Original CIR Waveform");

mpc_index = [];


 for i = 1:useful_num
    mpc_index_tem = MPCDetect(After_records(i,1).uwbResult.cir,Template,Conv_Template);
    if (isempty(mpc_index_tem) == 0)
        mpc_index(i,1:length(mpc_index_tem)) = mpc_index_tem;  % 获得多径的位置
    end
    
  %加上一个变化速度判断
    if i >=2 
        if abs( mpc_index(i,1) - mpc_index(i-1,1)) > 3  % 如果变化太大 则视为无效 采用上一组数据的多径寻找结果
            mpc_index(i,1) = mpc_index(i-1,1);
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
         
         if (mpc_index(i,1) == 0) % 如果没有多径，则在多径的数据栏上增加标识: 多径的距离是 0 通过检测多径距离即可判断是否存在多径
             result(index,1).mpc_d.data(i,1) = 0;
             result(index,1).mpc_phi.data(i,1) = 0;
         else
             temp_mpc_index = mpc_index(i,find(mpc_index(i,:)));
             for mpc_th = 1:length(temp_mpc_index)  % 如果是空的，则不会填写
                 [D_Phi,sco_pp_mpc{antenna_num}(i,:)] = mpc_analyse_form_Runtime(After_records(i,1),mpc_index(i,mpc_th),los_path,antenna_index,sec_cali);
%                  [D_Phi,~] = mpc_analyse_form_Runtime(After_records(i,1),temp_mpc_index(mpc_th),los_path,antenna_index,sec_cali);
                 result(index,1).mpc_d.data(i,mpc_th) = D_Phi(1);
                 result(index,1).mpc_phi.data(i,mpc_th) = D_Phi(2);
             end
         end
    end
end
% figure(1);
% for antenna_num = 3:8
%     
%     plot(mean(sco_pp_mpc{antenna_num}(data_index,:),1));
%     hold on;
% end

%% 对角度进行离线滤波

% EKF_angle();
Flat_angle()

data_folder = './data/20210413_indoor/';
data_name = [data_folder,files_name(1:end - 5),'.mat'];
save(data_name,"result");





% %% 数据进行反向处理
% for antenna_num = 3:8
%     index = antenna_num -2;
%     result(index,1).los_d.data = flip(result(index,1).los_d.data );
%     result(index,1).los_phi.data = flip(result(index,1).los_phi.data );
%     result(index,1).mpc_d.data = flip(result(index,1).mpc_d.data );
%     result(index,1).mpc_phi.data = flip(result(index,1).mpc_phi.data );
% end
% 
% % result(6).mpc_phi.data(280:329,1) = result(6).mpc_phi.data(280:329,1) + 0.02;
% save("result_inv.mat")




%result(6,1).mpc_phi.data  =result(6,1).mpc_phi.data - ( mean(result(6,1).mpc_phi.data ) -  atan(6))
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

%% 进行定位 
% global loc_result;
% [loc_result] = Loc_losmpc(0);  % （联合定位）
% [loc_result] = Loc_los(0);     % 单独直射径定位
% [loc_result] = Loc_mpc(0);     % 单独反射径定位
% %% 存储数据
