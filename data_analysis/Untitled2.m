clear all;clc;close all;
%% 读取数据
folder = './ml_data/20210822_11201/';
files_name = 'move_01.json';
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


%% 识别多径
mpc_index = zeros(useful_num,1);
start_index = 1;
min_distance = 3;
max_distance = 10;
mpc_update_mat = zeros(1,1);
mpc_select_style = 1;
if mpc_select_style == 1
    for i = start_index:useful_num
        mpc_index_tem = MPCDetect(After_records(i,1).uwbResult.cir,Template,Conv_Template,0);
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
%%   直接测角
% for thisPoint = 1:10
%     thisPoint
%     for i = 1:8
%         angleMat(i,:) = angle(After_records(thisPoint,1).uwbResult.cir{1,i});
%     end
%     %% 聚焦处理
%     meanIndex = (round(mpc_index(thisPoint,1))):(round(mpc_index(thisPoint,1)));
%     % 首达径
%     for los_path = 1:50
%         angleMat(1:8,los_path) = wrapToPi(angleMat(1:8,los_path) - cali');
%         for i = 1:8
%             pdoa(i) = angleMat(i,los_path) -angleMat(1,los_path);
%         end
% %         pdoa = wrapToPi(pdoa - cali); 
% 
%         antenna_index = formation{8};
%         [los_phi,~,sco_pp_los_] = AOA_ML_Mat(pdoa,antenna_index, fc , c , radius, -pi , pi);
% 
%         los_phi_mat(thisPoint,los_path) = los_phi;
%     end
%     los_phi_mat(thisPoint,meanIndex) = mean(los_phi_mat(thisPoint,meanIndex));
% end
%% MUSIC测角
clear mpcAngle
close all;
figure(3)
hold on;
mpcAngle = zeros(useful_num,20);
for thisPoint = 1:useful_num
    thisPoint
    for i = 1:8
        CIRMat(i,:) = (After_records(thisPoint,1).uwbResult.cir{1,i});
    end
    for los_path = 1:50
        CIRMat(1:8,los_path) = CIRMat(1:8,los_path).*exp(-1j*cali');
    end
    %
    Rx = zeros(8,8);
    meanIndex = (round(mpc_index(thisPoint,1))-4):(round(mpc_index(thisPoint,1))+5);
    meanIndex = 1:50;
    for los_path = meanIndex
        Rx = Rx + CIRMat(1:8,los_path) * CIRMat(1:8,los_path)';
    end
    Rx = Rx / length(meanIndex);
    [V,D] = eig(Rx);
    En = V(:,4:end);
    kk = 1;
    theta_mat = -pi/2:0.001:pi/2;
    A = AOA_Phi(theta_mat,fc,c,radius);
    Pmu = 1./ diag(A' * En * En' * A);
    Pmu = real(Pmu);
    [pks,locs] = findpeaks(Pmu,theta_mat,'SortStr','descend');
%     figure(3)
    plot(theta_mat,Pmu)
    mpcAngle(thisPoint,1:length(locs)) = locs;
%     mpcAngle(thisPoint,1) = locs(1);
%     mpcAngle(thisPoint,2) = locs(2);
end