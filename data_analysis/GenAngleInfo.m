clear all;
folder = './data/20210413_indoor_angle/';
File = dir(fullfile(folder,'*.mat')); 
FileNames = {File.name}';

file_num = length(FileNames);
Mpc_Angle_truth = [1.1659,1.0517,0.9509,1.2120,1.1071,1.0122,1.2490,1.1526,1.0637,1.1659,1.0517,0.9509,1.2120,1.1071,1.0122,1.2490,1.1526,1.0637];
Los_Angle_truth = [0.3218,0.2450,0.1974,     0,  0    , 0    -0.3218,-0.2450,-0.1974,0.3218,0.2450,0.1974,     0,     0,    0 , -0.3218,-0.2450,-0.1974];
Los_d_truth = [3.1623,4.1231,5.0990,3,4,5,3.1623,4.1231,5.0990,3.1623,4.1231,5.0990,3,4,5,3.1623,4.1231,5.0990];
Mpc_d_truth = [7.6158,8.0623,8.6023,8.5440,8.9443,9.4340,9.4868,9.8489,10.2956,7.6158,8.0623,8.6023,8.5440,8.9443,9.4340,9.4868,9.8489,10.2956];


for i = 1:file_num
    i
    filenames{i} = [folder, FileNames{i,1}];
    load(filenames{i});
    for antenna_num = 3:8
        index = antenna_num - 2;
        [~,formation_num] = size(result(index,1).los_phi.data);
        for k = 1:formation_num
            result(index,1).los_phi.square_error(:,k) = (wrapToPi(result(index,1).los_phi.data{1,k} - Los_Angle_truth(i))).^2;
            result(index,1).mpc_phi.square_error(:,k)= (wrapToPi(result(index,1).mpc_phi.data{1,k} - Mpc_Angle_truth(i))).^2;
            result(index,1).los_phi.mse = mean(result(index,1).los_phi.square_error,2);
            result(index,1).mpc_phi.mse = mean(result(index,1).mpc_phi.square_error,2);
        end
        result(index,1).los_d.square_error = (result(index,1).los_d.data{1,1} - Los_d_truth(i)).^2;
        result(index,1).mpc_d.square_error = (result(index,1).mpc_d.data{1,1}- Mpc_d_truth(i)).^2;
    end
    save(filenames{i},"result");
end


for i = 1:file_num
    i
    filenames{i} = [folder, FileNames{i,1}];
    load(filenames{i});
    for antenna_num = 3:8
        index = antenna_num - 2;
        [~,formation_num] = size(result(index,1).los_phi.data);
        for k = 1:formation_num
            Angle_MSE(index,1).los_mse(:,i) = mean(result(index,1).los_phi.mse,1);
            Angle_MSE(index,1).mpc_mse(:,i) = mean(result(index,1).mpc_phi.mse,1);
        end
        Angle_MSE(index,1).los_d_mse(:,i) = mean(result(index,1).los_d.square_error,1);
        Angle_MSE(index,1).mpc_d_mse(:,i) = mean(result(index,1).mpc_d.square_error,1);
    end
end

for antenna_num = 3:8
    index = antenna_num - 2;
    Angle_MSE(index,1).los_rmse = sqrt(mean(Angle_MSE(index,1).los_mse,2));
    Angle_MSE(index,1).mpc_rmse = sqrt(mean(Angle_MSE(index,1).mpc_mse,2));
    Angle_MSE(index,1).los_d_rmse = sqrt(mean(Angle_MSE(index,1).los_d_mse,2));
    Angle_MSE(index,1).mpc_d_rmse = sqrt(mean(Angle_MSE(index,1).mpc_d_mse,2));
    
    los_rmse_mat(1,antenna_num) = sqrt(mean(Angle_MSE(index,1).los_mse,2));
    mpc_rmse_mat(1,antenna_num) = sqrt(mean(Angle_MSE(index,1).mpc_mse,2));
    los_d_rmse(1,antenna_num) = sqrt(mean(Angle_MSE(index,1).los_d_mse,2));
    mpc_d_rmse(1,antenna_num) = sqrt(mean(Angle_MSE(index,1).mpc_d_mse,2));
end

% 
% close all;
% f = figure();
% load('LOS_los.mat')
% hdRMSE(1) = plot(los_rmse_mat);
% hold on;
% load('NLOS_los.mat')
% hdRMSE(2) = plot( los_rmse_mat);
% hold on;
% load('MPC.mat')
% hdRMSE(3) = plot( mpc_rmse_mat);
% hold on;
% 
% 
% set(hdRMSE(1),'color','r','linewidth',2);
% set(hdRMSE(2),'color','b','linewidth',2);
% set(hdRMSE(3),'color','g','linewidth',2);
% 
% 
% xlabel('Number of Antennas');
% ylabel('RMSE [rad]');
% set(gca,'FontSize',14);
% axis([3 8 0 0.2]);
% grid on;
% legend("RMSE of LOS AOA","RMSE of LOS AOA with NLOS","RMSE of MPC AOA");
% % myfolder ='./graph/';
% % filesname = [myfolder,'RMSE of MPC Angle.fig'];
% % savefig(f,filesname);


Acceleration = Ling_Acc

