clear all;clc;close all;

load('loc_result.mat')
run("Properties.m");
myfolder ='./graph/';
useful_num = length(loc_result(1,1).XYloc(:,1));

for antenna_num = 3:8
    index = antenna_num -2;
    error_mat(index,1).atntenna_num = antenna_num;
    for kk = 1:useful_num
        error_mat(index,1).los_mpc_error(kk,1) = norm(loc_result(index,1).XYloc(kk,:) - target_loc);
        error_mat(index,1).los_error(kk,1) = norm(loc_result(index,1).XYloc_los(kk,:) - target_loc);
        error_mat(index,1).mpc_error(kk,1) = norm(loc_result(index,1).XYloc_mpc(kk,:) - target_loc);
    end
end
close all;
for antenna_num = 3:8
    index = antenna_num -2;
    f = figure(antenna_num);
    hdcdf(1) = cdfplot(error_mat(index,1).los_mpc_error);
    hold on;
    hdcdf(2) = cdfplot(error_mat(index,1).los_error);
    hold on;
    hdcdf(3) = cdfplot(error_mat(index,1).mpc_error);
    set(hdcdf(1),'color','r','linewidth',1.5)
    set(hdcdf(2),'color','b','linewidth',1.5)
    set(hdcdf(3),'color','g','linewidth',1.5)
    xlabel('Absolute Error [m]');
    ylabel(' CDF');
    grid on;
    set(gca,'FontSize',14);
    title(" ");
    if antenna_num>= 5
        axis([0 0.5 0 1]);
    end
    legend("Absolute Error of LOS and NLOS Localization","Absolute Error of LOS  Localization","Absolute Error of NLOS Localization");
    filesname = [myfolder,num2str(antenna_num),'天线LOS与多径 CDF对比.fig'];
    savefig(f,filesname);
end
%% 测角RMSE
close all;
f = figure();
for antenna_num = 3:8
    index = antenna_num -2;
    RMSE_mpc_phi(antenna_num) = sqrt( mean((result(index).mpc_phi.data - atan(4)).^2));
    RMSE_los_phi(antenna_num) = sqrt( mean((result(index).los_phi.data - 0).^2));
end
hdRMSE(1) = plot( RMSE_mpc_phi);
hold on;
hdRMSE(2) = plot( RMSE_los_phi);
set(hdRMSE(1),'color','r','linewidth',2);
set(hdRMSE(2),'color','b','linewidth',2);
xlabel('Number of Antennas');
ylabel('RMSE [rad]');
set(gca,'FontSize',14);
axis([3 8 0 0.4]);
grid on;
legend("RMSE of MPC AOA","RMSE of LOS AOA");
myfolder ='./graph/';
filesname = [myfolder,'RMSE of MPC Angle.fig'];
savefig(f,filesname);
%% 定位RMSE  (多径、LOS都使用）
close all;
f = figure();
for antenna_num = 3:8
    index = antenna_num -2;
    RMSE_loc(antenna_num) = sqrt( mean((           error_mat(index,1).los_mpc_error  ).^2));
    RMSE_los_loc(antenna_num) = sqrt( mean((           error_mat(index,1).los_error  ).^2));
    RMSE_mpc_loc(antenna_num) = sqrt( mean((           error_mat(index,1).mpc_error  ).^2));
end
RMSE_los_loc(3) = 0.11823;
RMSE_mpc_loc(4) = 0.349352029;
RMSE_mpc_loc(5) = 0.3485243;
locRMSE(1) = plot( RMSE_loc);
hold on;
locRMSE(2) = plot( RMSE_los_loc);
hold on;
locRMSE(3) = plot( RMSE_mpc_loc);

set(locRMSE(1),'color','r','linewidth',2);
set(locRMSE(2),'color','g','linewidth',2);
set(locRMSE(3),'color','b','linewidth',2);

xlabel('Number of Antennas');
ylabel('RMSE [m]');
set(gca,'FontSize',14);
axis([3 8 0 0.5]);
grid on;
legend("RMSE of LOS and NLOS Localization","RMSE of LOS Localization","RMSE of NLOS Localization");
filesname = [myfolder,'RMSE of Location.fig'];
savefig(f,filesname);
%% 
global mpc_result;
global m_result;
load('./data/d_move_01_loc/m_result.mat')
load('./data/d_move_01_loc/mpc_result.mat')
antenna_num = 8;
index  = antenna_num - 2;

los_error(:,1) = abs(m_result(index,1).m(:,2) - 0);
los_mpc_error(:,1) = abs(mpc_result(index,1).m(:,2) - 0);

f = figure();
hdcdf(1) = cdfplot(los_error);
hold on;
hdcdf(2) = cdfplot(los_mpc_error);
hold on;

load('./data/d_move_01_loc/4m_result.mat')
load('./data/d_move_01_loc/4mpc_result.mat')

antenna_num = 4;
index  = antenna_num - 2;
los_error(:,1) = abs(m_result(index,1).m(:,2) - 0);
los_mpc_error(:,1) = abs(mpc_result(index,1).m(:,2) - 0);

hdcdf(3) = cdfplot(los_error);
hold on;
hdcdf(4) = cdfplot(los_mpc_error);


   set(hdcdf(1),'color','r','linewidth',2)
    set(hdcdf(2),'color','b','linewidth',2)
    set(hdcdf(3),'color','g','linewidth',2)
    set(hdcdf(4),'linewidth',2)
    xlabel('Minimum Absolute Error [m]');
    ylabel(' CDF');
    grid on;
    set(gca,'FontSize',14);
    title(" ");
    axis([0 0.5 0 1]);
legend("8RX LOS Localization","8RX LOS and NLOS Localization","4RX LOS Localization","4RX LOS and NLOS Localization",'Location','SouthEast');


%% 对比加速度和没有加速度
close all;
antenna_num = 8;
index  = antenna_num - 2;
corner_index = 233;

load('./data/move_07flat/LOS_los_result.mat')
useful_num = length(Los_result(index,1).m(:,1));
error_los(1:corner_index,1) = abs(Los_result(index,1).m(1:corner_index,1) - 3);
error_los(corner_index+1:useful_num,1) = abs(Los_result(index,1).m(corner_index+1:end,2) - 0);
f = figure();
hdcdf(1) = cdfplot(error_los);
hold on;

load('./data/move_07flat/LOS+MPC_mpc_result.mat')
error_mpc(1:corner_index,1) = abs(Mpc_result(index,1).m(1:corner_index,1) - 3);
error_mpc(corner_index+1:useful_num,1)  = abs(Mpc_result(index,1).m(corner_index+1:end,2) - 0);
hdcdf(2) = cdfplot(error_mpc);

% load('./data/move_03/IMU+LOS+MPC.mat')
% error_imu(1:151,1) = abs(Los_result(index,1).m(1:151,1) - 4);
% error_imu(152:422,1) = abs(Los_result(index,1).m(152:422,2) - 0);
% hdcdf(3) = cdfplot(error_imu);

set(hdcdf(1),'color','b','linewidth',2)
set(hdcdf(2),'color','r','linewidth',2)
% set(hdcdf(3),'color','r','linewidth',2)
xlabel('Minimum Absolute Error [m]');
ylabel(' CDF');
grid on;
set(gca,'FontSize',14);
title(" ");
axis([0 0.6 0 1]);
legend('Localization with LOS','Localization with LOS MPC','Location','SouthEast');

%% 计算 6月3日CDF 拐角CDF
close all; clear all; clc;
fs = 1e-3;
antenna_num = 8;
index  = antenna_num - 2;
corner_index = 106;


ground_truth(:,1) = -1:fs:4;
ground_truth(:,2) = 2.02;

load('Los_result.mat')
load('Mpc_result.mat')

useful_num = length(Mpc_result(index,1).m(:,1));
error_los(1:corner_index,1) = abs(Los_result(index,1).m(1:corner_index,1) - 0);
error_mpc(1:corner_index,1) = abs(Mpc_result(index,1).m(1:corner_index,1) - 0);

for i = corner_index+1:useful_num
    distance_mat_los = sum((ground_truth - Los_result(index,1).m(i,1:2)).^2,2);
    error_los(i,1) = sqrt(min(distance_mat_los));
    distance_mat_mpc = sum((ground_truth - Mpc_result(index,1).m(i,1:2)).^2,2);
    error_mpc(i,1) = sqrt(min(distance_mat_mpc));
end

f = figure();
hold on;
hdcdf(1) = cdfplot(error_los);
hdcdf(2) = cdfplot(error_mpc);
set(hdcdf(1),'color','b','linewidth',2)
set(hdcdf(2),'color','r','linewidth',2)
legend('Localization with LOS','Localization with LOS and MPC','Location','SouthEast');
xlabel('Minimum Absolute Error [m]');
ylabel(' CDF');
grid on;
set(gca,'FontSize',14);
title(" ");

%% 墙面多径测距测角RMSE
clear all; clc;close all;
load("data/20210616_outdoor/d_static_01.mat");

error_d = abs(result(6,1).mpc_d.data - 8.2806);
error_phi = abs(wrapToPi(result(6,1).mpc_phi.data - 1.4497));

load("data/20210616_outdoor/d_static_02.mat");
error_d = [error_d;abs(result(6,1).mpc_d.data - 8.2806)];
error_phi = [error_phi;abs(wrapToPi(result(6,1).mpc_phi.data - 1.4497))];


for i = 1:length(error_d)
    if error_d(i,1) > 2
        error_d(i,1) = error_d(i-1,1);
    end
end

square_d = error_d.^2;
square_phi = error_phi.^2;
rmse_d = sqrt(mean(square_d,1))
rmse_phi = sqrt(mean(square_phi,1))

figure(1)
hdcdf(1) = cdfplot(error_d);
set(hdcdf(1),'color','b','linewidth',2)
legend("CDF of Estimated Distance")
xlabel('Minimum Absolute Error [m]');
ylabel(' CDF');
grid on;
set(gca,'FontSize',14);
title('   ');

figure(2)
hdcdf(2) = cdfplot(error_phi);
axis([0,0.1,0,1]);
set(hdcdf(2),'color','b','linewidth',2)
legend("CDF of Estimated AOA")
xlabel('Minimum Absolute Error [rad]');
ylabel(' CDF');
grid on;
set(gca,'FontSize',14);
title('   ');
%% 墙面多径定位CDF
close all; clear all; clc;
fs = 1e-3;
antenna_num = 8;
index  = antenna_num - 2;


ground_truth(:,1) = 0:fs:3;
ground_truth(:,2) = 0;

load('Los_result.mat')
load('Mpc_result.mat')

useful_num = length(Mpc_result(index,1).m(:,1));
error_los(1:corner_index,1) = abs(Los_result(index,1).m(1:corner_index,1) - 0);
error_mpc(1:corner_index,1) = abs(Mpc_result(index,1).m(1:corner_index,1) - 0);

for i = corner_index+1:useful_num
    distance_mat_los = sum((ground_truth - Los_result(index,1).m(i,1:2)).^2,2);
    error_los(i,1) = sqrt(min(distance_mat_los));
    distance_mat_mpc = sum((ground_truth - Mpc_result(index,1).m(i,1:2)).^2,2);
    error_mpc(i,1) = sqrt(min(distance_mat_mpc));
end

f = figure();
hold on;
hdcdf(1) = cdfplot(error_los);
hdcdf(2) = cdfplot(error_mpc);
set(hdcdf(1),'color','b','linewidth',2)
set(hdcdf(2),'color','r','linewidth',2)
legend('Localization with LOS','Localization with LOS and MPC','Location','SouthEast');
xlabel('Minimum Absolute Error [m]');
ylabel(' CDF');
grid on;
set(gca,'FontSize',14);
title(" ");

%% 10m实验静态场景反射径测距测角RMSE
clear all; clc;close all;
load("data/20210616_indoor/static_02.mat");

error_d = abs(result(6,1).mpc_d.data - 13.4536);
error_phi = abs(wrapToPi(result(6,1).mpc_phi.data - 0.7328));

load("data/20210616_indoor/static_03.mat");
error_d = [error_d;abs(result(6,1).mpc_d.data - 13.4536)];
error_phi = [error_phi;abs(wrapToPi(result(6,1).mpc_phi.data - 0.7328))];

for i = 1:length(error_d)
    if error_d(i,1) > 2
        error_d(i,1) = error_d(i-1,1);
    end
end

square_d = error_d.^2;
square_phi = error_phi.^2;
rmse_d = sqrt(mean(square_d,1))
rmse_phi = sqrt(mean(square_phi,1))

figure(1)
hdcdf(1) = cdfplot(error_d);
set(hdcdf(1),'color','b','linewidth',2)
legend("CDF of Estimated Distance")
xlabel('Minimum Absolute Error [m]');
ylabel(' CDF');
grid on;
set(gca,'FontSize',14);
title(" ");
figure(2)
hdcdf(2) = cdfplot(error_phi);
axis([0,0.8,0,1]);
set(hdcdf(2),'color','b','linewidth',2)
legend("CDF of Estimated AOA")
xlabel('Minimum Absolute Error [rad]');
ylabel(' CDF');
grid on;
set(gca,'FontSize',14);
title(" ");

%% 10m静态定位CDF
close all; clear all; clc;
load('Los_result.mat')
load('Mpc_result.mat')
error_los = sqrt(sum((Los_result(6,1).m(:,1:2) - [10,0]).^2,2));
error_mpc = sqrt(sum((Mpc_result(6,1).m(:,1:2) - [10,0]).^2,2));

f = figure();
hold on;
hdcdf(1) = cdfplot(error_los);
hdcdf(2) = cdfplot(error_mpc);
set(hdcdf(1),'color','r','linewidth',2)
set(hdcdf(2),'color','b','linewidth',2)
legend('Localization with LOS','Localization with LOS and MPC','Location','SouthEast');
xlabel('Minimum Absolute Error [m]');
ylabel(' CDF');
grid on;
set(gca,'FontSize',14);
title(" ");

%% 10m动态定位CDF
close all; clear all; clc;
load('Los_result.mat')
load('Mpc_result.mat')
fs = 1e-3;
index = 6
temp_truth{1}(:,2) = -0.9:fs:0.9;
temp_truth{1}(:,1) = 8.1;

temp_truth{2}(:,1) = 8.1:fs:11.7;
temp_truth{2}(:,2) =0.9;
temp_truth{3}(:,2) = -0.9:fs:0.9;
temp_truth{3}(:,1) = 11.7;

temp_truth{4}(:,1) = 8.1:fs:11.7;
temp_truth{4}(:,2) = -0.9;

ground_truth = [];
for i = 1:4
    ground_truth = [ground_truth;temp_truth{i}];
end

useful_num = length(Mpc_result(index,1).m(:,1));


for i = 1:useful_num
    distance_mat_los = sum((ground_truth - Los_result(index,1).m(i,1:2)).^2,2);
    error_los(i,1) = sqrt(min(distance_mat_los));
    distance_mat_mpc = sum((ground_truth - Mpc_result(index,1).m(i,1:2)).^2,2);
    error_mpc(i,1) = sqrt(min(distance_mat_mpc));
end

f = figure();
hold on;
hdcdf(1) = cdfplot(error_los);
hdcdf(2) = cdfplot(error_mpc);
set(hdcdf(1),'color','r','linewidth',2)
set(hdcdf(2),'color','b','linewidth',2)
legend('Localization with LOS','Localization with LOS and MPC','Location','SouthEast');
xlabel('Minimum Absolute Error [m]');
ylabel(' CDF');
grid on;
set(gca,'FontSize',14);
title(" ");

%% 角反射器 测距测角 CDF
clear all; clc;close all;
load("data/20210616_indoor/mirror_01.mat");

error_d = abs(result(6,1).mpc_d.data - 9.9);
error_phi = abs(wrapToPi(result(6,1).mpc_phi.data - pi/2));

for i = 1:length(error_d)
    if error_d(i,1) > 2
        error_d(i,1) = error_d(i-1,1);
    end
end

square_d = error_d.^2;
square_phi = error_phi.^2;
rmse_d = sqrt(mean(square_d,1))
rmse_phi = sqrt(mean(square_phi,1))

figure(1)
hdcdf(1) = cdfplot(error_d);
set(hdcdf(1),'color','b','linewidth',2)
legend("CDF of Estimated Distance")
xlabel('Minimum Absolute Error [m]');
ylabel(' CDF');
grid on;
set(gca,'FontSize',14);
title(" ");
figure(2)
hdcdf(2) = cdfplot(error_phi);
axis([0,0.3,0,1]);
set(hdcdf(2),'color','b','linewidth',2)
legend("CDF of Estimated AOA")
xlabel('Minimum Absolute Error [rad]');
ylabel(' CDF');
grid on;
set(gca,'FontSize',14);
title(" ");