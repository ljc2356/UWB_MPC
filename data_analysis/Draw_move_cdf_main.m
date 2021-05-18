close all; clear all; clc;


%% 生成真实轨迹
%方形
square1(2,:) = -1:0.001:1;
square1(1,:) = 3;
square2(1,:) = 3:0.001:5;
square2(2,:) = 1;
square3(2,:) = fliplr(-1:0.001:1);
square3(1,:) = 5;
square4(1,:) = fliplr(3:0.001:5);
square4(2,:) = -1;
gt_square = [square1 square2 square3 square4]';
%菱形
lingxing1(2,:) = 0:0.001:1;
lingxing1(1,:) = 3:0.001:4;
lingxing2(1,:) = 4:0.001:5;
lingxing2(2,:) = fliplr(0:0.001:1);
lingxing3(2,:) = fliplr(-1:0.001:0);
lingxing3(1,:) = fliplr(4:0.001:5);
lingxing4(1,:) = fliplr(3:0.001:4);
lingxing4(2,:) = -1:0.001:0;
gt_lingxing = [lingxing1 lingxing2 lingxing3 lingxing4]';
%8字形
eightzixing1(2,:) = -1:0.001:1;
eightzixing1(1,:) = 3;
eightzixing2(1,:) = 3:0.001:5;
eightzixing2(2,:) = fliplr(-1:0.001:1);
eightzixing3(2,:) = fliplr(-1:0.001:1);
eightzixing3(1,:) = 5;
eightzixing4(1,:) = fliplr(3:0.001:5);
eightzixing4(2,:) = fliplr(-1:0.001:1);
gt_8zixing = [eightzixing1 eightzixing2 eightzixing3 eightzixing4]';
%三角形
sanjiao1(2,:) = -1:0.002:1;
sanjiao1(1,:) = fliplr(3:0.001:4);
sanjiao2(1,:) = 3:0.001:5;
sanjiao2(2,:) = 1;
sanjiao3(2,:) = fliplr(-1:0.002:1);
sanjiao3(1,:) = fliplr(4:0.001:5);
gt_sanjiao = [sanjiao1 sanjiao2 sanjiao3]';

figure
scatter(gt_square(:,1),gt_square(:,2)); hold on;
scatter(gt_lingxing(:,1),gt_lingxing(:,2));
scatter(gt_8zixing(:,1),gt_8zixing(:,2));
scatter(gt_sanjiao(:,1),gt_sanjiao(:,2));
%%
Rx4_floder = './data/四天线定位结果/';
Rx8_floder = './data/八天线定位结果/';
Square_floder = 'move_02 正方形/';
SanJiao_floder = 'move_04 三角形/';
Ling_floder = 'move_06_2 菱形/';
Inf_floder = 'move_07_2 8字型/';
Mpc_files = 'Mpc_result.mat';
Los_files = 'Los_result.mat';
IMU_files = 'IMU_result.mat';


square_Mpc_result_8rx = [Rx8_floder,Square_floder,Mpc_files];
square_Los_result_8rx = [Rx8_floder,Square_floder,Los_files];
square_IMU_result_8rx = [Rx8_floder,Square_floder,IMU_files];
load(square_Mpc_result_8rx )
load(square_Los_result_8rx)
load(square_IMU_result_8rx)
x_fp_square_8rx = Los_result(6,1).m(:,1:2);
x_mpc_square_8rx = Mpc_result(6,1).m(:,1:2);
x_imu_square_8rx = IMU_result.m(:,1:2);


square_Mpc_result_4rx = [Rx4_floder,Square_floder,Mpc_files];
square_Los_result_4rx = [Rx4_floder,Square_floder,Los_files];
square_IMU_result_4rx = [Rx4_floder,Square_floder,IMU_files];
load(square_Mpc_result_4rx)
load(square_Los_result_4rx)
load(square_IMU_result_4rx)
x_fp_square_4rx = Los_result(2,1).m(:,1:2);
x_mpc_square_4rx = Mpc_result(2,1).m(:,1:2);
x_imu_square_4rx = IMU_result.m(:,1:2);

lingxing_Mpc_result_8rx = [Rx8_floder,Ling_floder,Mpc_files];
lingxing_Los_result_8rx = [Rx8_floder,Ling_floder,Los_files];
lingxing_IMU_result_8rx = [Rx8_floder,Ling_floder,IMU_files];
load(lingxing_Mpc_result_8rx)
load(lingxing_Los_result_8rx)
load(lingxing_IMU_result_8rx)
x_fp_lingxing_8rx = Los_result(6,1).m(:,1:2);
x_mpc_lingxing_8rx = Mpc_result(6,1).m(:,1:2);
x_imu_lingxing_8rx = IMU_result.m(:,1:2);

lingxing_Mpc_result_4rx = [Rx4_floder,Ling_floder,Mpc_files];
lingxing_Los_result_4rx = [Rx4_floder,Ling_floder,Los_files];
lingxing_IMU_result_4rx = [Rx4_floder,Ling_floder,IMU_files];
load(lingxing_Mpc_result_4rx)
load(lingxing_Los_result_4rx)
load(lingxing_IMU_result_4rx)
x_fp_lingxing_4rx = Los_result(2,1).m(:,1:2);
x_mpc_lingxing_4rx = Mpc_result(2,1).m(:,1:2);
x_imu_lingxing_4rx = IMU_result.m(:,1:2);


Inf_Mpc_result_8rx= [Rx8_floder,Inf_floder,Mpc_files];
Inf_Los_result_8rx = [Rx8_floder,Inf_floder,Los_files];
Inf_IMU_result_8rx = [Rx8_floder,Inf_floder,IMU_files];
load(Inf_Mpc_result_8rx)
load(Inf_Los_result_8rx)
load(Inf_IMU_result_8rx)
x_fp_8zixing_8rx = Los_result(6,1).m(:,1:2);
x_mpc_8zixing_8rx = Mpc_result(6,1).m(:,1:2);
x_imu_8zixing_8rx = IMU_result.m(:,1:2);

Inf_Mpc_result_4rx= [Rx4_floder,Inf_floder,Mpc_files];
Inf_Los_result_4rx = [Rx4_floder,Inf_floder,Los_files];
Inf_IMU_result_4rx = [Rx4_floder,Inf_floder,IMU_files];
load(Inf_Mpc_result_4rx)
load(Inf_Los_result_4rx)
load(Inf_IMU_result_4rx)
x_fp_8zixing_4rx = Los_result(2,1).m(:,1:2);
x_mpc_8zixing_4rx = Mpc_result(2,1).m(:,1:2);
x_imu_8zixing_4rx = IMU_result.m(:,1:2);

sanjiao_Mpc_result_8rx = [Rx8_floder,SanJiao_floder,Mpc_files];
sanjiao_Los_result_8rx = [Rx8_floder,SanJiao_floder,Los_files];
sanjiao_IMU_result_8rx = [Rx8_floder,SanJiao_floder,IMU_files];
load(sanjiao_Mpc_result_8rx)
load(sanjiao_Los_result_8rx)
load(sanjiao_IMU_result_8rx)
x_fp_sanjiao_8rx = Los_result(6,1).m(:,1:2);
x_mpc_sanjiao_8rx = Mpc_result(6,1).m(:,1:2);
x_imu_sanjiao_8rx = IMU_result.m(:,1:2);

sanjiao_Mpc_result_4rx = [Rx4_floder,SanJiao_floder,Mpc_files];
sanjiao_Los_result_4rx = [Rx4_floder,SanJiao_floder,Los_files];
sanjiao_IMU_result_4rx = [Rx4_floder,SanJiao_floder,IMU_files];
load(sanjiao_Mpc_result_4rx)
load(sanjiao_Los_result_4rx)
load(sanjiao_IMU_result_4rx)
x_fp_sanjiao_4rx = Los_result(2,1).m(:,1:2);
x_mpc_sanjiao_4rx = Mpc_result(2,1).m(:,1:2);
x_imu_sanjiao_4rx = IMU_result.m(:,1:2);

for i = 1 : length(x_fp_square_8rx)
    temp1 = x_fp_square_8rx(i,:) - gt_square;
    temp2 = x_mpc_square_8rx(i,:) - gt_square;
    temp3 = x_fp_square_4rx(i,:) - gt_square;
    temp4 = x_mpc_square_4rx(i,:) - gt_square;
    temp5 = x_imu_square_8rx(i,:) - gt_square;
    temp6 = x_imu_square_4rx(i,:) - gt_square;
    
    dis1 = sum(temp1.^2,2).^(1/2);
    dis2 = sum(temp2.^2,2).^(1/2);
    dis3 = sum(temp3.^2,2).^(1/2);
    dis4 = sum(temp4.^2,2).^(1/2);
    dis5 = sum(temp5.^2,2).^(1/2);
    dis6 = sum(temp6.^2,2).^(1/2);
    
    md_fp_square_8rx(i) = min(dis1);
    md_mpc_square_8rx(i) = min(dis2);
    md_fp_square_4rx(i) = min(dis3);
    md_mpc_square_4rx(i) = min(dis4);
    md_imu_square_8rx(i) = min(dis5);
    md_imu_square_4rx(i) = min(dis6);
end
for i = 1 : length(x_fp_lingxing_8rx)
    temp1 = x_fp_lingxing_8rx(i,:) - gt_lingxing;
    temp2 = x_mpc_lingxing_8rx(i,:) - gt_lingxing;
    temp3 = x_fp_lingxing_4rx(i,:) - gt_lingxing;
    temp4 = x_mpc_lingxing_4rx(i,:) - gt_lingxing;
    temp5 = x_imu_lingxing_8rx(i,:) - gt_lingxing;
    temp6 = x_imu_lingxing_4rx(i,:) - gt_lingxing;
    
    dis1 = sum(temp1.^2,2).^(1/2);
    dis2 = sum(temp2.^2,2).^(1/2);
    dis3 = sum(temp3.^2,2).^(1/2);
    dis4 = sum(temp4.^2,2).^(1/2);
    dis5 = sum(temp5.^2,2).^(1/2);
    dis6 = sum(temp6.^2,2).^(1/2);
    
    md_fp_lingxing_8rx(i) = min(dis1);
    md_mpc_lingxing_8rx(i) = min(dis2);
    md_fp_lingxing_4rx(i) = min(dis3);
    md_mpc_lingxing_4rx(i) = min(dis4);
    md_imu_lingxing_8rx(i) = min(dis5);
    md_imu_lingxing_4rx(i) = min(dis6);
end
for i = 1 : length(x_fp_8zixing_8rx)
    temp1 = x_fp_8zixing_8rx(i,:) - gt_8zixing;
    temp2 = x_mpc_8zixing_8rx(i,:) - gt_8zixing;
    temp3 = x_fp_8zixing_4rx(i,:) - gt_8zixing;
    temp4 = x_mpc_8zixing_4rx(i,:) - gt_8zixing;
    temp5 = x_imu_8zixing_8rx(i,:) - gt_8zixing;
    temp6 = x_imu_8zixing_4rx(i,:) - gt_8zixing;
    
    dis1 = sum(temp1.^2,2).^(1/2);
    dis2 = sum(temp2.^2,2).^(1/2);
    dis3 = sum(temp3.^2,2).^(1/2);
    dis4 = sum(temp4.^2,2).^(1/2);
    dis5 = sum(temp5.^2,2).^(1/2);
    dis6 = sum(temp6.^2,2).^(1/2);
    
    md_fp_8zixing_8rx(i) = min(dis1);
    md_mpc_8zixing_8rx(i) = min(dis2);
    md_fp_8zixing_4rx(i) = min(dis3);
    md_mpc_8zixing_4rx(i) = min(dis4);
    md_imu_8zixing_8rx(i) = min(dis5);
    md_imu_8zixing_4rx(i) = min(dis6);
end
for i = 1 : length(x_fp_sanjiao_8rx)
    temp1 = x_fp_sanjiao_8rx(i,:) - gt_sanjiao;
    temp2 = x_mpc_sanjiao_8rx(i,:) - gt_sanjiao;
    temp3 = x_fp_sanjiao_4rx(i,:) - gt_sanjiao;
    temp4 = x_mpc_sanjiao_4rx(i,:) - gt_sanjiao;
    temp5 = x_imu_sanjiao_8rx(i,:) - gt_sanjiao;
    temp6 = x_imu_sanjiao_4rx(i,:) - gt_sanjiao;
    
    dis1 = sum(temp1.^2,2).^(1/2);
    dis2 = sum(temp2.^2,2).^(1/2);
    dis3 = sum(temp3.^2,2).^(1/2);
    dis4 = sum(temp4.^2,2).^(1/2);
    dis5 = sum(temp5.^2,2).^(1/2);
    dis6 = sum(temp6.^2,2).^(1/2);
    
    md_fp_sanjiao_8rx(i) = min(dis1);
    md_mpc_sanjiao_8rx(i) = min(dis2);
    md_fp_sanjiao_4rx(i) = min(dis3);
    md_mpc_sanjiao_4rx(i) = min(dis4);
    md_imu_sanjiao_8rx(i) = min(dis5);
    md_imu_sanjiao_4rx(i) = min(dis6);
end

%%
close all;
%方形
% figure
% h(1) = cdfplot(md_fp_square_8rx); hold on;
% set(h(1),'Color','r','linestyle','-','Marker','none','Linewidth',2); hold on;
% h(2) = cdfplot(md_mpc_square_8rx);
% set(h(2),'Color','b','linestyle','-','Marker','none','Linewidth',2); hold on;
% h(3) = cdfplot(md_fp_square_4rx);
% set(h(3),'Color','r','linestyle','--','Marker','none','Linewidth',2); hold on;
% h(4) = cdfplot(md_mpc_square_4rx);
% set(h(4),'Color','b','linestyle','--','Marker','none','Linewidth',2); hold on;
% h(5) = cdfplot(md_imu_square_8rx);
% set(h(5),'Color','g','linestyle','-','Marker','none','Linewidth',2); hold on;
% h(6) = cdfplot(md_imu_square_4rx);
% set(h(6),'Color','g','linestyle','--','Marker','none','Linewidth',2); hold on;
% lgd1 = legend(h([3 4 1 2]),'fp 4','mpc 4','fp 8','mpc 8');
% set(lgd1,'color','w','FontName','Times New Roman', 'FontSize',14);
% xlabel('Error [m]');
% ylabel('CDF');
% set(gca,'FontSize',14);  
% title('sqrare only');
%菱形
% figure
% h(1) = cdfplot(md_fp_lingxing_8rx); hold on;
% set(h(1),'Color','r','linestyle','-','Marker','none','Linewidth',2); hold on;
% h(2) = cdfplot(md_mpc_lingxing_8rx);
% set(h(2),'Color','b','linestyle','-','Marker','none','Linewidth',2); hold on;
% h(3) = cdfplot(md_fp_lingxing_4rx);
% set(h(3),'Color','r','linestyle','--','Marker','none','Linewidth',2); hold on;
% h(4) = cdfplot(md_mpc_lingxing_4rx);
% set(h(4),'Color','b','linestyle','--','Marker','none','Linewidth',2); hold on;
% 
% h(5) = cdfplot(md_imu_lingxing_8rx);
% set(h(5),'Color','g','linestyle','-','Marker','none','Linewidth',2); hold on;
% h(6) = cdfplot(md_imu_lingxing_4rx);
% set(h(6),'Color','g','linestyle','--','Marker','none','Linewidth',2); hold on;
% lgd1 = legend(h([3 4 1 2]),'fp 4','mpc 4','fp 8','mpc 8');
% set(lgd1,'color','w','FontName','Times New Roman', 'FontSize',14);
% xlabel('Error [m]');
% ylabel('CDF');
% set(gca,'FontSize',14);  
% title('lingxing only');

% 8字形
figure
h(1) = cdfplot(md_fp_8zixing_8rx); hold on;
set(h(1),'Color','r','linestyle','-','Marker','none','Linewidth',2); hold on;
h(2) = cdfplot(md_mpc_8zixing_8rx);
set(h(2),'Color','b','linestyle','-','Marker','none','Linewidth',2); hold on;
h(3) = cdfplot(md_fp_8zixing_4rx);
set(h(3),'Color','r','linestyle','--','Marker','none','Linewidth',2); hold on;
h(4) = cdfplot(md_mpc_8zixing_4rx);
set(h(4),'Color','b','linestyle','--','Marker','none','Linewidth',2); hold on;

h(5) = cdfplot(md_imu_8zixing_8rx);
set(h(5),'Color','g','linestyle','-','Marker','none','Linewidth',2); hold on;
h(6) = cdfplot(md_imu_8zixing_4rx);
set(h(6),'Color','g','linestyle','--','Marker','none','Linewidth',2); hold on;
lgd1 = legend(h([3 4 1 2]),'fp 4','mpc 4','fp 8','mpc 8');
set(lgd1,'color','w','FontName','Times New Roman', 'FontSize',14);
xlabel('Error [m]');
ylabel('CDF');
set(gca,'FontSize',14);  
title('8zixing only');
% 三角形
% figure
% h(1) = cdfplot(md_fp_sanjiao_8rx); hold on;
% set(h(1),'Color','r','linestyle','-','Marker','none','Linewidth',2); hold on;
% h(2) = cdfplot(md_mpc_sanjiao_8rx);
% set(h(2),'Color','b','linestyle','-','Marker','none','Linewidth',2); hold on;
% h(3) = cdfplot(md_fp_sanjiao_4rx);
% set(h(3),'Color','r','linestyle','--','Marker','none','Linewidth',2); hold on;
% h(4) = cdfplot(md_mpc_sanjiao_4rx);
% set(h(4),'Color','b','linestyle','--','Marker','none','Linewidth',2); hold on;
% h(5) = cdfplot(md_imu_sanjiao_8rx);
% set(h(5),'Color','g','linestyle','-','Marker','none','Linewidth',2); hold on;
% h(6) = cdfplot(md_imu_sanjiao_4rx);
% set(h(6),'Color','g','linestyle','--','Marker','none','Linewidth',2); hold on;
% 
% lgd1 = legend(h([3 4 1 2]),'fp 4','mpc 4','fp 8','mpc 8');
% set(lgd1,'color','w','FontName','Times New Roman', 'FontSize',14);
% xlabel('Error [m]');
% ylabel('CDF');
% set(gca,'FontSize',14);  
% title('sanjiao only');
% 
% %所有
% md_fp_8rx = [md_fp_square_8rx md_fp_lingxing_8rx md_fp_8zixing_8rx md_fp_sanjiao_8rx];
% md_mpc_8rx = [md_mpc_square_8rx md_mpc_lingxing_8rx md_mpc_8zixing_8rx md_mpc_sanjiao_8rx];
% md_fp_4rx = [md_fp_square_4rx md_fp_lingxing_4rx md_fp_8zixing_4rx md_fp_sanjiao_4rx];
% md_mpc_4rx = [md_mpc_square_4rx md_mpc_lingxing_4rx md_mpc_8zixing_4rx md_mpc_sanjiao_4rx];
% figure
% h(1) = cdfplot(md_fp_8rx); hold on;
% set(h(1),'Color','r','linestyle','-','Marker','none','Linewidth',2); hold on;
% h(2) = cdfplot(md_mpc_8rx);
% set(h(2),'Color','b','linestyle','-','Marker','none','Linewidth',2); hold on;
% h(3) = cdfplot(md_fp_4rx);
% set(h(3),'Color','r','linestyle','--','Marker','none','Linewidth',2); hold on;
% h(4) = cdfplot(md_mpc_4rx);
% set(h(4),'Color','b','linestyle','--','Marker','none','Linewidth',2); hold on;
% lgd1 = legend(h([3 4 1 2]),'fp 4','mpc 4','fp 8','mpc 8');
% set(lgd1,'color','w','FontName','Times New Roman', 'FontSize',14);
% xlabel('Error [m]');
% ylabel('CDF');
% set(gca,'FontSize',14);  
% title('all');


