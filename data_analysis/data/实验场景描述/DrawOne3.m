

close all;clear all;

Datafolder = '../八天线定位结果/move_02 正方形/';
load([Datafolder,'IMU_result.mat']);
load([Datafolder,'Los_result.mat']);
load([Datafolder,'Mpc_result.mat']);
index = 8 -2;

square1(2,:) = -1:0.001:1;
square1(1,:) = 3;
square2(1,:) = 3:0.001:5;
square2(2,:) = 1.25;
square3(2,:) = fliplr(-1:0.001:1);
square3(1,:) = 5;
square4(1,:) = fliplr(3:0.001:5);
square4(2,:) = -1;
gt_square = [square1 square2 square3 square4]';


figure();hold on;
useful_num = length(IMU_result.m(:,1));
hd(1) = scatter(Los_result(2,1).m(:,1),Los_result(2,1).m(:,2),50,"ro");
hd(2) = plot(gt_square(:,1),gt_square(:,2)); 
set(hd(2),'color','b','linewidth',2)
axis ([2.5 5.5 -1.5 1.5]);

xlabel('x');
ylabel('y');
grid on;
lgd = legend({"LOS Localization", "Truth Trajectory"},'Location','northeast','FontSize',12);
lgd.AutoUpdate = 'off'
lgd.Box = 'off'
set(gca,'FontSize',16);  

