

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
hd(2) = scatter(Mpc_result(index,1).m(:,1),Mpc_result(index,1).m(:,2),50,"b*"); 

hd(3) = plot(gt_square(:,1),gt_square(:,2)); 
set(hd(3),'color','g','linewidth',2)
axis ([2.5 5.5 -1.5 1.5]);

xlabel('x');
ylabel('y');
grid on;
lgd = legend({"LOS Localization", "LOS\\MPC Localization"},'Location','northeast','FontSize',12);
lgd.AutoUpdate = 'off'
lgd.Box = 'off'
set(gca,'FontSize',16);  



for i = 1:useful_num     % 旋转并记录每个画面
    
   hd(1) = scatter(Los_result(2,1).m(i,1),Los_result(2,1).m(i,2),50,"ro");           % 以绘画函数来产生动画
   hold on;
   hd(2) = scatter(Mpc_result(index,1).m(i,1),Mpc_result(index,1).m(i,2),50,"b*");           % 以绘画函数来产生动画
   hold on;
%    hd(3) = scatter(IMU_result.m(i,1),IMU_result.m(i,2),50,"bx");           % 以绘画函数来产生动画
%    hold on;

   axis ([2.5 5.5 -1.5 1.5]);
   
   M(i) = getframe;          % 抓取画面值
   im=frame2im(M(i));
   [I,map]=rgb2ind(im,256);

    k=i-0;
    if k==1
        xlabel('x');
        ylabel('y');
        lgd = legend({"LOS Localization", "LOS\\MPC Localization"},'Location','northeast','FontSize',12);
        lgd.AutoUpdate = 'off'
        lgd.Box = 'off'
        set(gca,'FontSize',16);  
        
        imwrite(I,map,"1.gif",'gif','Loopcount',inf,...
            'DelayTime',0.01);%loopcount只是在i==1的时候才有用
    elseif k == useful_num
        
        imwrite(I,map,"1.gif",'gif','WriteMode','append',...
            'DelayTime',0.01);%DelayTime用于设置gif文件的播放快慢
    else
        imwrite(I,map,"1.gif",'gif','WriteMode','append',...
            'DelayTime',0.01);%DelayTime用于设置gif文件的播放快慢
    end

end



