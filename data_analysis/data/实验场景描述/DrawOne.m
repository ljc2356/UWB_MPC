

close all;clear all;

Datafolder = '../八天线定位结果/move_02 正方形/';
load([Datafolder,'IMU_result.mat']);
load([Datafolder,'Los_result.mat']);
load([Datafolder,'Mpc_result.mat']);
index = 8 -2;

useful_num = length(IMU_result.m(:,1));
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



