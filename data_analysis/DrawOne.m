%% 生成LOS_result 定位结果动图脚本
close all;
load('Mpc_result.mat')
load('Los_result.mat')

xlabel('x');
ylabel('y');
set(gca,'FontSize',12);  
for i = 1:useful_num     % 旋转并记录每个画面
    
   hd(1) = scatter(Los_result(index,1).m(i,1),Los_result(index,1).m(i,2),50,"ro");           % 以绘画函数来产生动画
   hold on;

   axis ([2.5 5.5 -1.5 1.5]);
   M(i) = getframe;          % 抓取画面值
   im=frame2im(M(i));
   [I,map]=rgb2ind(im,256);

    k=i-0;
    if k==1
        imwrite(I,map,"Loc.gif",'gif','Loopcount',inf,...
            'DelayTime',0.01);%loopcount只是在i==1的时候才有用
    else
        imwrite(I,map,"Loc.gif",'gif','WriteMode','append',...
            'DelayTime',0.01);%DelayTime用于设置gif文件的播放快慢
    end

end