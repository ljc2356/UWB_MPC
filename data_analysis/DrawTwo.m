%%
close all;
load('Mpc_result.mat')
load('Los_result.mat')

useful_num = length(Los_result(index,1).m(:,1));
for i = 1:useful_num     % 旋转并记录每个画面
    
   hd(1) = scatter(Los_result(index,1).m(i,1),Los_result(index,1).m(i,2),50,"ro");           % 以绘画函数来产生动画
   hold on;
   hd(2) = scatter(Mpc_result(index,1).m(i,1),Mpc_result(index,1).m(i,2),50,"b*");           % 以绘画函数来产生动画
   hold on;
   axis([7,14,-8,4])
    grid on;

   
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

