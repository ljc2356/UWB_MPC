
% load('Mpc_result.mat')
% load('Los_result.mat')
close all;clear a b ab Base cl;
a(2,:) = -2:0.001:0;
a(1,:) = 3;
b(1,:) = 3:0.001:5;
b(2,:) = 0;
ab = [a b];
Base(1,1) = 0;
Base(2,1) = 0;
c1(2,:) = -0.2:0.01:0.2;
c1(1,:) = 0.8;

hd(1) = plot(ab(1,:),ab(2,:));
hold on;


hd(3) = scatter(Base(1,:),Base(2,:),60,"sk",'linewidth',2);
hold on;

hd(4) = plot(c1(1,:),c1(2,:));
hold on;


set(hd(1),'color','g','linestyle','-','linewidth',2)
set(hd(4),'color',[1 0.5 0],'linestyle','-','linewidth',2)
xlabel('x');
ylabel('y');
set(gca,'FontSize',12);  
for i = 1:useful_num     % 旋转并记录每个画面
    
   hd(5) = scatter(Los_result(index,1).m(i,1),Los_result(index,1).m(i,2),50,"ro");           % 以绘画函数来产生动画
   hold on;
%    hd(5) = scatter(IMU_result.m(i,1),IMU_result.m(i,2),50,"ro");           % 以绘画函数来产生动画
%    hold on;

   axis ([2.5 5.5 -1.5 1.5]);
   if i == useful_num
       legend([hd(1),hd(3),hd(4),hd(5)],"Ground Truth","Anchor" ,"Obstacles","LOS Localization",'Location','SouthEast');
   end
   
   M(i) = getframe;          % 抓取画面值
   im=frame2im(M(i));
   [I,map]=rgb2ind(im,256);

    k=i-0;
    if k==1
        imwrite(I,map,"1.gif",'gif','Loopcount',inf,...
            'DelayTime',0.01);%loopcount只是在i==1的时候才有用
    else
        imwrite(I,map,"1.gif",'gif','WriteMode','append',...
            'DelayTime',0.01);%DelayTime用于设置gif文件的播放快慢
    end

end