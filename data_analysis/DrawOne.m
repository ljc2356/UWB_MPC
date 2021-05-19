%% Generate LOS_result positioning result animation script
close all;
% load('Mpc_result.mat')
% load('Los_result.mat')

xlabel('x');
ylabel('y');
set(gca,'FontSize',12);
for i = 1: useful_num% rotate and record each picture
    
   hd(1) = scatter(Los_result(index,1).m(i,1),Los_result(index,1).m(i,2),50,"ro");% Use drawing function to generate animation
   hold on;

   axis ([2.5 5.5 -1.5 1.5]);
   M(i) = getframe;% grab the frame value
   im=frame2im(M(i));
   [I,map]=rgb2ind(im,256);

    k=i-0;
    if k==1
        imwrite(I,map,"Loc.gif",'gif','Loopcount',inf,...
            'DelayTime',0.01);%loopcount is only useful when i==1
    else
        imwrite(I,map,"Loc.gif",'gif','WriteMode','append',...
            'DelayTime',0.01);%DelayTime is used to set the playback speed of the gif file
    end

end