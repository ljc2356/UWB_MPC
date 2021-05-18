clear all;clc;close all;
% 
% set(locRMSE(1),'color','r','linewidth',2);
% set(locRMSE(2),'color','g','linewidth',2);
% set(locRMSE(3),'color','b','linewidth',2);
% 
% xlabel('Number of Antennas');
% ylabel('RMSE [m]');
% 
% axis([3 8 0 0.5]);
% grid on;
% legend("RMSE of LOS and NLOS Localization","RMSE of LOS Localization","RMSE of NLOS Localization");
% filesname = [myfolder,'RMSE of Location.fig'];
% savefig(f,filesname);
%% 室内场景
fs = 0.0001;
Base = [0,0];
NLOS = [2,-0.25;
        2, 0.5];
Mirror = [1,4;
          4,4];
Target = [3 1;
          4 1;
          5 1;
          3 0;
          4 0;
          5 0;
          3 -1;
          4 -1;
          5 -1];
StepPoint = [7 1 3 9 7
             8 1 2 3 8
             8 4 2 6 8
             7 1 9 3 7];  % 直接在两点之间画线
x_min = -1*0.5;
x_max = 5.5;

%% 作图 
figure(1);

plot(Base(1),Base(2),'rs','LineWidth',2);
hold on;
plot(NLOS(:,1),NLOS(:,2),'k','LineWidth',2);
plot(Mirror(:,1),Mirror(:,2),'b','LineWidth',2);
plot(Target(:,1),Target(:,2),'mx','LineWidth',2);
for i = 1:length(StepPoint(:,1))
    TempPath = [];
    for k = 1:length(StepPoint(i,:))
        TempPath = [TempPath;Target(StepPoint(i,k),:)];
        
    end
    plot(TempPath(:,1),TempPath(:,2),'LineWidth',1.5);
end

axis([x_min,x_max,-1.5,4.5])
set(gca,'FontSize',14);


k1 = NLOS(1,2)/NLOS(1,1);
x1 = 0 : fs: x_max;
y1 = k1 * x1;
p1 = area(x1 , y1 ,'EdgeColor','none','FaceColor','k', 'LineStyle', 'None');
p1.FaceAlpha = 0.2;
p1.BaseLine.LineStyle = 'none';

k2 = NLOS(2,2)/NLOS(2,1);
x2 = 0 : fs: x_max;
y2 = k2 * x2;
p1 = area(x2 , y2 ,'EdgeColor','none','FaceColor','k', 'LineStyle', 'None');
p1.FaceAlpha = 0.2;
p1.BaseLine.LineStyle = 'none';
legend("Anchor","Aluminum Foil Cardboard","Metal Reflector","Real Trajectories and spot");
xlabel('X [m]');
ylabel('Y [m]');





