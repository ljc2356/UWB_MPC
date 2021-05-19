%% Generate LOS positioning and multipath auxiliary positioning comparison diagram
load('Mpc_result.mat')
load('Los_result.mat')

close all;
figure();

xlabel('x');
ylabel('y');
set(gca,'FontSize',14);  
hd(1) = scatter(Los_result(index,1) .m(:,1),Los_result(index,1) .m(:,2),50,"ro");
hold on;
hd(2) = scatter(Mpc_result(index,1).m(:,1),Mpc_result(index,1).m(:,2),50,"b+");
axis ([2.5 5.5 -1.5 1.5]);