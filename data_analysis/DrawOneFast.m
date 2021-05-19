%% Generate the LOS_RESULT positioning result graph 

close all;
figure();
% load('Mpc_result.mat')
% load('Los_result.mat')
xlabel('x');
ylabel('y');
set(gca,'FontSize',14);  
hd(5) = scatter(Los_result(index,1) .m(:,1),Los_result(index,1) .m(:,2),50,"ro");
axis ([2.5 5.5 -1.5 1.5]);