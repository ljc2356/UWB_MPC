%%
load('Mpc_result.mat')
load('Los_result.mat')

figure();
hold on;

groundtruth1(:,1) = [1.242:0.001:4.357];
groundtruth1(:,2) = 0;
groundtruth2(:,2) = [0:0.001:3,6];
groundtruth2(:,1) = 4.357;
groundtruth = [groundtruth1;groundtruth2];


xlabel('x');
ylabel('y');
set(gca,'FontSize',14);  
% hd(7) = scatter(ground_truth(:,1),ground_truth(:,2),20,"g*");
hd(3)= scatter(groundtruth(:,1),groundtruth(:,2),50,"g.");
hd(5) = scatter(Los_result(index,1) .m(:,1),Los_result(index,1) .m(:,2),50,"ro");
hd(6) = scatter(Mpc_result(index,1).m(:,1),Mpc_result(index,1).m(:,2),50,"b+");





grid on;
legend("ground truth","Localization of LOS and MPC","Localization of LOS")

