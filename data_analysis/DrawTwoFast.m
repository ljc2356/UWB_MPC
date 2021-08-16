%%
load('Mpc_result.mat')
load('Los_result.mat')

figure();
hold on;

fs = 0.001;
temp_truth{1}(:,2) = -0.9:fs:0.9;
temp_truth{1}(:,1) = 8.1;

temp_truth{2}(:,1) = 8.1:fs:11.7;
temp_truth{2}(:,2) =0.9;
temp_truth{3}(:,2) = -0.9:fs:0.9;
temp_truth{3}(:,1) = 11.7;

temp_truth{4}(:,1) = 8.1:fs:11.7;
temp_truth{4}(:,2) = -0.9;

ground_truth = [];
for i = 1:4
    ground_truth = [ground_truth;temp_truth{i}];
end





xlabel('x');
ylabel('y');
set(gca,'FontSize',14);  
% hd(7) = scatter(ground_truth(:,1),ground_truth(:,2),20,"g*");
hd(5) = scatter(Los_result(index,1) .m(:,1),Los_result(index,1) .m(:,2),50,"ro");
hd(6) = scatter(Mpc_result(index,1).m(:,1),Mpc_result(index,1).m(:,2),50,"b+");

grid on;
legend("Groundtruth","Localization of LOS","Localization of LOS and MPC")

