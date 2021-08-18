%%
load('Mpc_result.mat')
load('Los_result.mat')

figure();
hold on;




xlabel('x');
ylabel('y');
set(gca,'FontSize',14);  
% hd(7) = scatter(ground_truth(:,1),ground_truth(:,2),20,"g*");
hd(5) = scatter(Los_result(index,1) .m(:,1),Los_result(index,1) .m(:,2),50,"ro");
hd(6) = scatter(Mpc_result(index,1).m(:,1),Mpc_result(index,1).m(:,2),50,"b+");

grid on;
legend("Localization of LOS","Localization of LOS and MPC")

