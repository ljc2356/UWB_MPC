
clear a b ab Base cl;
figure();

xlabel('x');
ylabel('y');
set(gca,'FontSize',14);  
hd(5) = scatter(Mpc_result(index,1) .m(:,1),Mpc_result(index,1) .m(:,2),50,"ro");
axis ([2.5 5.5 -1.5 1.5]);