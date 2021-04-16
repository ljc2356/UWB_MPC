%%
load('Mpc_result.mat')
load('Los_result.mat')

close all;
figure();
a(2,:) = 2.7:0.001:4;
a(1,:) = 0;
b(1,:) = 0:0.001:3;
b(2,:) = 4;
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
set(gca,'FontSize',14);  
hd(5) = scatter(Los_result(index,1) .m(:,1),Los_result(index,1) .m(:,2),50,"ro");
hd(6) = scatter(Mpc_result(index,1).m(:,1),Mpc_result(index,1).m(:,2),50,"b+");
axis ([2.5 5.5 -1.5 1.5]);