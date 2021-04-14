clear all;clc;close all;
load('data/d_move_10filter.mat')
load('data/d_move_01.mat')
load('data/acc-move-04.mat');
x_back = mean(Acceleration.X);
y_back = mean(Acceleration.Y);
Acceleration.X = Acceleration.X - x_back;
Acceleration.Y = Acceleration.Y - y_back;
global result;
global m_result;
global IMU_result;
global Los_result;
global Mpc_result;
global Avg_result;
beta = 1/3;

%% 指定参数


antenna_num = 8;
index = antenna_num - 2;
init_state = [0.55  0   0.1  0   0    0  3   0]; 
% 初始状态    x     y    vx  vy  ax   ay h theta

%% 噪声模型估计
% 运动噪声
Delta_u = 1;  
 
% 观测噪声
Sigma_ax = 0.0492;
Sigma_ay = 0.1529;
Sigma_los_d = 0.0226;
Sigma_los_phi = 0.0046;
Sigma_mpc_d = 0.0626;
Sigma_mpc_phi = 0.0106; 


IMU_result.m(1,:) = init_state(1,1:6);
IMU_result.P{1} = eye(6);
IMU_result.Q = eye(6)/10000;
IMU_result.R = diag([Sigma_ax^2,Sigma_ay^2]);


Los_result(index,1).antenna_num = antenna_num;
Los_result(index,1).m(1,:) = init_state(1,1:4);% 指定初值
Los_result(index,1).P{1} = eye(4);  %相关噪声给大一些
Los_result(index,1).Q = eye(4)/10000;  %运动噪声小一点
Los_result(index,1).R = diag([Sigma_los_d^2,Sigma_los_phi^2]);   %观测噪声任取


Mpc_result(index,1).antenna_num = antenna_num;
Mpc_result(index,1).m(1,:) = [init_state(1,1:4),init_state(1,7:8)];% 指定初值
Mpc_result(index,1).P{1} = eye(6);   %相关噪声给大一些
Mpc_result(index,1).Q = [eye(4)/10000,zeros(4,2);zeros(2,4),zeros(2,2)];   %运动噪声小一点
Mpc_result(index,1).R =  diag([Sigma_mpc_d^2,Sigma_mpc_phi^2]);%观测噪声任取

result(index,1).mpc_d.data(112:194,1) = result(index,1).mpc_d.data(98,1);
%% 开始进行LOS_EKF
 for i = 2:useful_num
    %%  预测
        Delta_time = result(index,1).Delta_time(i,1);
        Q_full = diag([(Delta_u^2)*(Delta_time^3)/3,(Delta_u^2)*(Delta_time^3)/3,(Delta_u^2)*(Delta_time^2)/2,(Delta_u^2)*(Delta_time^2)/2,(Delta_u^2)*(Delta_time^1)/1,(Delta_u^2)*(Delta_time^1)/1]);              
%         Q_local = [0.00647402979153186,0,0,0;0,0.00126659154525236,0,0;0,0,0.00896807568852319,0;0,0,0,0.00133350406617509];
%         Q_full = [Q_local,zeros(4,2);zeros(2,4),diag([00126659154525236,00126659154525236])];
        A_a = [1 0 Delta_time 0 1/2*Delta_time^2 0;
               0 1 0 Delta_time 0 1/2*Delta_time^2;
               0 0 1 0          Delta_time       0;
               0 0 0 1             0        Delta_time;
               0 0 0 0             1             0;
               0 0 0 0             0             1];
        
       Motion_model.fx = @(m,contrl) A_a*m;
       Motion_model.JFx = @(m,contrl) A_a;
       Obser_model.hx = @(m) m(5:6,1);
       Obser_model.JHx = @(m) [zeros(2,4),eye(2)];
       
       Observation = [Acceleration.Y(i,1);Acceleration.X(i,1)];
       IMU_result.Q = Q_full;
       IMU_result = EKF(IMU_result , i , Observation, Delta_time, Motion_model, Obser_model,[]);
       
       Delta_time = result(index,1).Delta_time(i,1);
       A = [1 0 Delta_time 0;
            0 1 0 Delta_time;
            0 0 1 0;
            0 0 0 1];
       Motion_model.fx = @(m,contrl) A*m;
       Motion_model.JFx = @(m,contrl) A;
       Obser_model.hx = @(m) [norm(m(1:2,1)'); atan2(m(2,1), m(1,1))];
       Obser_model.JHx = @Hx_J_los;
       Observation =   [result(index,1).los_d.data(i,1);
                         result(index,1).los_phi.data(i,1)];
       Los_result(index,1).Q = Q_full(1:4,1:4);      
       Los_result(index,1) = EKF(Los_result(index,1) , i , Observation, Delta_time, Motion_model, Obser_model,[2]);
       
       A_expend = [1 0 Delta_time 0       0  0 ;
                   0 1     0  Delta_time  0  0 ;
                   0 0     1      0       0  0 ;
                   0 0     0      1       0  0 ;
                   0 0     0      0       1  0 ;
                   0 0     0      0       0  1 ];
       Motion_model.fx = @(m,contrl) A_expend*m;
       Motion_model.JFx = @(m,contrl) A_expend;
       Obser_model.hx = @h_mirror;
       Obser_model.JHx = @Hx_J_mirror;
       Observation =   [result(index,1).mpc_d.data(i,1);
                        result(index,1).mpc_phi.data(i,1)];
       Mpc_result(index,1).Q = [Q_full(1:4,1:4),zeros(4,2);zeros(2,4),zeros(2,2)]; 
       Mpc_result(index,1) = EKF(Mpc_result(index,1) , i , Observation, Delta_time, Motion_model, Obser_model,[2]);
       
       %% 数据融合
       if (i <= 194 && i>=130)
           P_MPC = Mpc_result(index,1).P{i}(1:4,1:4);

           Pg = (  P_MPC^(-1))^(-1);
 
           X3 = (P_MPC^(-1)) * Mpc_result(index,1).m(i,1:4)';
           Xg_hat = Pg * (X3);

           IMU_result.m(i,1:4) = Xg_hat';
           Los_result(index,1).m(i,1:4) = Xg_hat';
           Mpc_result(index,1).m(i,1:4) = Xg_hat';

           IMU_result.P{i}(1:4,1:4) = 1000000000000*Pg;
           Los_result(index,1).P{i}(1:4,1:4) = 10000000000000*Pg;
           Mpc_result(index,1).P{i}(1:4,1:4)  = 1* Pg;

           Avg_result.m(i,:) = Xg_hat';
           Avg_result.P{i} = Pg;
           
       else
           
           P_IMU  = IMU_result.P{i}(1:4,1:4);
           P_LOS = Los_result(index,1).P{i}(1:4,1:4);
           P_MPC = Mpc_result(index,1).P{i}(1:4,1:4);

           Pg = (P_IMU^(-1) + P_LOS^(-1) +  P_MPC^(-1))^(-1);
           X1 = (P_IMU^(-1)) * IMU_result.m(i,1:4)';
           X2 = (P_LOS^(-1)) * Los_result(index,1).m(i,1:4)';
           X3 = (P_MPC^(-1)) * Mpc_result(index,1).m(i,1:4)';
           Xg_hat = Pg * (X1 + X2 + X3);

           IMU_result.m(i,1:4) = Xg_hat';
           Los_result(index,1).m(i,1:4) = Xg_hat';
           Mpc_result(index,1).m(i,1:4) = Xg_hat';

           IMU_result.P{i}(1:4,1:4) = 1/beta*Pg;
           Los_result(index,1).P{i}(1:4,1:4) = 1/beta*Pg;
           Mpc_result(index,1).P{i}(1:4,1:4)  = 1/beta* Pg;

           Avg_result.m(i,:) = Xg_hat';
           Avg_result.P{i} = Pg;
       end

           


 end

close all;
M = moviein(useful_num);
a(2,:) = -1:0.001:0;
a(1,:) = 1;
b(1,:) = 1:0.001:2;
b(2,:) = 0;
ab = [a b];
Base(1,1) = 0;
Base(2,1) = 0;
c1(2,:) = -0.2:0.01:0.2;
c1(1,:) = 0.8;

figure(2);
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

   axis ([-0.5 3 -1.5 1]);
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

% 
% 
% 
% 
% 
% 
% 








